// sram_wr_ctrl.sv
// ---------------
// Stage 2C: single-pixel SRAM write controller, 100 MHz memory domain.
//
// Pops one command at a time from the 48-bit command FIFO, maps the raw
// protocol address onto an SRAM word address and byte lane, and drives all
// three channel SRAMs on the same cycle.
//
// -----------------------------------------------------------------------
// COMMAND LAYOUT (as written by chip_top on the 130 MHz side)
// -----------------------------------------------------------------------
//   cmd_rd_data[47:24] = cmd_addr  -- raw 24-bit linear PIXEL index
//   cmd_rd_data[23: 0] = cmd_pixel = {R[7:0], G[7:0], B[7:0]}
//
// -----------------------------------------------------------------------
// BYTE-LANE ORIENTATION -- the one thing most likely to be wrong
// -----------------------------------------------------------------------
// Each 32-bit SRAM word packs four consecutive pixels of ONE colour channel.
// The LOWEST pixel index sits in the MOST significant byte:
//
//   pixel index % 4 == 0  ->  word bits [31:24]  ->  wr_be[3]
//   pixel index % 4 == 1  ->  word bits [23:16]  ->  wr_be[2]
//   pixel index % 4 == 2  ->  word bits [15: 8]  ->  wr_be[1]
//   pixel index % 4 == 3  ->  word bits [ 7: 0]  ->  wr_be[0]
//
// so the byte enable is  1 << (PIX_PER_WORD-1 - lane),  NOT  1 << lane.
//
// This orientation was established by measurement, not by reading comments.
// The comments in rom_sequencer.sv lines 84-87 label bits[31:24] as
// "Pixel 3" and bits[7:0] as "Pixel 0" -- THOSE COMMENTS ARE BACKWARDS.
// pixels[] are pushed in index order 0,1,2,3, so pixels[0], taken from
// bits[31:24], is the leftmost pixel. Confirmed against the captured image:
// red word 4392 is 0x3D0000FF, and the captured PNG at row 68 cols 160..163
// reads 61, 0, 0, 255 -- matching [31:24] first.
//
// Getting this backwards does not fail loudly. It silently writes the wrong
// pixel within the correct word, which only shows up as a mislocated pixel
// in a captured image.
//
// -----------------------------------------------------------------------
// WRITE DATA
// -----------------------------------------------------------------------
// Each channel byte is REPLICATED across all four lanes of its 32-bit word.
// The byte enable alone then selects the destination, so no lane mux is
// needed and the value is correct whichever lane fires.
//
// -----------------------------------------------------------------------
// TIMING -- two-deep pipeline, no FSM
// -----------------------------------------------------------------------
// async_fifo registers rd_data unconditionally from the PRE-INCREMENT
// pointer, so asserting rd_en in cycle N both advances the pointer and loads
// rd_data with the entry just consumed, visible in cycle N+1. Back-to-back
// pops therefore deliver distinct entries in order with NO gap required.
//
//   cycle N    cmd_rd_en = 1                     (combinational)
//   cycle N+1  pop_q = 1, cmd_rd_data valid      (derive and register)
//   cycle N+2  wr_en = 1, SRAM commits at the closing edge
//
// pop_q is registered from the ACCEPTED pop (cmd_rd_en && !cmd_empty), not
// from cmd_empty directly: popping the last entry makes cmd_empty assert in
// exactly the cycle the final word is on rd_data, so gating the capture on
// live cmd_empty would discard it.
//
// -----------------------------------------------------------------------
// OUT-OF-RANGE COMMANDS
// -----------------------------------------------------------------------
// cmd_addr is 24 bits but only 0..(IMG_WIDTH*IMG_HEIGHT-1) is meaningful.
// An out-of-range command is POPPED AND DISCARDED with wr_en suppressed, and
// sets a sticky reject flag. It is deliberately not left in the FIFO, which
// would block every later command behind a host protocol error -- but it is
// not dropped silently either.

`timescale 1ns/1ps

module sram_wr_ctrl
    import memory_pkg::*;
#(
    parameter int CMD_W        = 48,   // {addr[23:0], pixel[23:0]}
    parameter int CMD_ADDR_W   = 24,
    parameter int CMD_PIX_W    = 24,
    parameter int ADDR_W       = SRAM_ADDR_WIDTH,   // 14
    parameter int DATA_W       = SRAM_DATA_WIDTH,   // 32
    parameter int PIX_PER_WORD = PIXELS_PER_WORD    // 4
)(
    input  logic clk,
    input  logic rst_n,

    // ---- command FIFO read side (100 MHz) -----------------------------
    input  logic             cmd_empty,
    input  logic [CMD_W-1:0] cmd_rd_data,
    output logic             cmd_rd_en,

    // ---- arbitration ---------------------------------------------------
    input  logic             wr_allowed,   // from mem_interlock
    output logic             wr_busy,      // a command is in the pipeline

    // ---- SRAM write port, shared by all three channel SRAMs -----------
    output logic                wr_en,
    output logic [DATA_W/8-1:0] wr_be,
    output logic [ADDR_W-1:0]   wr_addr,
    output logic [DATA_W-1:0]   wr_data_r,
    output logic [DATA_W-1:0]   wr_data_g,
    output logic [DATA_W-1:0]   wr_data_b,

    // ---- observability -------------------------------------------------
    output logic wr_seen,      // sticky: at least one pixel written
    output logic wr_rejected   // sticky: at least one command discarded
);

    localparam int NUM_BE     = DATA_W / 8;                   // 4
    localparam int LANE_W     = $clog2(PIX_PER_WORD);         // 2
    localparam int NUM_PIXELS = IMG_WIDTH * IMG_HEIGHT;       // 65536

    // NUM_BE and PIX_PER_WORD are the same number here (one byte per pixel
    // per channel). The byte-enable shift below assumes that.

    // -----------------------------------------------------------------
    // Field extraction from the head-of-queue command.
    // Valid during pop_q; garbage otherwise.
    // -----------------------------------------------------------------
    logic [CMD_ADDR_W-1:0] cmd_addr;
    logic [CMD_PIX_W-1:0]  cmd_pixel;

    assign cmd_addr  = cmd_rd_data[CMD_W-1 -: CMD_ADDR_W];  // [47:24]
    assign cmd_pixel = cmd_rd_data[CMD_PIX_W-1:0];          // [23:0]

    // -----------------------------------------------------------------
    // Address mapping
    // -----------------------------------------------------------------
    logic                in_range;
    logic [LANE_W-1:0]   byte_lane;
    logic [ADDR_W-1:0]   word_addr;
    logic [NUM_BE-1:0]   lane_be;

    // Compared against the pixel count rather than testing high bits, so
    // this survives a non-power-of-two image geometry.
    assign in_range  = (cmd_addr < CMD_ADDR_W'(NUM_PIXELS));

    assign byte_lane = cmd_addr[LANE_W-1:0];
    assign word_addr = cmd_addr[LANE_W+ADDR_W-1 : LANE_W];   // [15:2]

    // 1 << (PIX_PER_WORD-1 - lane). See the orientation note in the header:
    // lane 0 is the MOST significant byte, so this is a REVERSED shift.
    // Computed via an explicitly LANE_W-wide intermediate so the subtraction
    // stays in 2-bit arithmetic rather than promoting to 32-bit int.
    //
    //   lane 0 -> rev 3 -> 4'b1000     lane 2 -> rev 1 -> 4'b0010
    //   lane 1 -> rev 2 -> 4'b0100     lane 3 -> rev 0 -> 4'b0001
    logic [LANE_W-1:0] rev_lane;
    assign rev_lane = LANE_W'(PIX_PER_WORD - 1) - byte_lane;
    assign lane_be  = NUM_BE'(1) << rev_lane;

    // -----------------------------------------------------------------
    // Pop. Combinational, gated by the interlock.
    //
    // Deliberately NOT fed back into wr_busy: doing so would close a
    // combinational loop cmd_rd_en -> wr_busy -> wr_pending -> read_go ->
    // read_active -> wr_allowed -> cmd_rd_en. The cycle in which cmd_rd_en
    // is high is already covered by !cmd_empty on the interlock side.
    // -----------------------------------------------------------------
    assign cmd_rd_en = wr_allowed && !cmd_empty;

    logic pop_q;

    // A command occupies the pipeline from the capture cycle until the
    // write cycle inclusive.
    assign wr_busy = pop_q || wr_en;

    // -----------------------------------------------------------------
    // Capture, map and drive
    // -----------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pop_q       <= 1'b0;
            wr_en       <= 1'b0;
            wr_be       <= '0;
            wr_addr     <= '0;
            wr_data_r   <= '0;
            wr_data_g   <= '0;
            wr_data_b   <= '0;
            wr_seen     <= 1'b0;
            wr_rejected <= 1'b0;
        end else begin
            pop_q <= cmd_rd_en && !cmd_empty;

            // Default: no write. Overridden below for an in-range command,
            // which makes wr_en a clean one-cycle pulse per command.
            wr_en <= 1'b0;

            if (pop_q) begin
                wr_addr   <= word_addr;
                wr_be     <= lane_be;

                // Replicate each channel byte across all four lanes; wr_be
                // selects which one actually lands.
                wr_data_r <= {PIX_PER_WORD{cmd_pixel[23:16]}};
                wr_data_g <= {PIX_PER_WORD{cmd_pixel[15: 8]}};
                wr_data_b <= {PIX_PER_WORD{cmd_pixel[ 7: 0]}};

                wr_en     <= in_range;

                if (in_range) wr_seen     <= 1'b1;
                else          wr_rejected <= 1'b1;
            end
        end
    end

    // -----------------------------------------------------------------
    // Simulation-only checks
    // -----------------------------------------------------------------
`ifndef SYNTHESIS
    // The interlock must never allow a pop while a read is in progress.
    a_no_pop_while_blocked: assert property (
        @(posedge clk) disable iff (!rst_n)
        cmd_rd_en |-> wr_allowed
    ) else $error("%m: popped a command while writes were disallowed");

    // Exactly one byte lane per write.
    a_one_hot_be: assert property (
        @(posedge clk) disable iff (!rst_n)
        wr_en |-> $onehot(wr_be)
    ) else $error("%m: wr_be = %b is not one-hot", wr_be);
`endif

endmodule : sram_wr_ctrl
