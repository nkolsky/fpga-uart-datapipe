// -----------------------------------------------------------------------------
// mem_write_subsystem.sv
//
// Write path from the message router to the SRAM banks. The writer decodes
// incoming memory commands, packs pixels into 32-bit words, and issues masked
// byte writes to the RGB SRAMs.
//
// The important point is that this path writes only the enabled byte lanes. It
// does not do a read-modify-write, so it does not contend for the read port or
// disturb unrelated pixels in the same word.
// -----------------------------------------------------------------------------

`timescale 1ns/1ps

module mem_write_subsystem
    import msg_format_pkg::*;
    import memory_pkg::*;
#(
    parameter int ADDR_W = SRAM_ADDR_WIDTH,
    parameter int DATA_W = SRAM_DATA_WIDTH,
    parameter int NLANE  = PIXELS_PER_WORD
)(
    input  logic clk,                 // 100 MHz memory domain
    input  logic rst_n,

    // ---- message in, from mem_msg_router ---------------------------------
    input  logic              msg_valid,
    output logic              msg_ready,
    input  msg_kind_t         msg_kind,
    input  msg_payload_t      msg_payload,

    // ---- SRAM write port --------------------------------------------------
    // DIRECT write port. Idle while a full-image write is bursting.
    output logic              wr_en,
    output logic [NLANE-1:0]  wr_be,
    output logic [ADDR_W-1:0] wr_addr,
    output logic [DATA_W-1:0] wr_data_r,
    output logic [DATA_W-1:0] wr_data_g,
    output logic [DATA_W-1:0] wr_data_b,

    // ---- AHB burst write path (full image only) --------------------------
    // High for the whole of a full-image write, from pack_start until the
    // gather finishes. memory_subsystem uses it to switch each SRAM's write
    // port from the direct path to the AHB slaves.
    output logic              burst_active,
    output logic              ahb_req_valid,
    output logic              ahb_req_write,
    output logic              ahb_req_burst,
    output logic [1:0]        ahb_req_channel,
    output logic [13:0]       ahb_req_word,
    input  logic              ahb_busy,
    input  logic [1:0]        ahb_wr_beat,
    input  logic              ahb_wr_ack,
    output logic [DATA_W-1:0] ahb_wr_data,

    // ---- arbitration, from mem_interlock ---------------------------------
    // Writes stand off while either reader owns the memory. They do not need
    // the READ port -- byte enables mean no read -- but rgb_sram forbids a
    // read and a write to the same address in one cycle, and the readers
    // walk the whole image.
    input  logic              wr_allowed,

    // ---- status -----------------------------------------------------------
    output logic              rect_busy,        // a rectangle is in progress
    output logic              wr_rejected       // sticky: address out of image
);

    // ---- writer -> packer ------------------------------------------------
    logic        pack_start;
    logic [23:0] pack_base_addr;
    logic [9:0]  pack_height, pack_width;
    logic        pixel_valid, pixel_ready;
    logic [7:0]  pixel_r, pixel_g, pixel_b;

    mem_msg_writer u_writer (
        .clk            (clk),
        .rst_n          (rst_n),
        .msg_valid      (msg_valid),
        .msg_ready      (msg_ready),
        .msg_kind       (msg_kind),
        .msg_payload    (msg_payload),
        .pack_start     (pack_start),
        .pack_base_addr (pack_base_addr),
        .pack_height    (pack_height),
        .pack_width     (pack_width),
        .pixel_valid    (pixel_valid),
        .pixel_ready    (pixel_ready),
        .pixel_r        (pixel_r),
        .pixel_g        (pixel_g),
        .pixel_b        (pixel_b),
        .pack_busy      (rect_busy),
        .wr_rejected    (wr_rejected)
    );

    // ---- packer -> SRAM ---------------------------------------------------
    // wr_allowed is the packer's ready: while a reader owns the memory the
    // packer holds its word, which back-pressures through the writer, the
    // router, the crossing and the RX side to the PC.
    logic pack_wr_valid;
    logic pack_done;
    logic pack_wr_ready;

    pixel_word_packer u_packer (
        .clk         (clk),
        .rst_n       (rst_n),
        .start       (pack_start),
        .base_addr   (pack_base_addr),
        .height      (pack_height),
        .width       (pack_width),
        .pixel_valid (pixel_valid),
        .pixel_ready (pixel_ready),
        .pixel_r     (pixel_r),
        .pixel_g     (pixel_g),
        .pixel_b     (pixel_b),
        .wr_valid    (pack_wr_valid),
        // wr_allowed is the interlock's permission; bw_wr_ready is the
        // gather holding the packer off while three bursts drain. Both must
        // be high for a word to move.
        .wr_ready    (pack_wr_ready),
        .wr_addr     (wr_addr),
        .wr_be       (wr_be),
        .wr_data_r   (wr_data_r),
        .wr_data_g   (wr_data_g),
        .wr_data_b   (wr_data_b),
        .busy        (rect_busy),
        .done        (pack_done)
    );

// FULL IMAGE ONLY, decided once from the burst header geometry before the
// rectangle starts. Everything else -- single pixels (1x1), offset or short
// rectangles -- keeps the direct port and its byte enables untouched.
//
// The restriction is not arbitrary. AHB-Lite has no byte strobes, so a
// partial word cannot be expressed; and pixel_word_packer emits partial words
// at the end of every row of a narrow rectangle, because the linear address
// jumps by IMG_WIDTH between rows. Only a full-width rectangle produces
// complete words at consecutive addresses.
logic full_image;
assign full_image = (pack_base_addr == 24'd0) &&
                    (pack_height    == 10'(memory_pkg::IMG_HEIGHT)) &&
                    (pack_width     == 10'(memory_pkg::IMG_WIDTH));

logic burst_mode;
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)            burst_mode <= 1'b0;
    else if (pack_start)   burst_mode <= full_image;
end

logic bw_wr_ready, bw_busy;

img_burst_writer #(.ADDR_W_IN(ADDR_W)) u_burst_writer (
    .clk(clk), .rst_n(rst_n),
    .burst_mode   (burst_mode),
    .pack_busy    (rect_busy),
    .pack_done    (pack_done),
    .wr_valid     (pack_wr_valid),
    .wr_ready     (bw_wr_ready),
    .wr_addr      (wr_addr),
    .wr_be        (wr_be),
    .wr_data_r    (wr_data_r),
    .wr_data_g    (wr_data_g),
    .wr_data_b    (wr_data_b),
    .req_valid    (ahb_req_valid),
    .req_write    (ahb_req_write),
    .req_burst    (ahb_req_burst),
    .req_channel  (ahb_req_channel),
    .req_word     (ahb_req_word),
    .ahb_busy     (ahb_busy),
    .ahb_wr_beat  (ahb_wr_beat),
    .ahb_wr_ack   (ahb_wr_ack),
    .ahb_wr_data  (ahb_wr_data),
    .busy         (bw_busy)
);

assign burst_active = burst_mode && (rect_busy || bw_busy);

// The DIRECT port stays silent through a burst write -- the AHB slaves drive
// the SRAMs instead.
assign pack_wr_ready = wr_allowed && (burst_mode ? bw_wr_ready : 1'b1);

assign wr_en = pack_wr_valid && pack_wr_ready && !burst_mode;

`ifndef SYNTHESIS
    // A write is never issued with no lanes enabled.
    a_be_nonzero: assert property (
        @(posedge clk) disable iff (!rst_n)
        wr_en |-> (wr_be != '0)
    ) else $error("%m: write issued with an empty byte enable");

    // Writes never occur while a reader owns the memory.
    a_write_when_allowed: assert property (
        @(posedge clk) disable iff (!rst_n)
        wr_en |-> wr_allowed
    ) else $error("%m: write issued without permission");
`endif

endmodule : mem_write_subsystem
