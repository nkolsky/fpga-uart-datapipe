// pixel_word_packer.sv
// ====================
// Turns a stream of pixels into MASKED WORD WRITES.
//
// This is the shared write infrastructure. A single pixel write and an image
// burst write are the same operation here, differing only in the byte enable
// that comes out:
//
//   single pixel        a 1x1 rectangle  ->  one write, be one-hot   (0010)
//   burst, aligned      4 pixels/word    ->  one write, be = 1111
//   burst, unaligned    straddles words  ->  two writes, 0111 then 1000
//
// -----------------------------------------------------------------------
// WHY THIS EXISTS -- THE COST IS THE ACCESS, NOT THE MASK
// -----------------------------------------------------------------------
// A BRAM write activates a whole row: word line drive, bit line precharge and
// the sense amplifiers all happen regardless of how many byte lanes are
// enabled. Only the per-column write drivers are gated by the byte enable. So
// a masked write costs very nearly what a full-word write costs.
//
// The previous design split every burst data message into four separate
// single-pixel commands, each writing ONE lane of the SAME word. That is four
// row activations to fill one word -- roughly four times the energy for
// identical data, on the traffic that dominates the system.
//
// Four pixels is exactly one 32-bit word per colour channel, and a burst data
// message carries exactly four pixels. The data arrives already assembled;
// this module stops taking it apart.
//
// -----------------------------------------------------------------------
// ROW-END FLUSH -- THE EASY THING TO GET WRONG
// -----------------------------------------------------------------------
// A rectangle narrower than the image is NOT contiguous in linear address.
// A 2x4 burst at the origin of an 8-wide image covers
//
//     row 0, cols 0..3  ->  linear 0, 1, 2, 3     word 0
//     row 1, cols 0..3  ->  linear 8, 9, 10, 11   word 2
//
// The address jumps between rows. So the accumulator MUST be flushed at the
// end of every rectangle row, not only when four pixels have been gathered.
// Assume contiguity and full-image bursts work while every sub-rectangle
// silently corrupts the words either side of the gap.
//
// -----------------------------------------------------------------------
// LANE ORIENTATION -- LANE 0 IS THE MOST SIGNIFICANT BYTE
// -----------------------------------------------------------------------
//     linear addr 4 -> word 1, lane 0 -> word[31:24], be[3]
//     linear addr 5 -> word 1, lane 1 -> word[23:16], be[2]
//     linear addr 6 -> word 1, lane 2 -> word[15: 8], be[1]
//     linear addr 7 -> word 1, lane 3 -> word[ 7: 0], be[0]
//
// So the byte-enable bit index and the data byte position are the SAME index,
// 3 - lane. This matches sram_wr_ctrl's existing convention exactly; changing
// it here would shuffle pixels within each group of four.
//
// -----------------------------------------------------------------------
// FLOW CONTROL
// -----------------------------------------------------------------------
// pixel_ready falls while a write is waiting to be accepted, so a stalled
// memory side back-pressures all the way to the source. A pixel is never
// dropped and never double-counted.

`timescale 1ns/1ps

module pixel_word_packer
    import memory_pkg::*;
#(
    // Derived like the three below it, which already come from memory_pkg.
    // It was the only literal in the group.
    parameter int PIX_W  = CHANNEL_WIDTH,           // bits per channel
    parameter int ADDR_W = SRAM_ADDR_WIDTH,         // word address width
    parameter int DATA_W = SRAM_DATA_WIDTH,         // 32
    parameter int NLANE  = PIXELS_PER_WORD          // 4
)(
    input  logic clk,
    input  logic rst_n,

    // ---- rectangle start -----------------------------------------------
    // One pulse per write operation. For a single pixel write, height and
    // width are both 1 and base_addr is the pixel address.
    input  logic                 start,
    input  logic [23:0]          base_addr,   // linear pixel address
    input  logic [9:0]           height,      // rows,    >= 1
    input  logic [9:0]           width,       // columns, >= 1

    // ---- pixel stream --------------------------------------------------
    // Pixels arrive in rectangle order: left to right, then top to bottom.
    input  logic                 pixel_valid,
    output logic                 pixel_ready,
    input  logic [PIX_W-1:0]     pixel_r,
    input  logic [PIX_W-1:0]     pixel_g,
    input  logic [PIX_W-1:0]     pixel_b,

    // ---- masked word write out -----------------------------------------
    output logic                 wr_valid,
    input  logic                 wr_ready,
    output logic [ADDR_W-1:0]    wr_addr,
    output logic [NLANE-1:0]     wr_be,
    output logic [DATA_W-1:0]    wr_data_r,
    output logic [DATA_W-1:0]    wr_data_g,
    output logic [DATA_W-1:0]    wr_data_b,

    // ---- status ---------------------------------------------------------
    output logic                 busy,        // a rectangle is in progress
    output logic                 done         // one pulse, rectangle complete
);

    localparam int LANE_W = $clog2(NLANE);      // 2

    // A single set lane, typed UNSIGNED and already the width of wr_be.
    // Writing NLANE'(1) instead makes the literal SIGNED, because NLANE is
    // an int, and the OR below then mixes signed and unsigned operands.
    localparam logic [NLANE-1:0] LANE_ONE = {{(NLANE-1){1'b0}}, 1'b1};

    // -------------------------------------------------------------------
    // Position within the image and within the rectangle
    // -------------------------------------------------------------------
    logic [23:0] base_row, base_col;            // rectangle origin
    logic [9:0]  rows_left, cols_left;          // remaining in rectangle
    logic [9:0]  rect_width;                    // reload value for cols_left
    logic [23:0] cur_lin;                       // current linear pixel address

    logic [LANE_W-1:0] lane;
    logic [ADDR_W-1:0] word_addr;

    assign lane      = cur_lin[LANE_W-1:0];
    assign word_addr = ADDR_W'(cur_lin >> LANE_W);

    // Index used for BOTH the byte-enable bit and the data byte position.
    logic [LANE_W-1:0] idx;
    assign idx = LANE_W'(NLANE - 1) - lane;

    // -------------------------------------------------------------------
    // Accumulator. Holds the word being assembled plus the lanes written
    // into it so far.
    // -------------------------------------------------------------------
    logic [DATA_W-1:0] acc_r, acc_g, acc_b;
    logic [NLANE-1:0]  acc_be;
    logic [ADDR_W-1:0] acc_addr;

    // -------------------------------------------------------------------
    // Handshakes
    //
    // A pixel is accepted only when the output is free, so an accumulated
    // word can never be overwritten before it has been issued.
    // -------------------------------------------------------------------
    logic accept_pixel;
    logic flush_now;

    assign pixel_ready  = busy && (!wr_valid || wr_ready);
    assign accept_pixel = pixel_valid && pixel_ready;

    // Flush when the word is full, at the end of a rectangle row, or when the
    // rectangle finishes. The row-end term is what makes narrow rectangles
    // work: the linear address jumps between rows.
    assign flush_now = accept_pixel &&
                       ((lane == LANE_W'(NLANE-1)) ||        // word full
                        (cols_left == 10'd1));               // row ends here

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            busy       <= 1'b0;
            done       <= 1'b0;
            wr_valid   <= 1'b0;
            wr_addr    <= '0;
            wr_be      <= '0;
            wr_data_r  <= '0;
            wr_data_g  <= '0;
            wr_data_b  <= '0;
            acc_r      <= '0;
            acc_g      <= '0;
            acc_b      <= '0;
            acc_be     <= '0;
            acc_addr   <= '0;
            cur_lin    <= '0;
            base_row   <= '0;
            base_col   <= '0;
            rows_left  <= '0;
            cols_left  <= '0;
            rect_width <= '0;
        end else begin
            done <= 1'b0;

            // Clear the output once it has been taken.
            if (wr_valid && wr_ready) wr_valid <= 1'b0;

            // ---- rectangle start ---------------------------------------
            if (start && !busy) begin
                busy       <= 1'b1;
                cur_lin    <= base_addr;
                base_row   <= base_addr / 24'(IMG_WIDTH);
                base_col   <= base_addr % 24'(IMG_WIDTH);
                rows_left  <= height;
                cols_left  <= width;
                rect_width <= width;
                acc_be     <= '0;
            end

            // ---- absorb a pixel ----------------------------------------
            if (accept_pixel) begin
                // First lane of a new word fixes its address.
                if (acc_be == '0) acc_addr <= word_addr;

                acc_r[idx*8 +: 8] <= pixel_r;
                acc_g[idx*8 +: 8] <= pixel_g;
                acc_b[idx*8 +: 8] <= pixel_b;
                acc_be[idx]       <= 1'b1;

                // ---- advance position ----------------------------------
                if (cols_left == 10'd1) begin
                    // End of a rectangle row: jump to the next row's start
                    // column, NOT simply the next linear address.
                    cols_left <= rect_width;
                    rows_left <= rows_left - 10'd1;
                    cur_lin   <= (base_row + 24'(height) - 24'(rows_left) + 24'd1)
                                 * 24'(IMG_WIDTH) + base_col;
                end else begin
                    cols_left <= cols_left - 10'd1;
                    cur_lin   <= cur_lin + 24'd1;
                end
            end

            // ---- issue the word ----------------------------------------
            if (flush_now) begin
                wr_valid  <= 1'b1;
                // acc_addr has not been written yet this cycle when this is
                // the first lane of a word, so take word_addr directly.
                wr_addr   <= (acc_be == '0) ? word_addr : acc_addr;

                wr_be     <= acc_be | (LANE_ONE << idx);

                wr_data_r <= acc_r; wr_data_r[idx*8 +: 8] <= pixel_r;
                wr_data_g <= acc_g; wr_data_g[idx*8 +: 8] <= pixel_g;
                wr_data_b <= acc_b; wr_data_b[idx*8 +: 8] <= pixel_b;

                // Start the next word empty.
                acc_be    <= '0;
                acc_r     <= '0;
                acc_g     <= '0;
                acc_b     <= '0;
            end

            // ---- rectangle complete ------------------------------------
            if (accept_pixel && (cols_left == 10'd1) && (rows_left == 10'd1)) begin
                busy <= 1'b0;
                done <= 1'b1;
            end
        end
    end

`ifndef SYNTHESIS
    // A write is never issued with no lanes enabled.
    a_be_nonzero: assert property (
        @(posedge clk) disable iff (!rst_n)
        wr_valid |-> (wr_be != '0)
    ) else $error("%m: word write issued with an empty byte enable");

    // Pixels are only taken while a rectangle is in progress.
    a_pixel_only_when_busy: assert property (
        @(posedge clk) disable iff (!rst_n)
        accept_pixel |-> busy
    ) else $error("%m: pixel accepted outside a rectangle");
`endif

endmodule : pixel_word_packer
