// mem_write_subsystem.sv
// ======================
// The memory-side write path: messages in, SRAM writes out.
//
//   msg -> mem_msg_writer -> pixel_word_packer -> rgb_sram x3
//
// -----------------------------------------------------------------------
// WHAT EACH STAGE DOES
// -----------------------------------------------------------------------
//   mem_msg_writer     decodes msg_kind. A burst header opens a rectangle;
//                      burst data feeds it; a single pixel write is a 1x1
//                      rectangle. Reads and register writes never reach
//                      here -- mem_msg_router sends them elsewhere.
//
//   pixel_word_packer  accumulates pixels into 32-bit words and issues one
//                      write per word, with a BYTE ENABLE saying which
//                      lanes it covers. Flushes when a word fills, at the
//                      end of a rectangle row, and when the rectangle ends.
//
// -----------------------------------------------------------------------
// BYTE ENABLES, NOT READ-MODIFY-WRITE
// -----------------------------------------------------------------------
// One 32-bit word holds FOUR pixels of one colour channel, so changing a
// single pixel means changing one byte of a word. BRAM provides a per-byte
// write enable in hardware, so that costs ONE WRITE AND NO READ: the lanes
// whose enable is low are simply not driven and keep their contents.
//
// Read-modify-write would be the alternative and is the wrong answer here.
// It needs a read, so the memory turn-around time gets in the way, it makes
// the write path contend for the read port that the image and pixel readers
// already share, and it opens a hazard where two updates to the same word
// both read the same stale value.
//
// -----------------------------------------------------------------------
// WHY GROUPING BURST PIXELS STILL MATTERS
// -----------------------------------------------------------------------
// A BRAM write activates a whole row -- word line drive, bit line
// precharge, sense amplifiers -- regardless of how many byte lanes are
// enabled. THE COST IS THE ACCESS, NOT THE MASK.
//
// A burst data message carries exactly four pixels, and four pixels are
// exactly one word per channel. Grouped, a message is THREE writes with the
// enable all ones. The previous design split each message into four
// single-pixel commands and wrote one lane at a time: twelve accesses to
// deliver data that arrived already assembled.

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
    output logic              wr_en,
    output logic [NLANE-1:0]  wr_be,
    output logic [ADDR_W-1:0] wr_addr,
    output logic [DATA_W-1:0] wr_data_r,
    output logic [DATA_W-1:0] wr_data_g,
    output logic [DATA_W-1:0] wr_data_b,

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
        .wr_ready    (wr_allowed),
        .wr_addr     (wr_addr),
        .wr_be       (wr_be),
        .wr_data_r   (wr_data_r),
        .wr_data_g   (wr_data_g),
        .wr_data_b   (wr_data_b),
        .busy        (rect_busy),
        .done        (pack_done)
    );

    assign wr_en = pack_wr_valid && wr_allowed;

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
