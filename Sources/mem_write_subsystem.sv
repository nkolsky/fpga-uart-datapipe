// mem_write_subsystem.sv
// ======================
// The complete memory-side write path: messages in, SRAM writes out.
//
//   msg -> mem_msg_writer -> pixel_word_packer -> sram_rmw -> rgb_sram x3
//
// -----------------------------------------------------------------------
// WHAT EACH STAGE DOES
// -----------------------------------------------------------------------
//   mem_msg_writer     decodes msg_kind. A burst header opens a rectangle;
//                      burst data feeds it; a single pixel write is a 1x1
//                      rectangle. Reads and register writes are dropped --
//                      they are handled elsewhere on this side.
//
//   pixel_word_packer  accumulates pixels into 32-bit words and issues one
//                      request per word, with a mask saying which lanes it
//                      covers. Flushes when a word fills, at the end of a
//                      rectangle row, and when the rectangle completes.
//
//   sram_rmw           performs the update. A full-word request is a plain
//                      write; a partial one is read-modify-write, because
//                      rgb_sram has no per-byte write enable.
//
// -----------------------------------------------------------------------
// WHY THE PIPELINE IS SHAPED THIS WAY
// -----------------------------------------------------------------------
// A burst data message carries exactly FOUR pixels, and four pixels are
// exactly one 32-bit word per colour channel. Grouped, a message costs three
// plain writes -- one per channel -- and no reads at all.
//
// The previous design split each message into four single-pixel commands.
// Without byte enables every one of those is a read-modify-write, so the same
// message would cost four reads and four writes per channel: twenty-four SRAM
// accesses instead of three, to deliver data that arrived already assembled.
//
// -----------------------------------------------------------------------
// BACK-PRESSURE IS CONTINUOUS FROM THE SRAM TO THE PC
// -----------------------------------------------------------------------
// Every stage here carries ready/valid, and msg_ready reaches back through
// the crossing to rx_classifier, rx_mac and finally UART_CTS. If the
// interlock gives the port to a reader, the whole chain stalls rather than
// dropping anything.

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

    // ---- message in, from cdc_msg_sync -----------------------------------
    input  logic              msg_valid,
    output logic              msg_ready,
    input  msg_kind_t         msg_kind,
    input  msg_payload_t      msg_payload,

    // ---- SRAM port --------------------------------------------------------
    output logic              rd_en,
    output logic [ADDR_W-1:0] rd_addr,
    input  logic [DATA_W-1:0] rd_data_r,
    input  logic [DATA_W-1:0] rd_data_g,
    input  logic [DATA_W-1:0] rd_data_b,

    output logic              wr_en,
    output logic [ADDR_W-1:0] wr_addr,
    output logic [DATA_W-1:0] wr_data_r,
    output logic [DATA_W-1:0] wr_data_g,
    output logic [DATA_W-1:0] wr_data_b,

    // ---- arbitration, to mem_interlock -----------------------------------
    output logic              port_req,
    input  logic              port_grant,
    output logic              wr_busy,          // mid read-modify-write

    // ---- status -----------------------------------------------------------
    output logic              rect_busy,        // a rectangle is in progress
    output logic              rmw_count_pulse,  // one per read-modify-write
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

    // ---- packer -> rmw ---------------------------------------------------
    logic              req_valid, req_ready;
    logic [ADDR_W-1:0] req_addr;
    logic [NLANE-1:0]  req_be;
    logic [DATA_W-1:0] req_dr, req_dg, req_db;
    logic              pack_done;

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
        .wr_valid    (req_valid),
        .wr_ready    (req_ready),
        .wr_addr     (req_addr),
        .wr_be       (req_be),
        .wr_data_r   (req_dr),
        .wr_data_g   (req_dg),
        .wr_data_b   (req_db),
        .busy        (rect_busy),
        .done        (pack_done)
    );

    // ---- rmw -> SRAM -----------------------------------------------------
    sram_rmw u_rmw (
        .clk             (clk),
        .rst_n           (rst_n),
        .req_valid       (req_valid),
        .req_ready       (req_ready),
        .req_addr        (req_addr),
        .req_be          (req_be),
        .req_data_r      (req_dr),
        .req_data_g      (req_dg),
        .req_data_b      (req_db),
        .rd_en           (rd_en),
        .rd_addr         (rd_addr),
        .rd_data_r       (rd_data_r),
        .rd_data_g       (rd_data_g),
        .rd_data_b       (rd_data_b),
        .wr_en           (wr_en),
        .wr_addr         (wr_addr),
        .wr_data_r       (wr_data_r),
        .wr_data_g       (wr_data_g),
        .wr_data_b       (wr_data_b),
        .port_req        (port_req),
        .port_grant      (port_grant),
        .busy            (wr_busy),
        .rmw_count_pulse (rmw_count_pulse)
    );

endmodule : mem_write_subsystem
