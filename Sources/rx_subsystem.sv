// rx_subsystem.sv
// ---------------
// Complete 130 MHz receive path: serial line in, command-FIFO write port out.
//
//   rx_in -> rx_phy -> rx_mac -> rx_msg_parser -> rx_classifier -> cmd mux
//                                      |                              ^
//                                      +----> rx_burst_ctrl ----------+
//                                                   |
//                                                   +-> bypass_active
//                                                       (back to the parser)
//
// -----------------------------------------------------------------------
// WHO OWNS WHICH BYTE
// -----------------------------------------------------------------------
//   rx_mac         bytes 0, 5, 10, 15 -- '{', ',' and '}'. Framing only.
//   rx_msg_parser  bytes 1, 6, 11 -- opcodes. Extracts 2-4, 7-9, 12-14.
//   rx_classifier  no bytes at all. Range-checks the extracted fields.
//
// Dataflow is strictly one-directional. rx_mac finds the frame boundaries
// itself from the delimiter positions, so nothing feeds back into it: the
// parser no longer supplies expected_len, and bypass_active reaches only the
// parser, never the MAC.
//
// -----------------------------------------------------------------------
// SINGLE CLOCK DOMAIN -- 130 MHz
// -----------------------------------------------------------------------
// Everything here runs on pll_clk_out and resets from sync_pll_rst_n. There
// is NO clock-domain crossing inside this module and none may ever be added:
// every crossing in this design lives at chip_top, deliberately, so the whole
// CDC map is visible in one file. Both async FIFOs also remain in chip_top.
//
// -----------------------------------------------------------------------
// ONE FRAME BUFFER, NO COPY
// -----------------------------------------------------------------------
// rx_mac exposes frame_buf directly rather than publishing a latched copy.
// It is both the live tap the parser sees as a frame fills and the completed
// frame the classifier reads while frame_done is high. Consumers sample only
// on frame_done, at which point byte_cnt bounds exactly which bytes are real.

`timescale 1ns/1ps

module rx_subsystem
    import msg_format_pkg::*;
(
    // ---- 130 MHz UART domain -------------------------------------------
    input  logic                        clk,             // pll_clk_out
    input  logic                        rst_n,           // sync_pll_rst_n

    // ---- serial input --------------------------------------------------
    input  logic                        rx_in,           // UART_TXD_IN

    // ---- command output: plain ready/valid ----------------------------
    //
    // Deliberately NOT named after a FIFO. This module produces commands; how
    // they reach the 100 MHz domain is chip_top's business, and that crossing
    // is expected to change. cmd_ready is a handshake, not a FIFO status, so
    // these names survive the crossing being replaced.
    input  logic                        cmd_ready,
    output logic                        cmd_valid,
    output logic [BURST_ADDR_W + BURST_PIX_W - 1:0] cmd_data,

    // ---- PHY / MAC status ----------------------------------------------
    output logic                        rx_phy_busy,
    output logic                        rx_parity_err_pulse,
    output logic                        rx_mac_busy,

    // ---- burst mode ----------------------------------------------------
    output logic                        rx_burst_active,

    // ---- legacy RGF control path ---------------------------------------
    output logic                        rx_classifier_valid,
    output logic                        rx_classifier_error,
    output logic [9:0]                  rx_row_q,
    output logic [9:0]                  rx_col_q,
    output logic [PAYLOAD_W-1:0]        rx_pixel_q,

    // ---- register read / write commands --------------------------------
    output logic                        rx_rr_cmd_valid,
    output logic [rgf_pkg::ADDR_WIDTH-1:0] rx_rr_cmd_addr,
    output logic                        rx_rw_cmd_valid,
    output logic [rgf_pkg::ADDR_WIDTH-1:0] rx_rw_cmd_addr,
    output logic [rgf_pkg::DATA_WIDTH-1:0] rx_rw_cmd_data,

    // ---- single pixel read command -------------------------------------
    output logic                        rx_pr_cmd_valid,
    output logic [9:0]                  rx_pr_cmd_row,
    output logic [9:0]                  rx_pr_cmd_col,

    // ---- image burst read command --------------------------------------
    output logic                        rx_br_cmd_valid,
    output logic [9:0]                  rx_br_cmd_base_row,
    output logic [9:0]                  rx_br_cmd_base_col,
    output logic [9:0]                  rx_br_cmd_height,
    output logic [9:0]                  rx_br_cmd_width,

    // ---- diagnostics ---------------------------------------------------
    output logic                        pix_wr_seen_sticky,
    output logic                        cmd_ovf_sticky
);


// -------------------------------------------------------------------------
// Internal interconnect
// -------------------------------------------------------------------------
logic [7:0]                  rx_byte_val;
logic                        rx_byte_valid;

logic [FRAME_W-1:0]          rx_frame_buf;    // live tap AND completed frame
logic [BYTE_CNT_W-1:0]       rx_byte_cnt;
logic                        rx_frame_done;
logic                        rx_frame_err;

msg_kind_t                   rx_msg_kind;
logic [PAYLOAD_W-1:0]        rx_field0, rx_field1, rx_field2;
logic [BURST_PIX_PER_MSG-1:0][BURST_PIX_W-1:0] rx_burst_pixels;

logic                        rx_bypass_active;

logic                        rx_cmd_valid;
logic [BURST_ADDR_W-1:0]     rx_cmd_addr;
logic [BURST_PIX_W-1:0]      rx_cmd_pixel;

logic                        rx_burst_cmd_valid;
logic [BURST_ADDR_W-1:0]     rx_burst_cmd_addr;
logic [BURST_PIX_W-1:0]      rx_burst_cmd_pixel;

logic                        rx_burst_done;
logic                        rx_burst_err_hdr, rx_burst_err_data,
                             rx_burst_err_unexp;

// =========================================================================
// PHY -- bit recovery, parity, byte assembly. No protocol knowledge.
// =========================================================================
rx_phy u_rx_phy (
    .clk              (clk),
    .rst_n            (rst_n),
    .rx_in            (rx_in),
    .byte_valid       (rx_byte_valid),
    .rx_byte          (rx_byte_val),
    .parity_err_pulse (rx_parity_err_pulse),
    .rx_busy          (rx_phy_busy)
);

// =========================================================================
// MAC -- framing. Owns bytes 0, 5, 10 and 15.
// =========================================================================
rx_mac u_rx_mac (
    .clk         (clk),
    .rst_n       (rst_n),
    .byte_valid  (rx_byte_valid),
    .rx_byte     (rx_byte_val),
    .par_val_rst (rx_parity_err_pulse),
    .frame_buf   (rx_frame_buf),
    .byte_cnt    (rx_byte_cnt),
    .frame_done  (rx_frame_done),
    .frame_err   (rx_frame_err),
    .mac_busy    (rx_mac_busy)
);

// =========================================================================
// PARSER -- identification. Owns bytes 1, 6 and 11; extracts the payload.
//
// Combinational on the live buffer, so its outputs are only meaningful while
// rx_frame_done is high. Every consumer below qualifies on that.
// =========================================================================
rx_msg_parser u_rx_msg_parser (
    .frame_buf     (rx_frame_buf),
    .byte_cnt      (rx_byte_cnt),
    .bypass_active (rx_bypass_active),
    .msg_kind      (rx_msg_kind),
    .field0        (rx_field0),
    .field1        (rx_field1),
    .field2        (rx_field2),
    .burst_pixels  (rx_burst_pixels)
);

// =========================================================================
// CLASSIFIER -- routing and range checks.
// =========================================================================
rx_classifier u_rx_classifier (
    .clk                 (clk),
    .rst_n               (rst_n),

    .msg_valid           (rx_frame_done),
    .frame_err           (rx_frame_err),
    .msg_kind            (rx_msg_kind),
    .field0              (rx_field0),
    .field1              (rx_field1),
    .field2              (rx_field2),

    .burst_active        (rx_burst_active),

    .classifier_valid    (rx_classifier_valid),
    .classifier_error    (rx_classifier_error),
    .row_q               (rx_row_q),
    .col_q               (rx_col_q),
    .pixel_q             (rx_pixel_q),

    .cmd_valid           (rx_cmd_valid),
    .cmd_addr            (rx_cmd_addr),
    .cmd_pixel           (rx_cmd_pixel),

    .rr_cmd_valid        (rx_rr_cmd_valid),
    .rr_cmd_addr         (rx_rr_cmd_addr),
    .rw_cmd_valid        (rx_rw_cmd_valid),
    .rw_cmd_addr         (rx_rw_cmd_addr),
    .rw_cmd_data         (rx_rw_cmd_data),

    .pr_cmd_valid        (rx_pr_cmd_valid),
    .pr_cmd_row          (rx_pr_cmd_row),
    .pr_cmd_col          (rx_pr_cmd_col),

    .br_cmd_valid        (rx_br_cmd_valid),
    .br_cmd_base_row     (rx_br_cmd_base_row),
    .br_cmd_base_col     (rx_br_cmd_base_col),
    .br_cmd_height       (rx_br_cmd_height),
    .br_cmd_width        (rx_br_cmd_width)
);

// =========================================================================
// BURST CONTROLLER -- owns burst mode and generates addresses.
//
// height and width come from the header's second and third payload slots;
// pixels from the parser's dedicated burst output, since burst payload spans
// the delimiters and does not fit the three-field model.
// =========================================================================
rx_burst_ctrl u_rx_burst_ctrl (
    .clk              (clk),
    .rst_n            (rst_n),
    .msg_valid        (rx_frame_done),
    .msg_kind         (rx_msg_kind),
    .height           (rx_field1),
    .width            (rx_field2),
    .pixels           (rx_burst_pixels),
    .frame_err        (rx_frame_err),
    .burst_abort      (1'b0),
    .cmd_ready        (cmd_ready),
    .cmd_valid        (rx_burst_cmd_valid),
    .cmd_addr         (rx_burst_cmd_addr),
    .cmd_pixel        (rx_burst_cmd_pixel),
    .bypass_active    (rx_bypass_active),
    .burst_active     (rx_burst_active),
    .burst_done       (rx_burst_done),
    .err_hdr_invalid  (rx_burst_err_hdr),
    .err_data_invalid (rx_burst_err_data),
    .err_unexpected   (rx_burst_err_unexp)
);

// =========================================================================
// COMMAND FIFO WRITE MUX
//
// Two producers of 48-bit {address, pixel} commands: rx_classifier (single
// pixel write) and rx_burst_ctrl (burst write, one pixel per clock). They
// are mutually exclusive by construction -- the classifier goes inert
// whenever burst_active is high -- and the assertion below checks it.
// =========================================================================
assign cmd_valid = rx_burst_cmd_valid || rx_cmd_valid;
assign cmd_data  = rx_burst_cmd_valid
                 ? {rx_burst_cmd_addr, rx_burst_cmd_pixel}
                 : {rx_cmd_addr,       rx_cmd_pixel};

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)                  pix_wr_seen_sticky <= 1'b0;
    else if (rx_cmd_valid)       pix_wr_seen_sticky <= 1'b1;
end

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)                                  cmd_ovf_sticky <= 1'b0;
    else if (cmd_valid && !cmd_ready)            cmd_ovf_sticky <= 1'b1;
end

`ifndef SYNTHESIS
    a_no_cmd_overlap: assert property (
        @(posedge clk) disable iff (!rst_n)
        !(rx_burst_cmd_valid && rx_cmd_valid)
    ) else $error("%m: burst and single-pixel commands overlapped");

    a_cls_cmd_not_dropped: assert property (
        @(posedge clk) disable iff (!rst_n)
        rx_cmd_valid |-> cmd_ready
    ) else $error("%m: single-pixel command presented while cmd_ready was low");
`endif

endmodule : rx_subsystem