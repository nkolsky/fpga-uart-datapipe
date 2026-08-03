// -----------------------------------------------------------------------------
// command_processing_subsystem.sv
//
// Pure hierarchy refactor of the complete 130 MHz command-processing stage.
// It contains the eight receive parsers, rx_classifier, rx_burst_ctrl, the
// command-FIFO write mux, and the two write-domain sticky diagnostics.
//
// CDC elements and the async command FIFO remain in chip_top.
// -----------------------------------------------------------------------------
`timescale 1ns/1ps

module command_processing_subsystem (
    input  logic                        clk,
    input  logic                        rst_n,

    input  logic                        rx_mac_msg_valid,
    input  logic [127:0]                rx_mac_msg_data,
    input  rx_msg_pkg::msg_kind_t       rx_msg_kind_q,
    input  logic                        cmd_fifo_full,

    output logic                        rx_bypass_active,
    output logic                        rx_burst_active,

    output logic                        rx_classifier_valid,
    output logic                        rx_classifier_error,
    output logic [9:0]                  rx_row_q,
    output logic [9:0]                  rx_col_q,
    output logic [23:0]                 rx_pixel_q,

    output logic                        rx_rr_cmd_valid,
    output logic [5:0]                  rx_rr_cmd_addr,
    output logic                        rx_rw_cmd_valid,
    output logic [5:0]                  rx_rw_cmd_addr,
    output logic [31:0]                 rx_rw_cmd_data,

    output logic                        rx_pr_cmd_valid,
    output logic [9:0]                  rx_pr_cmd_row,
    output logic [9:0]                  rx_pr_cmd_col,

    output logic                        rx_br_cmd_valid,
    output logic [9:0]                  rx_br_cmd_base_row,
    output logic [9:0]                  rx_br_cmd_base_col,
    output logic [9:0]                  rx_br_cmd_height,
    output logic [9:0]                  rx_br_cmd_width,

    output logic                        cmd_fifo_wr_en,
    output logic [47:0]                 cmd_fifo_wr_data,

    output logic                        pix_wr_seen_sticky,
    output logic                        cmd_ovf_sticky
);

logic        rx_parse_valid;
logic        rx_parse_error;
logic [9:0]  rx_row;
logic [9:0]  rx_col;
logic [23:0] rx_pixel;

logic        rx_pw_parse_valid;
logic        rx_pw_parse_error;
logic [23:0] rx_pw_addr;
logic [23:0] rx_pw_pixel;

logic        rx_cmd_valid;
logic [23:0] rx_cmd_addr;
logic [23:0] rx_cmd_pixel;

logic                                  rx_hdr_frame_ok;
logic                                  rx_hdr_dims_ok;
logic                                  rx_hdr_valid;
logic [rx_burst_pkg::BURST_DIM_W-1:0]  rx_burst_height;
logic [rx_burst_pkg::BURST_DIM_W-1:0]  rx_burst_width;

logic                                  rx_bdata_frame_ok;
logic                                  rx_bdata_error;
logic [rx_burst_pkg::BURST_PIX_PER_MSG-1:0]
      [rx_burst_pkg::BURST_PIX_W-1:0]  rx_burst_pixels;

logic                                  rx_burst_done;
logic                                  rx_burst_cmd_valid;
logic [rx_burst_pkg::BURST_ADDR_W-1:0] rx_burst_cmd_addr;
logic [rx_burst_pkg::BURST_PIX_W-1:0]  rx_burst_cmd_pixel;
logic                                  rx_burst_err_hdr;
logic                                  rx_burst_err_data;
logic                                  rx_burst_err_unexp;

logic        rx_rr_frame_ok;
logic        rx_rr_addr_ok;
logic        rx_rr_valid;
logic        rx_rr_addr_err;
logic [23:0] rx_rr_addr_raw;
logic [5:0]  rx_rr_rgf_addr;

logic        rx_rw_frame_ok;
logic        rx_rw_addr_ok;
logic        rx_rw_valid;
logic        rx_rw_addr_err;
logic [23:0] rx_rw_addr_raw;
logic [5:0]  rx_rw_rgf_addr;
logic [31:0] rx_rw_data;

logic        rx_pr_frame_ok;
logic        rx_pr_coord_ok;
logic        rx_pr_valid;
logic        rx_pr_coord_err;
logic [23:0] rx_pr_row_raw;
logic [23:0] rx_pr_col_raw;
logic [9:0]  rx_pr_row;
logic [9:0]  rx_pr_col;

logic        rx_br_frame_ok;
logic        rx_br_dims_ok;
logic        rx_br_addr_ok;
logic        rx_br_extent_ok;
logic        rx_br_valid;
logic        rx_br_err;
logic [23:0] rx_br_addr_raw;
logic [23:0] rx_br_h_raw;
logic [23:0] rx_br_w_raw;
logic [9:0]  rx_br_base_row;
logic [9:0]  rx_br_base_col;
logic [9:0]  rx_br_height;
logic [9:0]  rx_br_width;

rx_parser u_rx_parser (
    .msg_in      (rx_mac_msg_data),
    .parse_valid (rx_parse_valid),
    .parse_error (rx_parse_error),
    .row         (rx_row),
    .col         (rx_col),
    .pixel       (rx_pixel)
);

rx_pixel_wr_parser u_rx_pixel_wr_parser (
    .msg_in         (rx_mac_msg_data),
    .pw_parse_valid (rx_pw_parse_valid),
    .pw_parse_error (rx_pw_parse_error),
    .pw_addr        (rx_pw_addr),
    .pw_pixel       (rx_pw_pixel)
);

rx_burst_hdr_parser u_rx_burst_hdr_parser (
    .msg_in       (rx_mac_msg_data),
    .hdr_frame_ok (rx_hdr_frame_ok),
    .hdr_dims_ok  (rx_hdr_dims_ok),
    .hdr_valid    (rx_hdr_valid),
    .height       (rx_burst_height),
    .width        (rx_burst_width)
);

rx_burst_data_parser u_rx_burst_data_parser (
    .msg_in        (rx_mac_msg_data),
    .data_frame_ok (rx_bdata_frame_ok),
    .data_error    (rx_bdata_error),
    .pixels        (rx_burst_pixels)
);

rx_burst_ctrl u_rx_burst_ctrl (
    .clk              (clk),
    .rst_n            (rst_n),
    .msg_valid        (rx_mac_msg_valid),
    .msg_kind         (rx_msg_kind_q),
    .hdr_valid        (rx_hdr_valid),
    .height           (rx_burst_height),
    .width            (rx_burst_width),
    .data_frame_ok    (rx_bdata_frame_ok),
    .pixels           (rx_burst_pixels),
    .burst_abort      (1'b0),
    .cmd_ready        (!cmd_fifo_full),
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

rx_reg_write_parser u_rx_reg_write_parser (
    .msg_in      (rx_mac_msg_data),
    .rw_frame_ok (rx_rw_frame_ok),
    .rw_addr_ok  (rx_rw_addr_ok),
    .rw_valid    (rx_rw_valid),
    .rw_addr_err (rx_rw_addr_err),
    .rw_addr     (rx_rw_addr_raw),
    .rw_rgf_addr (rx_rw_rgf_addr),
    .rw_data     (rx_rw_data)
);

rx_reg_read_parser u_rx_reg_read_parser (
    .msg_in      (rx_mac_msg_data),
    .rr_frame_ok (rx_rr_frame_ok),
    .rr_addr_ok  (rx_rr_addr_ok),
    .rr_valid    (rx_rr_valid),
    .rr_addr_err (rx_rr_addr_err),
    .rr_addr     (rx_rr_addr_raw),
    .rr_rgf_addr (rx_rr_rgf_addr)
);

rx_pixel_rd_parser u_rx_pixel_rd_parser (
    .msg_in       (rx_mac_msg_data),
    .pr_frame_ok  (rx_pr_frame_ok),
    .pr_coord_ok  (rx_pr_coord_ok),
    .pr_valid     (rx_pr_valid),
    .pr_coord_err (rx_pr_coord_err),
    .pr_row_raw   (rx_pr_row_raw),
    .pr_col_raw   (rx_pr_col_raw),
    .pr_row       (rx_pr_row),
    .pr_col       (rx_pr_col)
);

rx_burst_rd_parser u_rx_burst_rd_parser (
    .msg_in       (rx_mac_msg_data),
    .br_frame_ok  (rx_br_frame_ok),
    .br_dims_ok   (rx_br_dims_ok),
    .br_addr_ok   (rx_br_addr_ok),
    .br_extent_ok (rx_br_extent_ok),
    .br_valid     (rx_br_valid),
    .br_err       (rx_br_err),
    .br_addr_raw  (rx_br_addr_raw),
    .br_h_raw     (rx_br_h_raw),
    .br_w_raw     (rx_br_w_raw),
    .br_base_row  (rx_br_base_row),
    .br_base_col  (rx_br_base_col),
    .br_height    (rx_br_height),
    .br_width     (rx_br_width)
);

rx_classifier u_rx_classifier (
    .clk              (clk),
    .rst_n            (rst_n),
    .msg_valid        (rx_mac_msg_valid),
    .msg_kind         (rx_msg_kind_q),
    .parse_valid      (rx_parse_valid),
    .parse_error      (rx_parse_error),
    .row              (rx_row),
    .col              (rx_col),
    .pixel            (rx_pixel),
    .pw_parse_valid   (rx_pw_parse_valid),
    .pw_parse_error   (rx_pw_parse_error),
    .pw_addr          (rx_pw_addr),
    .pw_pixel         (rx_pw_pixel),
    .classifier_valid (rx_classifier_valid),
    .classifier_error (rx_classifier_error),
    .row_q            (rx_row_q),
    .col_q            (rx_col_q),
    .pixel_q          (rx_pixel_q),
    .cmd_valid        (rx_cmd_valid),
    .cmd_addr         (rx_cmd_addr),
    .cmd_pixel        (rx_cmd_pixel),
    .burst_active     (rx_burst_active),
    .rr_valid         (rx_rr_valid),
    .rr_addr_err      (rx_rr_addr_err),
    .rr_rgf_addr      (rx_rr_rgf_addr),
    .rr_cmd_valid     (rx_rr_cmd_valid),
    .rr_cmd_addr      (rx_rr_cmd_addr),
    .rw_valid         (rx_rw_valid),
    .rw_addr_err      (rx_rw_addr_err),
    .rw_rgf_addr      (rx_rw_rgf_addr),
    .rw_data          (rx_rw_data),
    .rw_cmd_valid     (rx_rw_cmd_valid),
    .rw_cmd_addr      (rx_rw_cmd_addr),
    .rw_cmd_data      (rx_rw_cmd_data),
    .pr_valid         (rx_pr_valid),
    .pr_coord_err     (rx_pr_coord_err),
    .pr_row           (rx_pr_row),
    .pr_col           (rx_pr_col),
    .pr_cmd_valid     (rx_pr_cmd_valid),
    .pr_cmd_row       (rx_pr_cmd_row),
    .pr_cmd_col       (rx_pr_cmd_col),
    .br_valid         (rx_br_valid),
    .br_err           (rx_br_err),
    .br_base_row      (rx_br_base_row),
    .br_base_col      (rx_br_base_col),
    .br_height        (rx_br_height),
    .br_width         (rx_br_width),
    .br_cmd_valid     (rx_br_cmd_valid),
    .br_cmd_base_row  (rx_br_cmd_base_row),
    .br_cmd_base_col  (rx_br_cmd_base_col),
    .br_cmd_height    (rx_br_cmd_height),
    .br_cmd_width     (rx_br_cmd_width)
);

assign cmd_fifo_wr_en   = rx_burst_cmd_valid || rx_cmd_valid;
assign cmd_fifo_wr_data = rx_burst_cmd_valid
                        ? {rx_burst_cmd_addr, rx_burst_cmd_pixel}
                        : {rx_cmd_addr,       rx_cmd_pixel};

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)                  pix_wr_seen_sticky <= 1'b0;
    else if (rx_cmd_valid)       pix_wr_seen_sticky <= 1'b1;
end

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)                                  cmd_ovf_sticky <= 1'b0;
    else if (cmd_fifo_wr_en && cmd_fifo_full)    cmd_ovf_sticky <= 1'b1;
end

`ifndef SYNTHESIS
    a_no_cmd_overlap: assert property (
        @(posedge clk) disable iff (!rst_n)
        !(rx_burst_cmd_valid && rx_cmd_valid)
    ) else $error("command_processing_subsystem: burst and single-pixel commands overlapped");

    a_cls_cmd_not_dropped: assert property (
        @(posedge clk) disable iff (!rst_n)
        rx_cmd_valid |-> !cmd_fifo_full
    ) else $error("command_processing_subsystem: single-pixel command presented into a full FIFO");
`endif

endmodule : command_processing_subsystem
