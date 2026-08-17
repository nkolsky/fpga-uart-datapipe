// -----------------------------------------------------------------------------
// register_subsystem.sv
//
// 100 MHz register-file subsystem.
//
// Contains:
//   - destination-side RGF command decode
//   - rgf instance
//   - IMG_TX_MON status-write decode
//   - register-read capture
//
// CDC instances remain in chip_top.
// -----------------------------------------------------------------------------
`timescale 1ns/1ps

module register_subsystem (
    input  logic        clk,
    input  logic        rst_n,

    // Already synchronized command from chip_top's forward cdc_cmd_sync.
    input  logic        cmd_valid,
    input  logic        cmd_is_write,
    input  logic [7:0]  cmd_addr,
    input  logic [31:0] cmd_wdata,

    // Already synchronized 100 MHz status/event inputs.
    input  logic        tx_img_done,
    input  logic [9:0]  tx_row,
    input  logic [9:0]  tx_col,
    input  logic        parity_fault_incr,
    input  logic        fifo_full,
    input  logic        fifo_empty,
    input  logic        fifo_almost_full,
    input  logic        fifo_almost_empty,

    // Source-side payload for chip_top's return cdc_cmd_sync.
    output logic        rd_strobe,
    output logic [31:0] rd_value,

    // Register-controlled outputs used elsewhere in chip_top.
    output logic        start_img_read,
    output logic        clk_sel
);

import rgf_pkg::*;

logic [7:0]  rgf_pc_addr;
logic        rgf_pc_wen;
logic [31:0] rgf_pc_wdata;
logic [31:0] rgf_pc_rdata;

logic        rgf_status_wen;
logic [7:0]  rgf_status_addr;
logic [31:0] rgf_status_wdata;

assign rgf_pc_wen   = cmd_valid && cmd_is_write;
assign rgf_pc_addr  = cmd_addr;
assign rgf_pc_wdata = cmd_wdata;

// Capture a read result in the same cycle that the RGF sees the read address.
//
// cmd_addr is parked at rgf_pkg::IDLE_ADDR outside cmd_valid. That parking
// is MEM_MSG_ROUTER's job now -- it used to be done by the forward
// cdc_cmd_sync, which drove dst_addr to its IDLE_ADDR parameter whenever
// dst_valid was low, but that crossing was deleted when register commands
// became messages. It matters because rgf's IMG_TX_MON read-to-clear is a
// level-sensitive decode with no valid qualifier, so a held address clears
// img_send_complete/img_send_error on every idle cycle.
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        rd_strobe <= 1'b0;
        rd_value  <= 32'd0;
    end
    else begin
        rd_strobe <= cmd_valid && !cmd_is_write;
        if (cmd_valid && !cmd_is_write)
            rd_value <= rgf_pc_rdata;
    end
end

// Write IMG_TX_MON once when the synchronized image-complete pulse arrives.
assign rgf_status_wen   = tx_img_done;
assign rgf_status_addr  = IMG_TX_MON_ADDR;
assign rgf_status_wdata = {
    10'b0,
    1'b0,       // error
    1'b1,       // complete
    tx_col,
    tx_row
};

rgf u_rgf (
    .clk                (clk),
    .rst_n              (rst_n),
    .pc_wen             (rgf_pc_wen),
    .pc_addr            (rgf_pc_addr),
    .pc_wdata           (rgf_pc_wdata),
    .pc_rdata           (rgf_pc_rdata),
    .status_wen         (rgf_status_wen),
    .status_addr        (rgf_status_addr),
    .status_wdata       (rgf_status_wdata),
    .img_height_in      (10'(memory_pkg::IMG_HEIGHT)),
    .img_width_in       (10'(memory_pkg::IMG_WIDTH)),
    .img_ready_in       (1'b1),
    .start_img_read_out (start_img_read),
    .clk_sel_out        (clk_sel),
    .parity_fault_incr  (parity_fault_incr),
    .fifo_full          (fifo_full),
    .fifo_empty         (fifo_empty),
    .fifo_almost_full   (fifo_almost_full),
    .fifo_almost_empty  (fifo_almost_empty)
);

endmodule : register_subsystem
