// -----------------------------------------------------------------------------
// register_subsystem.sv
//
// 100 MHz register-file bridge. It decodes the synchronized command stream,
// drives the RGF, and captures the read data for the return path.
//
// IMG_TX_MON writes are generated from the synchronized image-complete pulse,
// while read responses are sampled in the same cycle that the RGF sees a read
// address.
// -----------------------------------------------------------------------------
`timescale 1ns/1ps

module register_subsystem (
    input  logic        clk,
    input  logic        rst_n,

    // APB slave port. The master and the decoder live at chip_top with the
    // rest of the interconnect; only the slave sits here, because it
    // translates to rgf's private pc_* port.
    input  logic                       psel,
    input  logic                       penable,
    input  logic                       pwrite,
    input  logic [apb_pkg::ADDR_W-1:0] paddr,
    input  logic [apb_pkg::DATA_W-1:0] pwdata,
    output logic                       pready,
    output logic [apb_pkg::DATA_W-1:0] prdata,
    output logic                       pslverr,

    // Already synchronized 100 MHz status/event inputs.
    input  logic        tx_img_done,
    input  logic [9:0]  tx_row,
    input  logic [9:0]  tx_col,
    input  logic        parity_fault_incr,
    input  logic        fifo_full,
    input  logic        fifo_empty,
    input  logic        fifo_almost_full,
    input  logic        fifo_almost_empty,

    // rd_strobe / rd_value are GONE. The read reply originates at apb_master
    // now -- the only module that knows a transfer completed. This one only
    // sees strobes. chip_top drives the return cdc_cmd_sync from the master.

    // Register-controlled outputs used elsewhere in chip_top.
    output logic        start_img_read,
    output logic        clk_sel
);

import rgf_pkg::*;

logic [7:0]  rgf_pc_addr;
logic        rgf_pc_wen;
logic        rgf_pc_ren;
logic [31:0] rgf_pc_wdata;
logic [31:0] rgf_pc_rdata;

logic        rgf_status_wen;
logic [7:0]  rgf_status_addr;
logic [31:0] rgf_status_wdata;

// APB slave front end. Owns the translation from the bus to rgf's mem-style
// port, including the pc_ren strobe that retired the IDLE_ADDR parking.
apb_slave_rgf #(
    .WAIT_STATES (0)          // rgf's read mux is combinational
) u_apb_slave (
    .clk      (clk),
    .rst_n    (rst_n),
    .psel     (psel),
    .penable  (penable),
    .pwrite   (pwrite),
    .paddr    (paddr),
    .pwdata   (pwdata),
    .pready   (pready),
    .prdata   (prdata),
    .pslverr  (pslverr),
    .pc_addr  (rgf_pc_addr),
    .pc_wen   (rgf_pc_wen),
    .pc_ren   (rgf_pc_ren),
    .pc_wdata (rgf_pc_wdata),
    .pc_rdata (rgf_pc_rdata)
);

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
    .pc_ren             (rgf_pc_ren),
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
