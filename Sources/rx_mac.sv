// rx_mac.sv
// ---------
// 4-state Moore FSM UART receiver - MAC layer.
// Updated to ensure stable msg_data output.

`timescale 1ns/1ps

import rx_mac_pkg::*;

module rx_mac (
    input  logic         clk,
    input  logic         rst_n,

    // RX PHY interface
    input  logic         byte_valid,
    input  logic [7:0]   rx_byte,
    
    //Parity check from RX PHY for soft reset
    input  logic         par_val_rst, //pulses High when parity error is detected

    // Upstream interface
    output logic         msg_valid,
    output logic [127:0] msg_data,
    output logic         mac_busy
);

// -------------------------------------------------------------------------
// Internal registers
// -------------------------------------------------------------------------
rx_mac_state_t cur_state, next_state;
logic [127:0] msg_buf;
logic [4:0]   byte_idx;
logic [7:0]   rx_byte_latch;

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n || par_val_rst) cur_state <= MAC_IDLE;
    else        cur_state <= next_state;
end

// -------------------------------------------------------------------------
// Next-state logic
// -------------------------------------------------------------------------
always_comb begin : next_state_logic
    next_state = cur_state;

    case (cur_state)
        MAC_IDLE:
            if (byte_valid) next_state = MAC_STORE;

        MAC_STORE:
            next_state = MAC_CHK_DONE;

        MAC_CHK_DONE:
            if (byte_idx == 5'd16) next_state = MAC_DONE;
            else                   next_state = MAC_IDLE;

        MAC_DONE:
            if (byte_valid) next_state = MAC_STORE;
            else            next_state = MAC_IDLE;

        default:
            next_state = MAC_IDLE;
    endcase
end : next_state_logic

// -------------------------------------------------------------------------
// Datapath
// -------------------------------------------------------------------------
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        rx_byte_latch <= 8'h00;
    else if ((cur_state == MAC_IDLE || cur_state == MAC_DONE) && byte_valid)
        rx_byte_latch <= rx_byte;
end

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n || par_val_rst) begin
        msg_buf  <= 128'd0;
        byte_idx <= 5'd0;
    end else begin
        if (cur_state == MAC_STORE) begin
            msg_buf[127 - (byte_idx[3:0] * 8) -: 8] <= rx_byte_latch;
            byte_idx <= byte_idx + 1'b1;
        end
        if (cur_state == MAC_DONE)
            byte_idx <= 5'd0;
    end
end

// -------------------------------------------------------------------------
// Registered outputs
// -------------------------------------------------------------------------
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) msg_valid <= 1'b0;
    else        msg_valid <= (cur_state == MAC_DONE);
end

// FINAL TWEAK: Directly output the buffer content during MAC_DONE 
// to guarantee stable data when msg_valid is high.
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) 
        msg_data <= 128'd0;
    else if (cur_state == MAC_DONE)
        msg_data <= msg_buf;
end

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) mac_busy <= 1'b0;
    else        mac_busy <= (next_state != MAC_IDLE);
end

endmodule : rx_mac