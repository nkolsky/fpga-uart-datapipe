`timescale 1ns / 1ps

// tx_mac
// ------
// Accepts a 128-bit message (16 x 8-bit bytes) from tx_sequencer
// and forwards each byte sequentially to the TX PHY.
//
// Per-byte cycle (matches mac_fsm spec):
//   MAC_TRIG_BYTE → registers phy_valid=1 and phy_data=byte_buf[byte_idx]
//   MAC_WAIT_BUSY → waits for phy_busy HIGH (PHY acknowledged)
//   MAC_WAIT_BYTE → waits for phy_busy LOW  (byte fully serialised)
//   MAC_CHK_DONE  → increments byte_idx, loops or exits
//
// All outputs are registered (Moore FSM).
// phy_busy is derived as !phy_ready.

module tx_mac (
    input  logic         clk,
    input  logic         rst_n,
    input  logic         rx_mode,

    // Upstream interface (tx_sequencer)
    input  logic         msg_valid,
    input  logic [127:0] msg_data,    // flat [127:0]: msg_data[7:0]=byte0, [15:8]=byte1 ...
    output logic         mac_busy,

    // TX PHY interface
    input  logic         phy_ready,   // HIGH when PHY idle
    output logic [7:0]   phy_data,    // registered byte to PHY
    output logic         phy_valid    // registered one-cycle start_tx pulse to PHY
);

import tx_mac_pkg::*;

    // ----------------------------------------------------------------
    // Internal registers
    // ----------------------------------------------------------------
    mac_state_t cur_state, next_state;

    logic [127:0] msg_buf;    // latched flat message
    logic [3:0]   byte_idx;

    localparam int NUM_BYTES = 16;

    logic phy_busy;
    assign phy_busy = !phy_ready;

    // ----------------------------------------------------------------
    // State register
    // ----------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n || rx_mode)
            cur_state <= MAC_IDLE;
        else
            cur_state <= next_state;
    end

    // ----------------------------------------------------------------
    // Next-state logic
    // ----------------------------------------------------------------
    always_comb begin
        next_state = cur_state;
        case (cur_state)
            MAC_IDLE:
                if (msg_valid) next_state = MAC_LOAD;

            MAC_LOAD:
                next_state = MAC_TRIG_BYTE;

            MAC_TRIG_BYTE:
                next_state = MAC_WAIT_BUSY;

            MAC_WAIT_BUSY:
                if (phy_busy)  next_state = MAC_WAIT_BYTE;

            MAC_WAIT_BYTE:
                if (!phy_busy) next_state = MAC_CHK_DONE;

            MAC_CHK_DONE:
                if (byte_idx == 4'd15) next_state = MAC_DONE;
                else                   next_state = MAC_TRIG_BYTE;

            MAC_DONE:
                next_state = MAC_IDLE;

            default:
                next_state = MAC_IDLE;
        endcase
    end

    // ----------------------------------------------------------------
    // Datapath: latch message and manage byte_idx
    // ----------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n || rx_mode) begin
            msg_buf  <= 128'd0;
            byte_idx <= 4'd0;
        end else begin
            if (cur_state == MAC_LOAD) begin
                msg_buf  <= msg_data;
                byte_idx <= 4'd0;
            end
            if (cur_state == MAC_CHK_DONE)
                byte_idx <= byte_idx + 4'd1;
        end
    end

    // ----------------------------------------------------------------
    // Registered outputs (Moore)
    // ----------------------------------------------------------------

    // mac_busy: high in any non-idle state, using registered next_state
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n || rx_mode) mac_busy <= 1'b0;
        else                   mac_busy <= (next_state != MAC_IDLE);
    end

    // phy_valid (start_tx): registered, high when leaving MAC_TRIG_BYTE
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n || rx_mode) phy_valid <= 1'b0;
        else                   phy_valid <= (cur_state == MAC_TRIG_BYTE);
    end

    // phy_data (tx_byte): registered, captures current byte in MAC_TRIG_BYTE
    // msg_buf[7:0]=byte0, [15:8]=byte1, ..., [127:120]=byte15
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n || rx_mode) phy_data <= 8'h00;
        else if (cur_state == MAC_TRIG_BYTE)
            phy_data <= msg_buf[byte_idx * 8 +: 8];
    end

endmodule
