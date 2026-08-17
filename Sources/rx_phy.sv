// rx_phy.sv
// ---------
// 6-state Moore FSM UART receiver - PHY layer only.
// Strictly structured 3-block FSM style:
//   - Clean separation of State, Next-State, and Output blocks.
//   - Implements a 3-FF CDC synchronizer to prevent simulation/hardware metastabilities.
//   - Aligns oversampling counters to incoming falling edges to eliminate clock jitter.
//
// Oversampling: 16 ticks per bit period. Data bits sampled at tick_q==8 (centre).
// Start/stop validated with 3-sample majority window at ticks 7,8,9.
//
// Parity (Lab 10):
//   Frame is now start + 8 data + parity + stop (11 bit-times), matching
//   tx_phy.sv's even-parity framing exactly.
//   - byte_valid is now GATED on parity_valid -- a byte that fails parity
//     is never forwarded to the MAC. This replaces an earlier version
//     that pushed byte_valid unconditionally "so the MAC never hangs" --
//     that reasoning was based on a misunderstanding: withholding
//     byte_valid for one bad byte does not deadlock the MAC (it just
//     waits for the next byte, harmlessly), but forwarding a bad byte
//     DOES corrupt the message by silently shifting every byte after it
//     out of position. Gating is the correct fix, not a hang risk.
//   - parity_err_pulse fires the same cycle byte_valid is withheld due to
//     a parity failure. BOTH of its intended consumers are now wired in
//     chip_top.sv -- this is no longer pending work:
//
//       1. rx_mac soft reset. Connected straight to rx_mac's par_val_rst
//          input, in this same 256 MHz domain, no synchroniser needed.
//          rx_mac uses it to force cur_state back to MAC_IDLE and clear
//          msg_buf / byte_idx / byte_we, so a frame corrupted by a bad
//          byte is abandoned rather than completed with every subsequent
//          byte shifted out of position.
//
//       2. RGF fault counter. Crosses to the 100 MHz domain through a
//          cdc_pulse_sync and drives rgf's dedicated parity_fault_incr
//          input, which increments PARITY_FAULT_CNT (0x14). Note it is an
//          INCREMENT PULSE on its own port, not a status_* bus write --
//          that bus has "write this whole value" semantics and cannot
//          express "add one".
//
//     The crossing in (2) is why this must stay a one-shot pulse rather
//     than becoming a level: cdc_pulse_sync is a toggle synchroniser and
//     would translate a held level into a single event anyway, while on
//     the rx_mac side a level would hold the MAC in reset for as long as
//     the condition stayed asserted.
//
// Two real bugs fixed from the previous draft, both worth understanding
// since they'd have made parity checking silently non-functional:
//   1. RX_PARITY previously transitioned to RX_STOP after a single raw
//      clock cycle ("transition after 1 clk to check parity"), not a
//      full bit period. This conflated "time to physically receive the
//      parity bit on the wire" (a full bit period, same as every other
//      bit) with "time to compute the XOR comparison" (combinational,
//      genuinely one cycle). RX_PARITY now waits a full tick_q 0->15
//      period like every other state, so tick_q==10 (the parity sample
//      point) is actually reachable within it.
//   2. The parity check compared against rx_byte, which only updates
//      LATER (during RX_STOP) -- meaning it still held the PREVIOUS
//      byte at the time the check ran, not the byte being validated.
//      Fixed to check against shift_reg, which already holds the
//      current byte's bits at that point.
`timescale 1ns/1ps

import rx_phy_pkg::*;
import uart_pkg::*;

module rx_phy (
    input  logic       clk,
    input  logic       rst_n,
    input  logic       rx_in,

    output logic       byte_valid,
    output logic [7:0] rx_byte,
    output logic       parity_err_pulse,
    output logic       rx_busy
);

// -------------------------------------------------------------------------
// CDC Synchronizer & Falling Edge Detector
// -------------------------------------------------------------------------
logic [2:0] rx_sync_r;
logic       rx_sync;
logic       fall_edge;

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        rx_sync_r <= 3'b111;
    end else begin
        rx_sync_r <= {rx_sync_r[1:0], rx_in};
    end
end

assign rx_sync   = rx_sync_r[2];           // Metastable-protected, synchronized rx line
assign fall_edge = ~rx_sync_r[1] & rx_sync_r[2]; // Clean falling-edge detection of start bit

// -------------------------------------------------------------------------
// Baud tick generator (16x oversampling)
// -------------------------------------------------------------------------
localparam int CNT_WIDTH = $clog2(DIV_RX + 1);
localparam logic [CNT_WIDTH-1:0] TICK_MAX  = CNT_WIDTH'(DIV_RX - 1);

logic [CNT_WIDTH-1:0] baud_cnt;
logic                 tick;

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        baud_cnt <= '0;
        tick     <= 1'b0;
    end else begin
        // CRITICAL: Align the baud accumulator dynamically to the falling edge
        // of the start bit. This eliminates asynchronous clock-jitter.
        if (curr_st == RX_IDLE && fall_edge) begin
            baud_cnt <= '0;
            tick     <= 1'b0;
        end else if (baud_cnt == TICK_MAX) begin
            baud_cnt <= '0;
            tick     <= 1'b1;
        end else begin
            baud_cnt <= baud_cnt + 1'b1;
            tick     <= 1'b0;
        end
    end
end

// -------------------------------------------------------------------------
// FSM State Registers
// -------------------------------------------------------------------------
rx_phy_state_t curr_st, next_st;

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) curr_st <= RX_IDLE;
    else        curr_st <= next_st;
end

// -------------------------------------------------------------------------
// Combinational Next-State Decoder (Block 2 of FSM)
// -------------------------------------------------------------------------
always_comb begin : next_state_logic
    next_st = curr_st;

    case (curr_st)
        RX_IDLE: begin
            if (fall_edge) begin
                next_st = RX_START;
            end
        end

        RX_START: begin
            // Wait for the full 16 oversampling ticks of the start bit
            // to maintain perfect phase lock for subsequent data bits.
            if (tick && tick_q == 4'd15) begin
                if (|start_window)
                    next_st = RX_IDLE; // Invalid start bit (noise glitch), abort
                else begin
                    next_st = RX_DATA;
                end
            end
        end

        RX_DATA: begin
            // Transition precisely at the end of the 8th bit (tick 15)
            if (bit_cnt == 4'd8 && tick && tick_q == 4'd15) begin
                next_st = RX_PARITY;
            end
        end

        RX_PARITY: begin
            // FIXED: wait a full bit period (same tick_q 0->15 pattern as
            // every other bit), not a single raw clock cycle -- the
            // parity bit takes a full bit period to physically arrive on
            // the wire, same as any other bit.
            if (tick && tick_q == 4'd15) begin
                next_st = RX_STOP;
            end
        end

        RX_STOP: begin
            // Transition to RX_DONE when stop bit window completes.
            if (tick && tick_q == 4'd10) begin
                next_st = RX_DONE;
            end
        end

        RX_DONE: begin
            next_st = RX_IDLE;
        end

        default: begin
            next_st = RX_IDLE;
        end
    endcase
end : next_state_logic

// -------------------------------------------------------------------------
// tick_q: oversampling tick counter within current bit period (0-15).
// -------------------------------------------------------------------------
logic [3:0] tick_q;

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        tick_q <= 4'd0;
    end else if (curr_st == RX_IDLE && fall_edge) begin
        // Reset timing lock precisely on start edge
        tick_q <= 4'd0;
    end else if (tick) begin
        tick_q <= tick_q + 1'b1;
    end
end

// -------------------------------------------------------------------------
// Majority-vote windows for start/stop validation.
// -------------------------------------------------------------------------
logic [2:0] start_window;
logic [2:0] stop_window;

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        start_window <= 3'b111;
    else if (curr_st == RX_START) begin
        if (tick && (tick_q == 4'd7 || tick_q == 4'd8 || tick_q == 4'd9))
            start_window <= {start_window[1:0], rx_sync};
    end else
        start_window <= 3'b111;
end

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        stop_window <= 3'b000;
    else if (curr_st == RX_STOP) begin
        if (tick && (tick_q == 4'd7 || tick_q == 4'd8 || tick_q == 4'd9))
            stop_window <= {stop_window[1:0], rx_sync};
    end else
        stop_window <= 3'b000;
end

// -------------------------------------------------------------------------
// Bit counter: counts bits sampled in RX_DATA (target: 8).
// -------------------------------------------------------------------------
logic [3:0] bit_cnt;

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        bit_cnt <= 4'd0;
    else if (curr_st == RX_IDLE)
        bit_cnt <= 4'd0;
    else if (curr_st == RX_DATA && tick && tick_q == 4'd8 && bit_cnt < 4'd8)
        bit_cnt <= bit_cnt + 1'b1;
end

// -------------------------------------------------------------------------
// Shift register: LSB-first reception.
// -------------------------------------------------------------------------
logic [7:0] shift_reg;

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        shift_reg <= 8'h00;
    else if (curr_st == RX_DATA && tick && tick_q == 4'd8 && bit_cnt < 4'd8)
        shift_reg <= {rx_sync, shift_reg[7:1]};
end

// -------------------------------------------------------------------------
// Registered Outputs (Moore) (Block 3 of FSM)
// -------------------------------------------------------------------------

// byte_valid: one-cycle pulse in RX_DONE, GATED on parity_valid -- a byte
// that failed parity is never forwarded to the MAC. By the time RX_DONE
// is reached, parity_valid already holds the freshly-computed result for
// THIS byte (it was set earlier, at RX_PARITY's tick_q==10, well before
// the RX_PARITY->RX_STOP transition at tick_q==15), so this is safe.
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) byte_valid <= 1'b0;
    else        byte_valid <= (curr_st == RX_DONE) && parity_valid;
end

// parity_err_pulse: fires exactly when byte_valid is withheld due to a
// parity failure -- mutually exclusive with byte_valid by construction
// (same curr_st==RX_DONE condition, opposite parity_valid polarity).

// Logic signal for parity check. Taken out of ports b/c everything is
// being driven off the pulse, so no need for parity_valid to be out facing
logic       parity_valid;

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) parity_err_pulse <= 1'b0;
    else        parity_err_pulse <= (curr_st == RX_DONE) && !parity_valid;
end

// rx_byte: latch shift_reg the moment the stop bit is confirmed.
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        rx_byte <= 8'h00;
    else if (curr_st == RX_STOP && tick && tick_q == 4'd10)
        rx_byte <= shift_reg;
end

// parity_valid: FIXED to check shift_reg (the byte currently being
// validated) instead of rx_byte (which still held the PREVIOUS byte at
// this point, since rx_byte doesn't update until RX_STOP, later).
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        parity_valid <= 1'b0;
    else if (curr_st == RX_PARITY && tick && tick_q == 4'd10)
        parity_valid <= (^shift_reg) == rx_sync;
end

// rx_busy: high in any non-IDLE state
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) rx_busy <= 1'b0;
    else        rx_busy <= (next_st != RX_IDLE);
end

endmodule : rx_phy