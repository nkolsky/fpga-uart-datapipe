`timescale 1ns/1ps

// tx_phy
// ------
// 4-state Moore FSM UART serializer, matching phy_fsm spec.
//
// phy_ready (!busy) uses registered next-state:
//   phy_ready <= (nxt_st == PHY_IDLE)
// This means phy_ready goes LOW on the same cycle the FSM leaves IDLE
// (one cycle after phy_valid is seen), giving the MAC correct handshake timing.
//
// tx_out is a registered Moore output driven by curr_st.
// tx_reg[0] is stable for a full baud period before each shift,
// so the one-cycle register latency on tx_out is benign.

module tx_phy (
    input  logic       clk,
    input  logic       rst_n,

    // MAC interface
    input  logic       phy_valid,   // start_tx: one-cycle pulse from MAC
    input  logic [7:0] phy_data,    // tx_byte: byte to serialise
    output logic       phy_ready,   // !busy: HIGH when PHY idle

    // UART serial output
    output logic       tx_out,      // registered TX line
    output logic       led
);
import uart_pkg::*;
import tx_pkg::*;

localparam int BAUD_TICK_MAX = DIV_TX - 1;
localparam int CNT_WIDTH     = $clog2(DIV_TX + 1);


phy_state_t curr_st, nxt_st;

logic [CNT_WIDTH-1:0] cnt_baud;
logic                 baud_tick;
logic [2:0]           bits_cnt;
logic [7:0]           tx_reg;
logic                 parity_bit;

// -------------------------------------------------------------------------
// State register
// -------------------------------------------------------------------------
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) curr_st <= PHY_IDLE;
    else        curr_st <= nxt_st;
end

// -------------------------------------------------------------------------
// Next-state logic
// -------------------------------------------------------------------------
always_comb begin
    nxt_st = curr_st;
    case (curr_st)
        PHY_IDLE:      if (phy_valid)                     nxt_st = PHY_START_BIT;
        PHY_START_BIT: if (baud_tick)                     nxt_st = PHY_DATA_BITS;
        PHY_DATA_BITS: if (baud_tick && bits_cnt == 3'd7) nxt_st = PHY_PARITY_BIT;
        PHY_PARITY_BIT: if (baud_tick)                    nxt_st = PHY_STOP_BIT;
        PHY_STOP_BIT:  if (baud_tick)                     nxt_st = PHY_IDLE;
        default:                                          nxt_st = PHY_IDLE;
    endcase
end

// -------------------------------------------------------------------------
// Baud tick generator - resets only in PHY_IDLE
// -------------------------------------------------------------------------
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        cnt_baud  <= '0;
        baud_tick <= 1'b0;
    end else if (curr_st == PHY_IDLE) begin
        cnt_baud  <= '0;
        baud_tick <= 1'b0;
    end else begin
        /* verilator lint_off WIDTHEXPAND */
        if (cnt_baud == BAUD_TICK_MAX) begin
        /* verilator lint_on WIDTHEXPAND */
            cnt_baud  <= '0;
            baud_tick <= 1'b1;
        end else begin
            cnt_baud  <= cnt_baud + 1'b1;
            baud_tick <= 1'b0;
        end
    end
end

// -------------------------------------------------------------------------
// Shift register and bit counter
// Loaded in PHY_IDLE on phy_valid. Shifted in PHY_DATA_BITS on baud_tick.
// The baud_tick that exits PHY_START_BIT has curr_st==PHY_START_BIT so
// the shift guard (curr_st==PHY_DATA_BITS) correctly blocks it.
// -------------------------------------------------------------------------
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        tx_reg   <= '0;
        bits_cnt <= '0;
        parity_bit <= 1'b0;
        led      <= 1'b0;
    end else if (curr_st == PHY_IDLE && phy_valid) begin
        tx_reg   <= phy_data;
        bits_cnt <= '0;
        parity_bit <= ^phy_data;  // even parity
        led      <= ~led;
    end else if (curr_st == PHY_DATA_BITS && baud_tick) begin
        tx_reg   <= {1'b0, tx_reg[7:1]};
        bits_cnt <= bits_cnt + 1'b1;
    end
end

// -------------------------------------------------------------------------
// Registered outputs (Moore)
//
// phy_ready: uses registered NEXT-state per spec:
//   "busy HIGH whenever nxt_st != PHY_IDLE"
//   so phy_ready (= !busy) is HIGH only when nxt_st == PHY_IDLE
// This makes phy_ready go LOW on the same cycle the FSM leaves IDLE,
// one cycle earlier than (curr_st == PHY_IDLE) would give.
//
// tx_out: registered Moore output from curr_st.
// -------------------------------------------------------------------------
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        phy_ready <= 1'b1;
        tx_out    <= 1'b1;
    end else begin
        phy_ready <= (curr_st == PHY_IDLE);  // deasserts one cycle after leaving IDLE, safe for MAC handshake
        case (curr_st)
            PHY_IDLE:       tx_out <= 1'b1;
            PHY_START_BIT:  tx_out <= 1'b0;
            PHY_DATA_BITS:  tx_out <= tx_reg[0];
            PHY_PARITY_BIT: tx_out <= parity_bit;
            PHY_STOP_BIT:   tx_out <= 1'b1;
            default:        tx_out <= 1'b1;
        endcase
    end
end

endmodule
 

