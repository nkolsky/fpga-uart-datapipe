// rx_mac.sv
// ---------
// 4-state Moore FSM UART receiver - MAC layer.
// Updated to ensure stable msg_data output.
//
// -----------------------------------------------------------------------
// STAGE 2A: VARIABLE-LENGTH FRAMER
// -----------------------------------------------------------------------
// This module is now a variable-length framer. The name is retained for
// this milestone to keep the diff auditable; "MAC" here means byte
// collection and frame-boundary detection only.
//
// It holds NO protocol knowledge. It counts bytes up to expected_len,
// which rx_msg_decode derives combinationally from the live buffer. The
// only structural change from the fixed-16 version is the terminal
// comparison in MAC_CHK_DONE.
//
// Why count-based and not delimiter-based: payload bytes in the Final
// Project message set are raw binary and can legitimately equal '{'
// (0x7B) or '}' (0x7D), so scanning for a closing brace would truncate
// valid frames.
//
// BYTE-COUNT TIMING CONTRACT (see rx_msg_decode.sv for the full trace):
//   byte_idx increments in MAC_STORE alongside the buffer write, both
//   non-blocking on the same edge. So on entry to MAC_CHK_DONE the
//   newest byte is already committed to msg_buf and byte_idx already
//   counts it. byte_cnt therefore means BYTES ALREADY STORED, not the
//   next storage index, and the decode sees the decisive byte on the
//   exact cycle the decision is made -- never one cycle late.
//
// PROVISIONAL VS FINAL KIND:
//   msg_kind_prov changes as bytes arrive and is meaningful only once
//   the frame is complete. It is latched here into msg_kind_q during
//   MAC_DONE, in the same cycle and the same manner as msg_data.
//   Downstream logic must act only on msg_valid / msg_data / msg_kind_q.

`timescale 1ns/1ps

import rx_mac_pkg::*;

module rx_mac
    import rx_msg_pkg::*;
(
    input  logic         clk,
    input  logic         rst_n,

    // RX PHY interface
    input  logic         byte_valid,
    input  logic [7:0]   rx_byte,
    
    //Parity check from RX PHY for soft reset
    input  logic         par_val_rst, //pulses High when parity error is detected

    // -----------------------------------------------------------------
    // Framing control from rx_msg_decode (combinational)
    // -----------------------------------------------------------------
    input  logic [BYTE_CNT_W-1:0] expected_len,   // total bytes in this frame
    input  msg_kind_t             msg_kind_prov,  // provisional; latched below

    // -----------------------------------------------------------------
    // Live frame state out to rx_msg_decode. This is the LIVE tap: the
    // buffer as it fills, not the stable copy. Distinct from msg_data.
    // -----------------------------------------------------------------
    output logic [127:0]          frame_buf,
    output logic [BYTE_CNT_W-1:0] byte_cnt,

    // -----------------------------------------------------------------
    // Upstream interface (registered tap -- stable while msg_valid high)
    // -----------------------------------------------------------------
    output logic         msg_valid,
    output logic [127:0] msg_data,
    output msg_kind_t    msg_kind_q,   // final classification of msg_data
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

        // STAGE 2A: the only structural change. Previously compared
        // against the constant 5'd16. byte_idx holds the number of bytes
        // already stored, and the newest byte was committed on the edge
        // entering this state, so expected_len is evaluated against a
        // buffer that already contains the deciding byte.
        MAC_CHK_DONE:
            if (byte_idx == expected_len) next_state = MAC_DONE;
            else                          next_state = MAC_IDLE;

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

// -----------------------------------------------------------------------------
// TIMING: REGISTERED ONE-HOT BYTE WRITE ENABLE
// -----------------------------------------------------------------------------
// The byte store used to be a variable DESCENDING part-select whose base was
// computed arithmetically:
//
//     msg_buf[127 - (byte_idx[3:0] * 8) -: 8] <= rx_byte_latch;
//
// Functionally correct, but it placed a chain of arithmetic between byte_idx's
// flop output and the clock enable of every msg_buf bit. In one cycle Vivado
// had to build, in this order:
//
//   1. byte_idx[3:0] * 8        a 7-bit shift (free -- this part was fine)
//   2. 127 - (...)              an 8-bit SUBTRACTOR, with its carry chain
//   3. a variable part-select off that 8-bit base, including the out-of-range
//      handling the tool must assume is reachable, because nothing in the RTL
//      told it the base only ever takes 16 legal values
//   4. AND with (cur_state == MAC_STORE)
//   5. OR with the MAC_DONE clear, which writes msg_buf from the same block
//
// That is the reported 11-level cone ending on msg_buf_reg[*]/CE:
//   byte_idx_reg[1]/C -> msg_buf_reg[*]/CE, 3.518 ns logic + 3.960 ns routing
//   = 7.478 ns against a 7.692 ns period, WNS -0.153 ns.
//
// Note the failure is on the ENABLE, not the data. rx_byte_latch reaches the D
// inputs through almost nothing; it is the decode of WHICH lane to enable that
// is deep, so that is what moves.
//
// THE FIX: compute the enable ONE CYCLE EARLY and register it as a 16-bit
// one-hot. msg_buf's clock enable then comes straight off a flop --
//
//     CE[lane] = byte_we[lane] || (cur_state == MAC_DONE)
//
// -- one LUT level, while the lane decode moves onto byte_we's D input where a
// whole cycle is available for it.
//
// NOTHING OBSERVABLE CHANGES. The write still lands in the MAC_STORE cycle, in
// the same lane, with the same data:
//
//   store_next is exactly the condition under which next_state becomes
//   MAC_STORE -- (MAC_IDLE or MAC_DONE) and byte_valid -- so byte_we is high in
//   precisely the cycles where cur_state == MAC_STORE. It is spelled out rather
//   than written as (next_state == MAC_STORE) so that the byte_idx ==
//   expected_len comparison from the MAC_CHK_DONE branch is not pulled into
//   this path.
//
//   store_lane is byte_idx[3:0], except when the decision is taken from
//   MAC_DONE, where byte_idx is being cleared to 0 on that same edge and the
//   upcoming store therefore belongs to lane 0.
//
//   store_next is the SAME term that enables rx_byte_latch above, so the enable
//   and the data it steers cannot drift apart.
//
// Byte ordering is unchanged -- lane 0 is msg_buf[127:120], as every
// receive-side parser expects. Frame length decoding, parity handling,
// byte_cnt, msg_valid, msg_data, msg_kind_q and mac_busy are all untouched.
// -----------------------------------------------------------------------------
logic [15:0] byte_we;
logic [3:0]  store_lane;
logic        store_next;

assign store_next = ((cur_state == MAC_IDLE) || (cur_state == MAC_DONE)) &&
                    byte_valid;

assign store_lane = (cur_state == MAC_DONE) ? 4'd0 : byte_idx[3:0];

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n || par_val_rst) byte_we <= 16'd0;
    else if (store_next)       byte_we <= 16'd1 << store_lane;
    else                       byte_we <= 16'd0;
end

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n || par_val_rst) begin
        msg_buf  <= 128'd0;
        byte_idx <= 5'd0;
    end else begin
        // Fully unrolled: every bit index below is a compile-time constant,
        // so no arithmetic and no variable part-select survives in the cone
        // that reaches msg_buf's clock enable.
        for (int i = 0; i < 16; i++) begin
            if (byte_we[i]) msg_buf[127 - i*8 -: 8] <= rx_byte_latch;
        end

        if (cur_state == MAC_STORE) begin
            byte_idx <= byte_idx + 1'b1;
        end
        if (cur_state == MAC_DONE) begin
            byte_idx <= 5'd0;
            // STAGE 2A: clear the buffer between frames as defence in
            // depth against stale-byte misdecodes. rx_msg_decode already
            // gates every narrowing rule on byte_cnt, so this is a second
            // line of defence, not the primary one -- but it makes bytes
            // above the count read as 0x00, which matches none of '}',
            // 'P', 'V', 'C' or 'H'.
            //
            // Safe alongside "msg_data <= msg_buf" below: both are
            // non-blocking and both sample the SAME pre-edge value, so
            // the outgoing message is the completed frame, not zeros.
            msg_buf <= 128'd0;
        end
    end
end

// -------------------------------------------------------------------------
// Live tap out to rx_msg_decode.
//
// Deliberately NOT the same as msg_data: the decode needs the buffer as
// it fills, one cycle-accurate view, whereas msg_data is the stable copy
// published to the parsers when the frame is complete.
// -------------------------------------------------------------------------
`ifndef SYNTHESIS
    // The registered enable must coincide EXACTLY with the store cycle. If
    // this fails, the restructuring has changed behaviour -- the one thing it
    // must not do.
    a_we_matches_store: assert property (
        @(posedge clk) disable iff (!rst_n || par_val_rst)
        (|byte_we) == (cur_state == MAC_STORE)
    ) else $error("%m: byte_we and MAC_STORE disagree");

    // Exactly one lane, never two.
    a_we_onehot: assert property (
        @(posedge clk) disable iff (!rst_n || par_val_rst)
        $onehot0(byte_we)
    ) else $error("%m: byte_we is not one-hot");

    // ...and it must be the lane byte_idx names during that store, which is
    // the property that guarantees byte ordering is preserved.
    a_we_lane_correct: assert property (
        @(posedge clk) disable iff (!rst_n || par_val_rst)
        (cur_state == MAC_STORE) |-> byte_we[byte_idx[3:0]]
    ) else $error("%m: byte_we selected the wrong lane");
`endif

assign frame_buf = msg_buf;
assign byte_cnt  = byte_idx;

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

// STAGE 2A: latch the FINAL classification in the same cycle and the
// same manner as msg_data. During MAC_DONE the buffer is complete and
// byte_idx still holds the final count, so rx_msg_decode's output is
// correct at this instant. Capturing it here is what turns a provisional
// value into a stable one that travels with msg_data.
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        msg_kind_q <= MSG_UNKNOWN;
    else if (cur_state == MAC_DONE)
        msg_kind_q <= msg_kind_prov;
end

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) mac_busy <= 1'b0;
    else        mac_busy <= (next_state != MAC_IDLE);
end

endmodule : rx_mac