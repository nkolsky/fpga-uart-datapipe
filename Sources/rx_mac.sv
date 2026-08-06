// rx_mac.sv
// ---------
// Collects bytes from the RX PHY into a frame buffer, finds the frame
// boundaries itself, and announces the result.
//
// =======================================================================
// THE FRAMING CONTRACT
// =======================================================================
// Every message has the same skeleton:
//
//   byte  0        '{'
//   byte  1        opcode         <- parser
//   bytes 2..4     payload        <- parser
//   byte  5        ',' or '}'
//   byte  6        opcode         <- parser
//   bytes 7..9     payload        <- parser
//   byte 10        ',' or '}'
//   byte 11        opcode         <- parser
//   bytes 12..14   payload        <- parser
//   byte 15        '}'
//
// THIS MODULE OWNS BYTES 0, 5, 10 AND 15. Nothing else. It never inspects an
// opcode and never extracts a field. rx_msg_parser owns 1, 6 and 11 and
// extracts 2-4, 7-9 and 12-14. The two never look at the same byte.
//
// A '}' at byte 5 ends the frame at 6 bytes; at byte 10, at 11 bytes.
//
// =======================================================================
// WHY THIS IS SAFE, AND WHY A DELIMITER SEARCH WOULD NOT BE
// =======================================================================
// Payload is RAW BINARY. A pixel value of 123 is 0x7B, which is '{'. A
// dimension byte of 125 is 0x7D, which is '}'. Hunting for a delimiter
// anywhere in the frame would truncate legitimate messages constantly.
//
// The stride is what makes it safe: delimiters occur ONLY at byte indices
// 0, 5, 10 and 15, and payload occurs only at indices that are not those. The
// check here is POSITIONAL -- it looks at byte 5 and asks what it is, never
// asks where the next '}' is. A 0x7D at byte 7 is a payload byte and is never
// examined.
//
// =======================================================================
// WHAT THIS REPLACED
// =======================================================================
// The MAC used to receive expected_len from the parser and count up to it.
// That made the parser's combinational decode part of this FSM's next-state
// logic, and it created a zero-margin case: a 6-byte register read had its
// length resolved by byte 5 -- the very byte that completed it. Correct, but
// only because the parser saw the newest byte in the same cycle the MAC
// compared against it.
//
// Deriving the boundary locally removes both. There is no expected_len, no
// feedback from parser to MAC, and dataflow is strictly one-directional:
//
//   rx_mac --frame_buf, byte_cnt, frame_done, frame_err--> rx_msg_parser
//
// It also removed bypass_active from this module entirely. A burst data frame
// is {<4 bytes>,<4 bytes>,<4 bytes>} -- ',' at 5 and 10, '}' at 15 -- so the
// positional rule frames it with no special case. bypass_active still goes to
// the parser, which must know those frames carry no opcodes, but that is
// classification, not framing.
//
// =======================================================================
// THE START GATE
// =======================================================================
// A frame may only begin on '{'. In IDLE with an empty buffer, any other byte
// is discarded where it stands: not stored, not counted, no state change.
// Mid-frame bytes are accepted unconditionally, because payload is raw binary
// and 0x7B is a legal value there.
//
// Combined with MAC_ERR this also handles truncation. A frame abandoned
// part-way used to leave byte_idx non-zero with no way back, so the host's
// next '{' was swallowed as payload and the two frames spliced. Now a
// malformed delimiter aborts to MAC_ERR, byte_idx clears, and the start gate
// discards bytes until a real '{' arrives.
//
// =======================================================================
// ONE BUFFER, NO COPY
// =======================================================================
// There is no separate published register. frame_buf is both the live tap the
// parser sees as the frame fills and the completed frame consumers read while
// frame_done is high, so the buffer is NOT cleared at the end of a frame.
// Positions at or above byte_cnt hold stale bytes from the previous message;
// consumers read the buffer only when frame_done is high, at which point
// byte_cnt bounds exactly which bytes are real.

`timescale 1ns/1ps

import rx_mac_pkg::*;

module rx_mac
    import msg_format_pkg::*;
(
    input  logic         clk,
    input  logic         rst_n,

    // ---- RX PHY interface ----------------------------------------------
    input  logic         byte_valid,
    input  logic [7:0]   rx_byte,

    // Parity error from the RX PHY -- soft reset.
    input  logic         par_val_rst,

    // ---- downstream back-pressure ---------------------------------------
    // High while rx_classifier is still holding a message that has not been
    // taken by the crossing. A completed frame WAITS IN THE BUFFER rather
    // than being announced, so the held message cannot be overwritten.
    input  logic         stall,

    // ---- frame out ------------------------------------------------------
    output logic [FRAME_W-1:0]    frame_buf,
    output logic [BYTE_CNT_W-1:0] byte_cnt,

    // One cycle each, mutually exclusive.
    //   frame_done : a complete, correctly delimited frame is in frame_buf
    //   frame_err  : a delimiter position held the wrong byte; frame abandoned
    output logic         frame_done,
    output logic         frame_err,

    output logic         mac_busy
);

// -------------------------------------------------------------------------
// Internal registers
// -------------------------------------------------------------------------
// Width of the byte-lane selector. MSG_BYTES_MAX lanes need this many bits.
localparam int LANE_W = $clog2(MSG_BYTES_MAX);

rx_mac_state_t cur_state, next_state;
logic [FRAME_W-1:0]    msg_buf;
logic [BYTE_CNT_W-1:0] byte_idx;
logic [7:0]            rx_byte_latch;

// -------------------------------------------------------------------------
// Start gate
// -------------------------------------------------------------------------
logic at_frame_start;
logic byte_accept;

assign at_frame_start = (cur_state == MAC_DONE) ||
                        (cur_state == MAC_ERR)  ||
                        (byte_idx  == '0);

assign byte_accept = byte_valid &&
                     (!at_frame_start || (rx_byte == CHAR_OPEN_BRACE));

// -------------------------------------------------------------------------
// Delimiter position test
//
// byte_idx counts bytes ALREADY STORED, so a count of 6, 11 or 16 means the
// newest byte sits at index 5, 10 or 15. Those counts are exactly the legal
// frame lengths -- MSG_BYTES_nGRP -- because a frame ends on its delimiter.
// The constants come from msg_format_pkg, so the positions are DEFINED in one
// place and merely referenced here.
// -------------------------------------------------------------------------
logic at_delim, at_last_delim;
logic last_is_close, last_is_comma;

assign at_delim = (byte_idx == BYTE_CNT_W'(MSG_BYTES_1GRP)) ||
                  (byte_idx == BYTE_CNT_W'(MSG_BYTES_2GRP)) ||
                  (byte_idx == BYTE_CNT_W'(MSG_BYTES_3GRP));

// Byte 15. Only '}' is legal here -- a comma would imply a fourth group,
// which does not exist. This is also the backstop that stops a frame running
// past the buffer.
assign at_last_delim = (byte_idx == BYTE_CNT_W'(MSG_BYTES_3GRP));

assign last_is_close = (rx_byte_latch == CHAR_CLOSE_BRACE);
assign last_is_comma = (rx_byte_latch == CHAR_COMMA);

// -------------------------------------------------------------------------
// State register
// -------------------------------------------------------------------------
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n || par_val_rst) cur_state <= MAC_IDLE;
    else                       cur_state <= next_state;
end

// -------------------------------------------------------------------------
// Next-state logic
// -------------------------------------------------------------------------
always_comb begin : next_state_logic
    next_state = cur_state;

    case (cur_state)
        // A byte that is not '{' at a frame start is dropped here: no state
        // change, so it is never stored and never counted.
        MAC_IDLE:
            if (byte_accept) next_state = MAC_STORE;

        MAC_STORE:
            next_state = MAC_CHK;

        MAC_CHK: begin
            if (!at_delim)
                next_state = MAC_IDLE;              // ordinary byte, continue
            else if (last_is_close)
                // Hold here, NOT in MAC_DONE. frame_done is a Moore decode
                // of MAC_DONE, so waiting there would hold it high for many
                // cycles and the classifier would accept the same frame over
                // and over. Waiting here keeps the completed frame in the
                // buffer and simply delays the announcement.
                //
                // mac_busy is (next_state != MAC_IDLE), so stalling here also
                // keeps mac_busy high, and UART_CTS already includes it. The
                // PC therefore pauses with no further change.
                next_state = stall ? MAC_CHK : MAC_DONE;
            else if (last_is_comma && !at_last_delim)
                next_state = MAC_IDLE;              // another group follows
            else
                next_state = MAC_ERR;               // wrong byte at a delimiter
        end

        MAC_DONE:
            if (byte_accept) next_state = MAC_STORE;
            else             next_state = MAC_IDLE;

        MAC_ERR:
            if (byte_accept) next_state = MAC_STORE;
            else             next_state = MAC_IDLE;

        default:
            next_state = MAC_IDLE;
    endcase
end : next_state_logic

// -------------------------------------------------------------------------
// Byte latch
// -------------------------------------------------------------------------
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        rx_byte_latch <= 8'h00;
    else if ((cur_state == MAC_IDLE || cur_state == MAC_DONE ||
              cur_state == MAC_ERR) && byte_accept)
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
// flop output and the clock enable of every msg_buf bit: a 7-bit shift, an
// 8-bit subtractor with its carry chain, and a variable part-select off that
// base. That was an 11-level cone ending on msg_buf_reg[*]/CE -- 3.518 ns
// logic + 3.960 ns routing = 7.478 ns against a 7.692 ns period,
// WNS -0.153 ns. The failure was on the ENABLE, not the data.
//
// THE FIX, RETAINED HERE: compute the enable ONE CYCLE EARLY and register it
// as a 16-bit one-hot, so msg_buf's clock enable comes straight off a flop.
//
// store_lane is byte_idx, except when the decision is taken from MAC_DONE or
// MAC_ERR, where byte_idx is being cleared on that same edge and the new
// frame therefore starts at lane 0.
// -----------------------------------------------------------------------------
logic                    store_next;
logic [LANE_W-1:0]       store_lane;
logic [MSG_BYTES_MAX-1:0] byte_we;

assign store_next = ((cur_state == MAC_IDLE) ||
                     (cur_state == MAC_DONE) ||
                     (cur_state == MAC_ERR)) && byte_accept;

assign store_lane = ((cur_state == MAC_DONE) || (cur_state == MAC_ERR))
                  ? '0 : byte_idx[LANE_W-1:0];

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n || par_val_rst) byte_we <= '0;
    else if (store_next)       byte_we <= MSG_BYTES_MAX'(1) << store_lane;
    else                       byte_we <= '0;
end

// -------------------------------------------------------------------------
// Frame buffer and byte counter
// -------------------------------------------------------------------------
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        msg_buf  <= '0;
        byte_idx <= '0;
    end
    else if (par_val_rst) begin
        byte_idx <= '0;
    end
    else begin
        // Fully unrolled: every bit index is a compile-time constant, so no
        // arithmetic survives in the cone reaching msg_buf's clock enable.
        for (int i = 0; i < MSG_BYTES_MAX; i++) begin
            if (byte_we[i]) msg_buf[FRAME_W-1 - i*8 -: 8] <= rx_byte_latch;
        end

        if (cur_state == MAC_STORE)
            byte_idx <= byte_idx + 1'b1;
        if (cur_state == MAC_DONE || cur_state == MAC_ERR)
            byte_idx <= '0;
    end
end

assign frame_buf = msg_buf;
assign byte_cnt  = byte_idx;

// -------------------------------------------------------------------------
// Outputs
//
// Moore decodes. During MAC_DONE the buffer holds the complete frame and
// byte_idx still holds its length, so every combinational consumer sees a
// settled frame in the cycle frame_done is high.
// -------------------------------------------------------------------------
assign frame_done = (cur_state == MAC_DONE);
assign frame_err  = (cur_state == MAC_ERR);

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) mac_busy <= 1'b0;
    else        mac_busy <= (next_state != MAC_IDLE);
end

`ifndef SYNTHESIS
    // The registered enable must coincide EXACTLY with the store cycle...
    a_we_matches_store: assert property (
        @(posedge clk) disable iff (!rst_n || par_val_rst)
        (|byte_we) == (cur_state == MAC_STORE)
    ) else $error("%m: byte_we and MAC_STORE disagree");

    a_we_onehot: assert property (
        @(posedge clk) disable iff (!rst_n || par_val_rst)
        $onehot0(byte_we)
    ) else $error("%m: byte_we is not one-hot");

    // ...and it must be the lane byte_idx names, which is what preserves
    // byte ordering.
    a_we_lane_correct: assert property (
        @(posedge clk) disable iff (!rst_n || par_val_rst)
        (cur_state == MAC_STORE) |-> byte_we[byte_idx[LANE_W-1:0]]
    ) else $error("%m: byte_we selected the wrong lane");

    // THE START GATE, CHECKED.
    a_frame_starts_with_brace: assert property (
        @(posedge clk) disable iff (!rst_n || par_val_rst)
        (cur_state == MAC_STORE && byte_idx == '0)
            |-> (rx_byte_latch == CHAR_OPEN_BRACE)
    ) else $error("%m: frame opened on a byte other than '{'");

    // A completed frame is always a legal length, and always ended on '}'.
    a_done_is_legal_length: assert property (
        @(posedge clk) disable iff (!rst_n || par_val_rst)
        frame_done |-> (byte_cnt == BYTE_CNT_W'(MSG_BYTES_1GRP) ||
                        byte_cnt == BYTE_CNT_W'(MSG_BYTES_2GRP) ||
                        byte_cnt == BYTE_CNT_W'(MSG_BYTES_3GRP))
    ) else $error("%m: frame_done at an illegal length");

    a_done_ends_on_brace: assert property (
        @(posedge clk) disable iff (!rst_n || par_val_rst)
        frame_done |-> last_is_close
    ) else $error("%m: frame_done without a closing brace");

    // Never both.
    a_done_xor_err: assert property (
        @(posedge clk) disable iff (!rst_n || par_val_rst)
        !(frame_done && frame_err)
    ) else $error("%m: frame_done and frame_err in the same cycle");

    // The frame can never run past the buffer.
    a_no_overrun: assert property (
        @(posedge clk) disable iff (!rst_n || par_val_rst)
        byte_cnt <= BYTE_CNT_W'(MSG_BYTES_3GRP)
    ) else $error("%m: byte_cnt exceeded the maximum frame length");
`endif

endmodule : rx_mac
