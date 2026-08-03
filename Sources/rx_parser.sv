// rx_parser.sv
// ------------
// Purely combinational parser for the RX (PC -> FPGA) command message.
//
// Lab 9 requirement: "Parser: Combo -> Generates error for invalid messages
// and data value combinations"
//
// UPDATED to match the confirmed real message format, verified against:
//   (a) the course-provided PC-side script, which sends messages like
//       '{R149,C052,V208}' -- 16 ASCII characters
//   (b) the rx_parser.sv from the earlier lab that first introduced the
//       UART RX MAC (unchanged in Lab 8/9 per spec: "No change from
//       previous labs"), which used this exact same field layout
//
// This REPLACES a previous version of this file that incorrectly assumed
// a binary-packed {R,pad,row,C,pad,col,P,R_ch,G_ch,B_ch} layout copied
// from the TX-side msg_composer format. That format does not match real
// PC traffic and has been confirmed wrong.
//
// Message format (16 bytes, byte 0 in msg_in[127:120]):
//   [0]  '{'         [1]  'R'
//   [2]  row hundreds digit (ASCII '0'-'9')
//   [3]  row tens digit
//   [4]  row ones digit
//   [5]  ','         [6]  'C'
//   [7]  col hundreds digit
//   [8]  col tens digit
//   [9]  col ones digit
//   [10] ','         [11] 'V'
//   [12] val hundreds digit
//   [13] val tens digit
//   [14] val ones digit
//   [15] '}'
//
// row/col/val are each reconstructed from 3 ASCII decimal digits
// (hundreds*100 + tens*10 + ones), matching the earlier lab's rx_parser.
//
// Validation (per Lab 9 spec, two independent failure classes):
//   1. Invalid message (framing): all 7 fixed framing bytes must match
//      exactly ('{', 'R', ',', 'C', ',', 'V', '}').
//   2. Invalid data value combination: each of the 9 digit bytes must be
//      a valid ASCII decimal digit ('0'-'9'); anything else (e.g. a
//      corrupted byte, or a digit-shaped value from a differently
//      framed packet) is flagged rather than silently decoded as
//      garbage.
//
// NOTE: val is a single 8-bit value (per the confirmed format), not a
// 24-bit RGB triplet. It is zero-extended into the 24-bit `pixel` output
// to keep the existing rx_classifier/sequencer port widths unchanged.
//
// -----------------------------------------------------------------------
// HOW THE THREE FIELDS ARE USED DOWNSTREAM -- RESOLVED
// -----------------------------------------------------------------------
// This carried a TODO asking whether `pixel` should travel differently
// once the RGF dispatch semantics were settled. They are settled, and the
// answer is no -- the zero-extension is the final arrangement. chip_top's
// dispatcher (see its "Config RGF + dispatcher" section) reads the three
// outputs as:
//
//   row   -> REGISTER INDEX, not a byte address. Index N maps to byte
//            address N*4, matching rgf_pkg's 4-byte stride:
//            0=IMG_STATUS, 1=IMG_TX_MON, 2=IMG_CTRL, 3=FIFO_STATUS,
//            4=CLK_CTRL, 5=PARITY_FAULT_CNT
//   col   -> READ/WRITE OPCODE, by parity. EVEN = write (every legacy
//            command historically sent col=000, so existing tooling is
//            unaffected), ODD = read.
//   pixel -> WRITE DATA, zero-extended to the RGF's 32-bit pc_wdata.
//
// The 8-bit ceiling on write data is a real limitation of this legacy
// format and is the reason the binary Register Write message
// {W<A>,V<..>,V<..>} exists -- that path (rx_reg_write_parser) is the
// only one that can write the upper 24 bits of a register. Both paths
// are retained deliberately and are distinct msg_kinds, so they can
// never collide on one frame.
//
// Examples: {R002,C000,V001} writes IMG_CTRL.start (trigger an image
// send); {R001,C001,V000} reads IMG_TX_MON (and read-to-clears it,
// re-arming the interlock).

`timescale 1ns/1ps

import msg_pkg::*;

module rx_parser (
    // Message from rx_mac (stable when msg_valid from MAC is high)
    input  logic [127:0] msg_in,

    // Combinational outputs - valid immediately, no clock
    output logic         parse_valid,   // 1 = message format correct
    output logic         parse_error,   // 1 = framing or digit error
    output logic [9:0]   row,           // extracted row index (0-255 range)
    output logic [9:0]   col,           // extracted column index (0-255 range)
    output logic [23:0]  pixel          // extracted value, zero-extended
);

// -------------------------------------------------------------------------
// Digit byte extraction (raw ASCII, before validation/decode)
// -------------------------------------------------------------------------
logic [7:0] row_d2, row_d1, row_d0;
logic [7:0] col_d2, col_d1, col_d0;
logic [7:0] val_d2, val_d1, val_d0;

assign row_d2 = msg_in[111:104];
assign row_d1 = msg_in[103: 96];
assign row_d0 = msg_in[ 95: 88];

assign col_d2 = msg_in[ 71: 64];
assign col_d1 = msg_in[ 63: 56];
assign col_d0 = msg_in[ 55: 48];

assign val_d2 = msg_in[ 31: 24];
assign val_d1 = msg_in[ 23: 16];
assign val_d0 = msg_in[ 15:  8];

// -------------------------------------------------------------------------
// Digit validity check ("data value combination" per spec wording) --
// every digit byte must be ASCII '0'-'9'
// -------------------------------------------------------------------------
logic digits_valid;
always_comb begin : check_digits
    digits_valid =
        (row_d2 >= ASCII_ZERO) && (row_d2 <= (ASCII_ZERO + 8'd9)) &&
        (row_d1 >= ASCII_ZERO) && (row_d1 <= (ASCII_ZERO + 8'd9)) &&
        (row_d0 >= ASCII_ZERO) && (row_d0 <= (ASCII_ZERO + 8'd9)) &&
        (col_d2 >= ASCII_ZERO) && (col_d2 <= (ASCII_ZERO + 8'd9)) &&
        (col_d1 >= ASCII_ZERO) && (col_d1 <= (ASCII_ZERO + 8'd9)) &&
        (col_d0 >= ASCII_ZERO) && (col_d0 <= (ASCII_ZERO + 8'd9)) &&
        (val_d2 >= ASCII_ZERO) && (val_d2 <= (ASCII_ZERO + 8'd9)) &&
        (val_d1 >= ASCII_ZERO) && (val_d1 <= (ASCII_ZERO + 8'd9)) &&
        (val_d0 >= ASCII_ZERO) && (val_d0 <= (ASCII_ZERO + 8'd9));
end : check_digits

// -------------------------------------------------------------------------
// Frame validation - all structural bytes must match
// -------------------------------------------------------------------------
logic frame_valid;
always_comb begin : validate_frame
    frame_valid = (msg_in[127:120] == CHAR_OPEN_BRACE)  &&  // '{'
                  (msg_in[119:112] == CHAR_R)            &&  // 'R'
                  (msg_in[ 87: 80] == CHAR_COMMA)        &&  // ','
                  (msg_in[ 79: 72] == CHAR_C)            &&  // 'C'
                  (msg_in[ 47: 40] == CHAR_COMMA)        &&  // ','
                  (msg_in[ 39: 32] == CHAR_V)            &&  // 'V'
                  (msg_in[  7:  0] == CHAR_CLOSE_BRACE);     // '}'
end : validate_frame

always_comb begin : validate
    parse_valid = frame_valid && digits_valid;
    parse_error = ~parse_valid;
end : validate

// -------------------------------------------------------------------------
// Field decode - ASCII decimal digits to binary
// (garbage when parse_valid is low; classifier only latches when
//  parse_valid & msg_valid are both high, same as before)
//
// Each 3-digit field is hundreds*100 + tens*10 + ones, using each ASCII
// digit byte minus ASCII_ZERO to get its 0-9 value. Widened to int before
// the multiply/add so nothing truncates mid-calculation, then explicitly
// cast down to the output width at the end.
// -------------------------------------------------------------------------
always_comb begin : decode_fields
    int row_val, col_val, val_val;

    row_val = (int'(row_d2) - int'(ASCII_ZERO)) * 100
            + (int'(row_d1) - int'(ASCII_ZERO)) * 10
            + (int'(row_d0) - int'(ASCII_ZERO));

    col_val = (int'(col_d2) - int'(ASCII_ZERO)) * 100
            + (int'(col_d1) - int'(ASCII_ZERO)) * 10
            + (int'(col_d0) - int'(ASCII_ZERO));

    val_val = (int'(val_d2) - int'(ASCII_ZERO)) * 100
            + (int'(val_d1) - int'(ASCII_ZERO)) * 10
            + (int'(val_d0) - int'(ASCII_ZERO));

    row   = 10'(row_val);   // 0-999 fits in 10 bits
    col   = 10'(col_val);
    pixel = 24'(val_val);   // zero-extended into the 24-bit field
end : decode_fields

endmodule : rx_parser