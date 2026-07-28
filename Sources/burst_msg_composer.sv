// burst_msg_composer.sv
// ---------------------
// Packs four pixels into one 16-byte Image Burst Read reply.
//
//   {<R0,G0,B0,R1>,<G1,B1,R2,G2>,<B2,R3,G3,B3>}
//
// Three 32-bit groups separated by commas: 4 fixed bytes plus 12 payload
// bytes, carrying four RGB triplets.
//
// -----------------------------------------------------------------------
// THIS IS THE EXACT INVERSE OF rx_burst_data_parser
// -----------------------------------------------------------------------
// The same packing already carries Image Burst WRITE data into the design,
// where rx_burst_data_parser unpacks it:
//
//     pixels[0] = msg_in[119:96];
//     pixels[1] = {msg_in[95:88], msg_in[79:64]};
//     pixels[2] = {msg_in[63:48], msg_in[39:32]};
//     pixels[3] = msg_in[31:8];
//
// Note pixels 1 and 2 STRADDLE the comma separators -- R1 sits at the end
// of group one while G1 and B1 begin group two. That is the detail most
// likely to be got wrong by writing this module from the format string
// instead of from the parser, so tb_burst_msg_composer round-trips this
// output back through rx_burst_data_parser and requires the identity to
// hold. The parser is hardware-verified on the write path, so that test
// checks this module against working silicon rather than against a reading
// of the specification.
//
// -----------------------------------------------------------------------
// BYTE ORDER
// -----------------------------------------------------------------------
// rx_burst_data_parser works in RECEIVE order, where byte 0 sits at
// msg_in[127:120]. This module emits TRANSMIT order, where byte 0 sits at
// msg[7:0], because that is what tx_mac consumes:
//
//     phy_data <= msg_buf[byte_idx * 8 +: 8];      // tx_mac.sv
//
// The two orders are byte-reverses of one another. The layout below is
// written out byte by byte rather than as a reversal expression so the
// mapping is checkable by eye against the format string.
//
//   byte  0 '{'   msg[  7:  0]      byte  8 R2    msg[ 71: 64]
//   byte  1 R0    msg[ 15:  8]      byte  9 G2    msg[ 79: 72]
//   byte  2 G0    msg[ 23: 16]      byte 10 ','   msg[ 87: 80]
//   byte  3 B0    msg[ 31: 24]      byte 11 B2    msg[ 95: 88]
//   byte  4 R1    msg[ 39: 32]      byte 12 R3    msg[103: 96]
//   byte  5 ','   msg[ 47: 40]      byte 13 G3    msg[111:104]
//   byte  6 G1    msg[ 55: 48]      byte 14 B3    msg[119:112]
//   byte  7 B1    msg[ 63: 56]      byte 15 '}'   msg[127:120]
//
// -----------------------------------------------------------------------
// PADDING
// -----------------------------------------------------------------------
// This module has no notion of padding. burst_rd_ctrl zero-fills unused
// slots before presenting them here, so a padded slot arrives as an
// ordinary pixel whose value happens to be 24'h000000. Keeping the
// knowledge of which slots are real in the controller means this module
// stays a pure format converter with nothing to get out of step.

`timescale 1ns/1ps

module burst_msg_composer
    import msg_pkg::*;
    import rx_burst_pkg::*;
(
    input  logic [BURST_PIX_PER_MSG-1:0][BURST_PIX_W-1:0] pixels,
    output logic [127:0]                                  msg
);

    // Named slices, purely for readability below.
    logic [7:0] r0, g0, b0, r1, g1, b1, r2, g2, b2, r3, g3, b3;

    assign {r0, g0, b0} = pixels[0];
    assign {r1, g1, b1} = pixels[1];
    assign {r2, g2, b2} = pixels[2];
    assign {r3, g3, b3} = pixels[3];

    always_comb begin : pack
        msg = 128'd0;

        msg[  7:  0] = CHAR_OPEN_BRACE;   // byte  0
        // group 1: <R0, G0, B0, R1>
        msg[ 15:  8] = r0;                // byte  1
        msg[ 23: 16] = g0;                // byte  2
        msg[ 31: 24] = b0;                // byte  3
        msg[ 39: 32] = r1;                // byte  4
        msg[ 47: 40] = CHAR_COMMA;        // byte  5
        // group 2: <G1, B1, R2, G2>
        msg[ 55: 48] = g1;                // byte  6
        msg[ 63: 56] = b1;                // byte  7
        msg[ 71: 64] = r2;                // byte  8
        msg[ 79: 72] = g2;                // byte  9
        msg[ 87: 80] = CHAR_COMMA;        // byte 10
        // group 3: <B2, R3, G3, B3>
        msg[ 95: 88] = b2;                // byte 11
        msg[103: 96] = r3;                // byte 12
        msg[111:104] = g3;                // byte 13
        msg[119:112] = b3;                // byte 14
        msg[127:120] = CHAR_CLOSE_BRACE;  // byte 15
    end : pack

endmodule : burst_msg_composer
