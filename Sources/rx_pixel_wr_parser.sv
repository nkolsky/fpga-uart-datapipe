// rx_pixel_wr_parser.sv
// ---------------------
// Stage 2A: purely combinational parser for the Final Project
// Single Pixel Write message (spec section 11):
//
//   {W<A2, A1, A0>, P<R, G, B>}        11 bytes
//
// Byte layout (byte 0 in msg_in[127:120], MSB first -- same convention
// as rx_mac's buffer packing and rx_parser's decode):
//
//   [0]  '{'          msg_in[127:120]
//   [1]  'W'          msg_in[119:112]
//   [2]  A2 (MSB)     msg_in[111:104]
//   [3]  A1           msg_in[103: 96]
//   [4]  A0 (LSB)     msg_in[ 95: 88]
//   [5]  ','          msg_in[ 87: 80]
//   [6]  'P'          msg_in[ 79: 72]
//   [7]  R            msg_in[ 71: 64]
//   [8]  G            msg_in[ 63: 56]
//   [9]  B            msg_in[ 55: 48]
//   [10] '}'          msg_in[ 47: 40]
//
// Bytes 11..15 (msg_in[39:0]) are NOT part of this message and are
// ignored entirely. They hold whatever rx_mac last left there.
//
// -----------------------------------------------------------------------
// ENCODING
// -----------------------------------------------------------------------
// Payload bytes are RAW BINARY, not ASCII decimal. This matches
// msg_composer.sv, which is already an exact implementation of the
// spec's Single Pixel Read in the opposite direction (msg[12..14] carry
// R, G, B as raw bytes). It also follows from arithmetic: three ASCII
// decimal digits cap an address at 999, well short of the 65536 pixels
// in a 256x256 image, whereas three binary bytes give 24 bits.
//
// Because payload bytes are binary, they can legitimately equal 0x7B
// ('{') or 0x7D ('}'). This parser must therefore never be used to find
// frame boundaries -- framing is count-based in rx_mac, driven by
// rx_msg_decode. Only the five FIXED byte positions are validated here,
// and none of them can be spoofed by a payload value because payload
// never occupies those positions.
//
// -----------------------------------------------------------------------
// ADDRESS SEMANTICS -- ASSUMPTION, PENDING CONFIRMATION
// -----------------------------------------------------------------------
// pix_addr is emitted as the raw 24-bit big-endian value {A2,A1,A0} with
// NO interpretation applied. The spec does not state its units. The
// working assumption for Stage 2B is a linear pixel index, giving
// word_addr = pix_addr[15:2] and byte lane = pix_addr[1:0] -- the only
// reading consistent with one message updating all three channel SRAMs
// at a shared word address and byte enable.
//
// That interpretation deliberately lives DOWNSTREAM of this module, so
// if the units turn out to be something else, only the address mapping
// changes and this parser does not.

`timescale 1ns/1ps

module rx_pixel_wr_parser
    import msg_pkg::*;
(
    // Complete frame from rx_mac, stable while msg_valid is high
    input  logic [127:0] msg_in,

    // Combinational outputs -- valid immediately, no clock
    output logic         pw_parse_valid,  // 1 = framing correct
    output logic         pw_parse_error,  // 1 = framing wrong
    output logic [23:0]  pw_addr,         // {A2,A1,A0}, raw, big-endian
    output logic [23:0]  pw_pixel         // {R,G,B}
);

    // -----------------------------------------------------------------
    // Framing validation -- the five fixed bytes must match exactly.
    //
    // No payload range check is applied. The address field is 24 bits by
    // construction and any value is structurally legal; bounds checking
    // against the image geometry belongs with whatever consumes the
    // address, not with frame validation.
    // -----------------------------------------------------------------
    always_comb begin : validate_frame
        pw_parse_valid = (msg_in[127:120] == CHAR_OPEN_BRACE)  &&  // '{'
                         (msg_in[119:112] == CHAR_W)           &&  // 'W'
                         (msg_in[ 87: 80] == CHAR_COMMA)       &&  // ','
                         (msg_in[ 79: 72] == CHAR_P)           &&  // 'P'
                         (msg_in[ 47: 40] == CHAR_CLOSE_BRACE);    // '}'

        pw_parse_error = ~pw_parse_valid;
    end : validate_frame

    // -----------------------------------------------------------------
    // Field extraction.
    //
    // Both fields are contiguous byte runs in the frame, so these are
    // pure slices -- no shifting or reassembly. Garbage when
    // pw_parse_valid is low; rx_classifier only latches them when
    // msg_valid, pw_parse_valid and msg_kind_q == MSG_PIX_WRITE all hold.
    // -----------------------------------------------------------------
    assign pw_addr  = msg_in[111:88];  // bytes 2,3,4  -> A2 A1 A0
    assign pw_pixel = msg_in[ 71:48];  // bytes 7,8,9  -> R  G  B

endmodule : rx_pixel_wr_parser
