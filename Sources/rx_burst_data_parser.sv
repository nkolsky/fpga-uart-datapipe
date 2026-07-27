// rx_burst_data_parser.sv
// -----------------------
// Stage 3, M2: purely combinational parser for an Image Burst Write DATA
// message. Extracts the four RGB pixels and validates the frame.
//
//   {<R0,G0,B0,R1>, <G1,B1,R2,G2>, <B2,R3,G3,B3>}          16 bytes
//
// -----------------------------------------------------------------------
// BYTE LAYOUT
// -----------------------------------------------------------------------
//   [0]  '{'        [4]  R1        [8]  R2        [12] R3
//   [1]  R0         [5]  ','       [9]  G2        [13] G3
//   [2]  G0         [6]  G1        [10] ','       [14] B3
//   [3]  B0         [7]  B1        [11] B2        [15] '}'
//
// so the four pixels are
//
//   pixel0 = bytes  1, 2, 3      -- contiguous
//   pixel1 = bytes  4, 6, 7      -- STRADDLES the comma at byte 5
//   pixel2 = bytes  8, 9, 11     -- STRADDLES the comma at byte 10
//   pixel3 = bytes 12,13,14      -- contiguous
//
// Byte n occupies msg_in[127 - 8*n -: 8], the same MSB-first convention as
// rx_mac, rx_pixel_wr_parser and rx_burst_hdr_parser.
//
// Strip bytes 0, 5, 10 and 15 and the remaining twelve are four consecutive
// RGB triplets in order -- the <...> groupings are frame punctuation, not a
// reordering. The commas simply happen to land mid-pixel, which is why
// pixel1 and pixel2 are not contiguous slices while pixel0 and pixel3 are.
// That asymmetry is exactly where a transposition would hide, so the
// testbench isolates each of the twelve payload bytes individually.
//
// -----------------------------------------------------------------------
// WHAT IS VALIDATED -- AND WHAT DELIBERATELY IS NOT
// -----------------------------------------------------------------------
// ONLY the four fixed bytes at 0, 5, 10 and 15 are checked.
//
// The twelve payload bytes are RAW BINARY colour values and every one of
// the 256 possible values is legal in every position. A payload byte may
// legitimately equal 0x7B ('{'), 0x7D ('}'), 0x2C (',') or any protocol
// opcode, and the parser must accept all of them. This is why framing is
// count-based in rx_mac rather than delimiter-based: scanning for a closing
// brace would truncate any message containing 0x7D in a colour channel.
//
// None of the four checked positions can be spoofed by payload, because
// payload never occupies them.
//
// -----------------------------------------------------------------------
// ASSUMPTION A4 -- PROVISIONAL
// -----------------------------------------------------------------------
// This module assumes the data message carries NO OPCODE at byte 1: that
// position holds R0, a raw colour value. It follows from the spec's own
// wording, "will enable opcode bypass for the duration of HxW pixels" --
// there would be nothing to bypass if byte 1 were an opcode.
//
// It is the reason bypass_active exists at all. If the assumption turns out
// to be wrong, rx_msg_decode could classify these frames directly, the
// bypass mechanism becomes unnecessary, and both this parser and
// rx_burst_ctrl change shape. Flagged here because the cost of being wrong
// is concentrated in Stage 3 and nowhere else.

`timescale 1ns/1ps

module rx_burst_data_parser
    import msg_pkg::*;
    import rx_burst_pkg::*;
(
    // Complete frame from rx_mac, stable while msg_valid is high
    input  logic [127:0] msg_in,

    // Framing: the four fixed bytes are correct
    output logic         data_frame_ok,
    output logic         data_error,

    // The four pixels, each {R,G,B}. pixels[0] is the first pixel of the
    // group, so rx_burst_ctrl can select one per clock with a 2-bit slot
    // counter: cmd_pixel = pixels[slot].
    //
    // Meaningful only when data_frame_ok; extraction is unconditional so a
    // diagnostic reading them on a rejected frame still sees real values
    // rather than noise.
    output logic [BURST_PIX_PER_MSG-1:0][BURST_PIX_W-1:0] pixels
);

    // -----------------------------------------------------------------
    // Framing check -- four fixed bytes only.
    // -----------------------------------------------------------------
    always_comb begin : validate_frame
        data_frame_ok = (msg_in[127:120] == CHAR_OPEN_BRACE)  &&  // byte  0
                        (msg_in[ 87: 80] == CHAR_COMMA)       &&  // byte  5
                        (msg_in[ 47: 40] == CHAR_COMMA)       &&  // byte 10
                        (msg_in[  7:  0] == CHAR_CLOSE_BRACE);    // byte 15
    end : validate_frame

    assign data_error = ~data_frame_ok;

    // -----------------------------------------------------------------
    // Pixel extraction.
    //
    // pixel0 and pixel3 fall between commas and are single slices.
    // pixel1 and pixel2 span a comma and are assembled from two.
    // -----------------------------------------------------------------
    always_comb begin : extract_pixels
        // bytes  1, 2, 3   -> R0 G0 B0
        pixels[0] = msg_in[119:96];

        // byte   4         -> R1
        // bytes  6, 7      -> G1 B1
        pixels[1] = {msg_in[95:88], msg_in[79:64]};

        // bytes  8, 9      -> R2 G2
        // byte  11         -> B2
        pixels[2] = {msg_in[63:48], msg_in[39:32]};

        // bytes 12,13,14   -> R3 G3 B3
        pixels[3] = msg_in[31:8];
    end : extract_pixels

endmodule : rx_burst_data_parser
