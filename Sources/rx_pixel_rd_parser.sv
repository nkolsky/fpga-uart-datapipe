// rx_pixel_rd_parser.sv
// ---------------------
// Combinational parser for the Single Pixel Read request.
//
//   {R<R2,R1,R0>,C<C2,C1,C0>,P<X,X,X>}                   16 bytes
//
//   byte  0 '{'  [127:120]      byte  8 C1   [ 63: 56]
//   byte  1 'R'  [119:112]      byte  9 C0   [ 55: 48]
//   byte  2 R2   [111:104]      byte 10 ','  [ 47: 40]
//   byte  3 R1   [103: 96]      byte 11 'P'  [ 39: 32]
//   byte  4 R0   [ 95: 88]      byte 12 X    [ 31: 24]   don't care
//   byte  5 ','  [ 87: 80]      byte 13 X    [ 23: 16]   don't care
//   byte  6 'C'  [ 79: 72]      byte 14 X    [ 15:  8]   don't care
//   byte  7 C2   [ 71: 64]      byte 15 '}'  [  7:  0]
//
// This is the SAME layout msg_composer emits, so a request and its reply
// differ only in the three payload bytes. That is why the reply reuses
// msg_composer rather than defining a second frame format.
//
// -----------------------------------------------------------------------
// COORDINATE VALIDATION -- THE WHOLE 24-BIT FIELD, NOT THE LOW TEN BITS
// -----------------------------------------------------------------------
// Each coordinate occupies THREE bytes and is therefore a 24-bit field:
//
//     row field = msg_in[111:88]      (bytes 2,3,4  = R2,R1,R0)
//     col field = msg_in[ 71:48]      (bytes 7,8,9  = C2,C1,C0)
//
// Both are validated in full against the real image geometry:
//
//     row_raw < IMG_HEIGHT   and   col_raw < IMG_WIDTH
//
// An earlier revision of this module extracted the ten significant bits
// FIRST -- row = msg_in[97:88] -- and range-checked only those. That was
// wrong, and wrong in the exact way this module exists to prevent. It left
// byte 2, byte 7 and the top six bits of bytes 3 and 8 completely
// unexamined, so a request carrying
//
//     R2 = 0xFF, R1 = 0x00, R0 = 0x05
//
// presented row_raw = 0xFF0005 -- far outside a 256-row image -- and was
// accepted as a perfectly ordinary read of row 5. High-order rubbish
// aliased silently into a valid low coordinate, the SRAM was read, and a
// reply was returned for a request the host never made.
//
// The order of operations below is therefore deliberate and load-bearing:
// VALIDATE THE FULL FIELD, THEN EXTRACT. Extracting first and checking the
// remainder afterwards is the same computation only if you remember to
// check the remainder, and the failure mode when you forget is silent.
//
// Because IMG_HEIGHT and IMG_WIDTH are both 256, any nonzero bit above
// bit 7 makes the comparison fail on its own. The comparison is written
// against the geometry constants rather than as a zero-check on the upper
// bits so that a future non-power-of-two image size stays correct without
// anyone having to revisit this line.
//
// -----------------------------------------------------------------------
// EXTRACTION
// -----------------------------------------------------------------------
// pr_row / pr_col carry the supported ten bits, and are forced to zero
// unless pr_valid holds. A rejected request therefore cannot present a
// plausible-looking coordinate to anything downstream even if a consumer
// were to sample the buses without qualifying them -- which rx_classifier
// does not do, but defence in depth costs nothing here.
//
// The raw fields are also exposed, unqualified, purely for diagnostics.
//
// -----------------------------------------------------------------------
// WHAT IS NOT VALIDATED
// -----------------------------------------------------------------------
// The three don't-care payload bytes (12, 13, 14) are deliberately not
// checked: the specification marks them as such, and a request carrying
// stale colour data there is still a legal request.

`timescale 1ns/1ps

module rx_pixel_rd_parser
    import msg_pkg::*;
    import memory_pkg::*;
(
    input  logic [127:0] msg_in,

    output logic         pr_frame_ok,   // the seven fixed bytes are correct
    output logic         pr_coord_ok,   // BOTH full 24-bit fields in range
    output logic         pr_valid,      // both of the above
    output logic         pr_coord_err,  // framing good, coordinates unusable

    // Full unvalidated fields, for diagnostics only.
    output logic [23:0]  pr_row_raw,
    output logic [23:0]  pr_col_raw,

    // Supported coordinate bits, zero unless pr_valid.
    output logic [9:0]   pr_row,
    output logic [9:0]   pr_col
);

    // -----------------------------------------------------------------
    // Step 1 -- take the COMPLETE fields.
    // -----------------------------------------------------------------
    assign pr_row_raw = msg_in[111:88];   // bytes 2,3,4
    assign pr_col_raw = msg_in[ 71:48];   // bytes 7,8,9

    // -----------------------------------------------------------------
    // Step 2 -- framing.
    //
    // Seven fixed bytes. Byte 11 = 'P' is what separates this message from
    // the legacy {R,C,V} form, exactly as rx_msg_decode already
    // discriminates. Bytes 10 and 15 are checked here even though
    // rx_msg_decode does not look at them, so a frame that decodes as
    // MSG_PIX_READ but is malformed at its tail is rejected rather than
    // acted on.
    // -----------------------------------------------------------------
    assign pr_frame_ok = (msg_in[127:120] == CHAR_OPEN_BRACE)  &&  // '{'
                         (msg_in[119:112] == CHAR_R)           &&  // 'R'
                         (msg_in[ 87: 80] == CHAR_COMMA)       &&  // ','
                         (msg_in[ 79: 72] == CHAR_C)           &&  // 'C'
                         (msg_in[ 47: 40] == CHAR_COMMA)       &&  // ','
                         (msg_in[ 39: 32] == CHAR_P)           &&  // 'P'
                         (msg_in[  7:  0] == CHAR_CLOSE_BRACE);    // '}'

    // -----------------------------------------------------------------
    // Step 3 -- range-check the FULL fields, before any narrowing.
    // -----------------------------------------------------------------
    assign pr_coord_ok = (pr_row_raw < 24'(IMG_HEIGHT)) &&
                         (pr_col_raw < 24'(IMG_WIDTH));

    assign pr_valid     = pr_frame_ok && pr_coord_ok;
    assign pr_coord_err = pr_frame_ok && !pr_coord_ok;

    // -----------------------------------------------------------------
    // Step 4 -- and only now, extract.
    // -----------------------------------------------------------------
    assign pr_row = pr_valid ? pr_row_raw[9:0] : 10'd0;
    assign pr_col = pr_valid ? pr_col_raw[9:0] : 10'd0;

endmodule : rx_pixel_rd_parser
