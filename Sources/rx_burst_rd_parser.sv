// rx_burst_rd_parser.sv
// ---------------------
// Combinational parser for the Image Burst Read request.
//
//   {R<A2,A1,A0>,H<H2,H1,H0>,W<W2,W1,W0>}                16 bytes
//
//   byte  0 '{'  [127:120]      byte  8 H1   [ 63: 56]
//   byte  1 'R'  [119:112]      byte  9 H0   [ 55: 48]
//   byte  2 A2   [111:104]      byte 10 ','  [ 47: 40]
//   byte  3 A1   [103: 96]      byte 11 'W'  [ 39: 32]
//   byte  4 A0   [ 95: 88]      byte 12 W2   [ 31: 24]
//   byte  5 ','  [ 87: 80]      byte 13 W1   [ 23: 16]
//   byte  6 'H'  [ 79: 72]      byte 14 W0   [ 15:  8]
//   byte  7 H2   [ 71: 64]      byte 15 '}'  [  7:  0]
//
// 7 fixed bytes, 9 payload bytes, no don't-cares -- matching the guidelines
// table exactly.
//
// This is byte-for-byte the Image Burst WRITE header (rx_burst_hdr_parser)
// except for two things:
//
//   byte 1 is 'R' rather than 'I'
//   bytes 2..4 carry a real 24-bit start address rather than three
//     don't-care bytes
//
// The H and W field positions are identical, deliberately, so both parsers
// agree on the geometry encoding.
//
// -----------------------------------------------------------------------
// VALIDATION -- FULL FIELDS, THEN NARROW
// -----------------------------------------------------------------------
// Every one of the three payload fields is 24 bits and every one is checked
// at full width BEFORE any narrowing, for the same reason
// rx_pixel_rd_parser does it: taking the low bits first and range-checking
// those lets high-order rubbish alias into a legal value. A request with
// A2 = 0xFF and a low byte of 5 must be rejected, not read as address 5.
//
// -----------------------------------------------------------------------
// THE EXTENT CHECK
// -----------------------------------------------------------------------
// rx_burst_ctrl on the write side says, in its own comments:
//
//   "with base = 0 the region is guaranteed to fit, because [H,W are each
//    bounded by the image]. When the base becomes non-zero a base+extent
//    check must be added."
//
// Image Burst Read is exactly that case: the base is a host-supplied
// address. Both axes are therefore checked against the image bounds:
//
//     base_row + height <= IMG_HEIGHT
//     base_col + width  <= IMG_WIDTH
//
// The column check is the one that matters most. Without it a region
// starting near the right edge would walk off the end of its row and wrap
// into the next one, returning pixels the host never asked for while still
// reporting the requested H and W. That is silent wrong data, not an
// error, which is the worst failure mode available here.
//
// -----------------------------------------------------------------------
// WHAT IS NOT CHECKED
// -----------------------------------------------------------------------
// H*W is NOT required to be a multiple of four. The reply stream pads the
// final message, so any legal rectangle is accepted.

`timescale 1ns/1ps

module rx_burst_rd_parser
    import msg_pkg::*;
    import memory_pkg::*;
#(
    parameter int MAX_HEIGHT = IMG_HEIGHT,   // 256
    parameter int MAX_WIDTH  = IMG_WIDTH,    // 256
    parameter int DIM_W      = 10            // carries 0..256
)(
    input  logic [127:0] msg_in,

    output logic         br_frame_ok,   // the seven fixed bytes are correct
    output logic         br_dims_ok,    // H and W individually in range
    output logic         br_addr_ok,    // start address inside the image
    output logic         br_extent_ok,  // base + extent fits, both axes
    output logic         br_valid,      // all of the above
    output logic         br_err,        // framing good, geometry unusable

    // Full unvalidated fields, diagnostics only.
    output logic [23:0]  br_addr_raw,
    output logic [23:0]  br_h_raw,
    output logic [23:0]  br_w_raw,

    // Narrowed geometry, zero unless br_valid.
    output logic [DIM_W-1:0] br_base_row,
    output logic [DIM_W-1:0] br_base_col,
    output logic [DIM_W-1:0] br_height,
    output logic [DIM_W-1:0] br_width
);

    localparam int TOTAL_PIXELS = MAX_HEIGHT * MAX_WIDTH;   // 65536

    // -----------------------------------------------------------------
    // Step 1 -- take the COMPLETE fields.
    // -----------------------------------------------------------------
    assign br_addr_raw = msg_in[111:88];   // bytes 2,3,4
    assign br_h_raw    = msg_in[ 71:48];   // bytes 7,8,9
    assign br_w_raw    = msg_in[ 31: 8];   // bytes 12,13,14

    // -----------------------------------------------------------------
    // Step 2 -- framing. Seven fixed bytes.
    // -----------------------------------------------------------------
    assign br_frame_ok = (msg_in[127:120] == CHAR_OPEN_BRACE)  &&  // '{'
                         (msg_in[119:112] == CHAR_R)           &&  // 'R'
                         (msg_in[ 87: 80] == CHAR_COMMA)       &&  // ','
                         (msg_in[ 79: 72] == CHAR_H)           &&  // 'H'
                         (msg_in[ 47: 40] == CHAR_COMMA)       &&  // ','
                         (msg_in[ 39: 32] == CHAR_W)           &&  // 'W'
                         (msg_in[  7:  0] == CHAR_CLOSE_BRACE);    // '}'

    // -----------------------------------------------------------------
    // Step 3 -- range-check the FULL fields.
    // -----------------------------------------------------------------
    assign br_dims_ok = (br_h_raw != 24'd0) &&
                        (br_h_raw <= 24'(MAX_HEIGHT)) &&
                        (br_w_raw != 24'd0) &&
                        (br_w_raw <= 24'(MAX_WIDTH));

    assign br_addr_ok = (br_addr_raw < 24'(TOTAL_PIXELS));

    // Decompose the linear start address. IMG_WIDTH is a power of two, so
    // both of these collapse to wiring; written as arithmetic so a future
    // non-power-of-two image stays correct.
    logic [23:0] base_row_full, base_col_full;
    assign base_row_full = br_addr_raw / 24'(MAX_WIDTH);
    assign base_col_full = br_addr_raw % 24'(MAX_WIDTH);

    // Extent, evaluated at full width so the sum cannot wrap.
    assign br_extent_ok = ((base_row_full + br_h_raw) <= 24'(MAX_HEIGHT)) &&
                          ((base_col_full + br_w_raw) <= 24'(MAX_WIDTH));

    assign br_valid = br_frame_ok && br_dims_ok && br_addr_ok && br_extent_ok;
    assign br_err   = br_frame_ok && !(br_dims_ok && br_addr_ok && br_extent_ok);

    // -----------------------------------------------------------------
    // Step 4 -- and only now, narrow.
    // -----------------------------------------------------------------
    assign br_base_row = br_valid ? DIM_W'(base_row_full) : '0;
    assign br_base_col = br_valid ? DIM_W'(base_col_full) : '0;
    assign br_height   = br_valid ? DIM_W'(br_h_raw)      : '0;
    assign br_width    = br_valid ? DIM_W'(br_w_raw)      : '0;

endmodule : rx_burst_rd_parser
