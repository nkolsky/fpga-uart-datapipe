// rx_burst_hdr_parser.sv
// ----------------------
// Stage 3, M1: purely combinational parser for the Image Burst Write header.
//
//   {I<0,0,0>, H<H2,H1,H0>, W<W2,W1,W0>}          16 bytes
//
// Validates the seven fixed bytes, extracts the two 24-bit dimensions, and
// range-checks them. Nothing sequential -- rx_burst_ctrl latches the result
// on the msg_valid cycle.
//
// -----------------------------------------------------------------------
// BYTE POSITIONS
// -----------------------------------------------------------------------
// Byte n lives at msg_in[127 - 8*n -: 8], the same MSB-first convention
// rx_mac uses and rx_pixel_wr_parser already relies on.
//
//   [0]  '{'   msg_in[127:120]        [8]  H1    msg_in[63:56]
//   [1]  'I'   msg_in[119:112]        [9]  H0    msg_in[55:48]
//   [2]  0     msg_in[111:104]        [10] ','   msg_in[47:40]
//   [3]  0     msg_in[103: 96]        [11] 'W'   msg_in[39:32]
//   [4]  0     msg_in[ 95: 88]        [12] W2    msg_in[31:24]
//   [5]  ','   msg_in[ 87: 80]        [13] W1    msg_in[23:16]
//   [6]  'H'   msg_in[ 79: 72]        [14] W0    msg_in[15: 8]
//   [7]  H2    msg_in[ 71: 64]        [15] '}'   msg_in[ 7: 0]
//
// so   height = msg_in[71:48]     width = msg_in[31:8]
//
// -----------------------------------------------------------------------
// WHAT IS AND IS NOT CHECKED
// -----------------------------------------------------------------------
// Bytes 2..4 are the spec's don't-care field and are DELIBERATELY NOT
// validated. Rejecting a header because a don't-care byte held an
// unexpected value would reject traffic the specification permits.
//
// ASSUMPTION (A3): those three bytes really are don't-cares. Image Burst
// READ carries an address in exactly this position -- {R<A2,A1,A0>, H<..>,
// W<..>} -- so the write form is conspicuously the only message with a
// blank group A. If they turn out to be a start address, this parser gains
// a third output and rx_burst_ctrl loads base_row/base_col from it instead
// of from zero. Nothing else changes, which is why the base is a register
// in the controller rather than a hardwired constant.
//
// -----------------------------------------------------------------------
// DIMENSION VALIDATION
// -----------------------------------------------------------------------
// A dimension must be non-zero and must not exceed the image. A zero
// dimension is rejected rather than silently treated as a no-op burst: it
// would otherwise arm bypass_active for a burst that can never complete,
// leaving the receive path permanently misinterpreting normal frames as
// pixel data.
//
// Only the EXTENT is checked here. Whether base + extent fits the image
// depends on the origin, which lives in rx_burst_ctrl -- this module never
// sees it. Keeping the two checks apart is what allows the origin to become
// non-zero later without touching this file.
//
// hdr_frame_ok and hdr_dims_ok are exposed separately so the controller can
// report a malformed frame distinctly from an out-of-range rectangle. Both
// are real host errors, but they mean different things.

`timescale 1ns/1ps

module rx_burst_hdr_parser
    import msg_pkg::*;
    import rx_burst_pkg::*;
#(
    // Default to the project geometry; overridable so the testbench can
    // exercise boundary behaviour without rebuilding memory_pkg.
    parameter int MAX_HEIGHT = memory_pkg::IMG_HEIGHT,   // 256
    parameter int MAX_WIDTH  = memory_pkg::IMG_WIDTH     // 256
)(
    // Complete frame from rx_mac, stable while msg_valid is high
    input  logic [127:0]           msg_in,

    // Framing: the seven fixed bytes are all correct
    output logic                   hdr_frame_ok,
    // Dimensions: both non-zero and within the image
    output logic                   hdr_dims_ok,
    // Both of the above -- safe to start a burst
    output logic                   hdr_valid,

    // Extracted dimensions, raw. Meaningful only when hdr_frame_ok.
    output logic [BURST_DIM_W-1:0] height,
    output logic [BURST_DIM_W-1:0] width
);

    // -----------------------------------------------------------------
    // Field extraction -- both fields are contiguous byte runs, so these
    // are pure slices with no reassembly.
    // -----------------------------------------------------------------
    assign height = msg_in[71:48];   // bytes 7,8,9   -> H2 H1 H0
    assign width  = msg_in[31: 8];   // bytes 12,13,14 -> W2 W1 W0

    // -----------------------------------------------------------------
    // Framing check -- the seven fixed bytes.
    //
    // None of these positions can be spoofed by payload, because payload
    // never occupies them. That is what makes count-based framing safe for
    // a message whose data bytes are raw binary.
    // -----------------------------------------------------------------
    always_comb begin : validate_frame
        hdr_frame_ok = (msg_in[127:120] == CHAR_OPEN_BRACE)  &&  // '{'
                       (msg_in[119:112] == CHAR_I)           &&  // 'I'
                       (msg_in[ 87: 80] == CHAR_COMMA)       &&  // ','
                       (msg_in[ 79: 72] == CHAR_H)           &&  // 'H'
                       (msg_in[ 47: 40] == CHAR_COMMA)       &&  // ','
                       (msg_in[ 39: 32] == CHAR_W)           &&  // 'W'
                       (msg_in[  7:  0] == CHAR_CLOSE_BRACE);    // '}'
        // Bytes 2..4 intentionally unchecked -- see the header.
    end : validate_frame

    // -----------------------------------------------------------------
    // Dimension range check
    // -----------------------------------------------------------------
    always_comb begin : validate_dims
        hdr_dims_ok = (height != '0) &&
                      (height <= BURST_DIM_W'(MAX_HEIGHT)) &&
                      (width  != '0) &&
                      (width  <= BURST_DIM_W'(MAX_WIDTH));
    end : validate_dims

    assign hdr_valid = hdr_frame_ok && hdr_dims_ok;

endmodule : rx_burst_hdr_parser
