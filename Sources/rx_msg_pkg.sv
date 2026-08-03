// rx_msg_pkg.sv
// -------------
// Stage 2A: message classification types and frame lengths for the
// variable-length receive path.
//
// The Final Project message set (spec section 11) is NOT fixed-length.
// Summing the spec's own Fixed Bytes + Payload Bytes columns:
//
//   Register Write      {W<A2,A1,A0>, V<0,DH1,DH0>, V<0,DL1,DL0>}   16
//   Register Read       {R<A2,A1,A0>}                                6
//   Single Pixel Write  {W<A2,A1,A0>, P<R,G,B>}                     11
//   Single Pixel Read   {R<R2,R1,R0>, C<C2,C1,C0>, P<R,G,B>}        16
//   Image Burst Write   header {I<..>, H<..>, W<..>}                16
//                       data   {<R0,G0,B0,R1>, <..>, <..>}          16
//   Image Burst Read    {R<A2,A1,A0>, H<..>, W<..>}                 16
//
// with a Don't Cares column of 0 for both short forms -- i.e. they are
// genuinely short, not padded. Framing is therefore count-based off a
// decoded length, never delimiter-based: payload bytes are raw binary
// and can legitimately equal '{' (0x7B) or '}' (0x7D).
//
// The legacy Lab 10 control message {Rnnn,Cnnn,Vnnn} is 16 bytes and
// remains the ACTIVE register-control path throughout Stage 2A.
//
// THREE 16-BYTE MESSAGES ALL OPEN WITH 'R' AT BYTE 1 AND ',' AT BYTE 5:
//
//   legacy            {R nnn , C nnn , V nnn }   byte 6 = 'C', byte 11 = 'V'
//   Single Pixel Read {R <..> , C <..> , P <..>}  byte 6 = 'C', byte 11 = 'P'
//   Image Burst Read  {R <..> , H <..> , W <..>}  byte 6 = 'H'
//
// Byte 6 separates Image Burst Read from the other two; byte 11 then
// separates legacy from Single Pixel Read. All three are 16 bytes, so
// this affects CLASSIFICATION ONLY and never the framed length.
//
// Identifying legacy positively at byte 11 rather than by exclusion
// matters: without it, any {R..,C..,X..} frame -- including a looped-back
// Single Pixel Read reply -- would classify as MSG_LEGACY_RGF and could
// drive a spurious RGF write through chip_top's rgf_pc_wen term.

`timescale 1ns/1ps

package rx_msg_pkg;

    // -----------------------------------------------------------------
    // Frame lengths, in bytes
    // -----------------------------------------------------------------
    localparam int MSG_BYTES_MAX       = 16;  // longest frame; also the default
    localparam int MSG_BYTES_REG_READ  = 6;   // {R<A2,A1,A0>}
    localparam int MSG_BYTES_PIX_WRITE = 11;  // {W<A2,A1,A0>, P<R,G,B>}

    // Width of the byte counter. 5 bits holds 0..16 inclusive, matching
    // rx_mac's existing byte_idx.
    localparam int BYTE_CNT_W = 5;

    // -----------------------------------------------------------------
    // Message classification.
    //
    // Every member below has a live consumer. The comment column names the
    // module that acts on each kind.
    // -----------------------------------------------------------------
    // Widened to 4 bits when MSG_PIX_READ was added -- 9 members no
    // longer fit in 3.
    typedef enum logic [3:0] {
        MSG_UNKNOWN    = 4'd0,  // unrecognised / malformed -> classifier_error
        MSG_LEGACY_RGF = 4'd1,  // {Rnnn,Cnnn,Vnnn}         -> rx_parser, RGF write
        MSG_REG_WRITE  = 4'd2,  // {W<A>, V<..>, V<..>}     -> rx_reg_write_parser
        MSG_REG_READ   = 4'd3,  // {R<A>}                   -> rx_reg_read_parser
        MSG_PIX_WRITE  = 4'd4,  // {W<A>, P<R,G,B>}         -> rx_pixel_wr_parser
        MSG_PIX_READ   = 4'd5,  // {R<..>, C<..>, P<..>}    -> rx_pixel_rd_parser
        MSG_BURST_HDR  = 4'd6,  // {I<..>, H<..>, W<..>}    -> rx_burst_hdr_parser
        MSG_BURST_READ = 4'd7,  // {R<A>, H<..>, W<..>}     -> rx_burst_rd_parser
        MSG_BURST_DATA = 4'd8   // {<4 px>,<4 px>,<4 px>}   -> rx_burst_data_parser
    } msg_kind_t;

    // MSG_LEGACY_RGF and MSG_REG_WRITE are BOTH register-write paths and
    // both remain live. The legacy ASCII frame carries a single 8-bit value;
    // the binary frame carries a full 32-bit value and is the one the project
    // specification defines. They are distinct kinds decided at byte 1
    // ('R' vs 'W'), so a frame can never be both, and the legacy path is
    // retained unchanged for backwards compatibility.

endpackage : rx_msg_pkg
