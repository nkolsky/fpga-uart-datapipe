// rx_reg_write_parser.sv
// ----------------------
// Combinational parser for the binary Register Write request.
//
//   {W<A2,A1,A0>, V<0,DH1,DH0>, V<0,DL1,DL0>}            16 bytes
//
//   byte  0 '{'  [127:120]      byte  8 DH1  [ 63: 56]
//   byte  1 'W'  [119:112]      byte  9 DH0  [ 55: 48]
//   byte  2 A2   [111:104]      byte 10 ','  [ 47: 40]
//   byte  3 A1   [103: 96]      byte 11 'V'  [ 39: 32]
//   byte  4 A0   [ 95: 88]      byte 12 0    [ 31: 24]   reserved
//   byte  5 ','  [ 87: 80]      byte 13 DL1  [ 23: 16]
//   byte  6 'V'  [ 79: 72]      byte 14 DL0  [ 15:  8]
//   byte  7 0    [ 71: 64]      byte 15 '}'  [  7:  0]
//              reserved
//
// Byte n occupies msg_in[127 - 8*n -: 8], the same MSB-first convention used
// by rx_mac, rx_reg_read_parser, rx_pixel_wr_parser and every burst parser.
//
// This is the write counterpart of rx_reg_read_parser. The address field sits
// in exactly the same position and orientation in both, deliberately, so the
// two agree on the register addressing scheme by construction rather than by
// inspection.
//
// -----------------------------------------------------------------------
// RESERVED BYTES 7 AND 12 -- ASSUMPTION (A5)
// -----------------------------------------------------------------------
// The specification renders these positions as a literal 0:
//
//     {W<A2,A1,A0>, V<0,DH1,DH0>, V<0,DL1,DL0>}
//
// but nowhere states NORMATIVELY that a receiver must reject a frame whose
// reserved bytes hold something else. They are therefore treated as
// DON'T-CARES and are NOT validated here.
//
// This follows the convention already established by rx_burst_hdr_parser
// (assumption A3), which treats the identically-positioned blank group in the
// Image Burst Write header the same way:
//
//     {I<0,0,0>, H<H2,H1,H0>, W<W2,W1,W0>}
//
// Consistency across the protocol is the point. A receiver that rejects a
// reserved byte in one message and ignores it in another is harder to reason
// about than one that is uniformly permissive, and the permissive reading is
// the one that cannot reject traffic the specification allows.
//
// It also removes a real interoperability hazard: a PC-side script that
// writes ASCII '0' (0x30) rather than binary 0x00 into those positions would
// have every Register Write refused, with no diagnostic distinguishing it
// from a framing fault.
//
// If a later revision of the specification makes zero normative, tightening
// this is additive -- an rw_rsvd_ok term ANDed into rw_frame_ok below -- and
// changes no interface.
//
// -----------------------------------------------------------------------
// WHAT IS VALIDATED
// -----------------------------------------------------------------------
// The seven framing bytes: 0, 1, 5, 6, 10, 11 and 15.
//
// The four payload bytes DH1, DH0, DL1, DL0 are RAW BINARY register data.
// Every one of the 256 values is legal in every position, including 0x7B
// ('{'), 0x7D ('}'), 0x2C (',') and 0x56 ('V'), so none of them is checked
// against anything. Framing is established by POSITION, never by content --
// the same rule rx_burst_data_parser relies on.
//
// -----------------------------------------------------------------------
// ADDRESS VALIDATION -- FULL FIELD, THEN NARROW
// -----------------------------------------------------------------------
// The A field is 24 bits and is checked at FULL WIDTH before any narrowing,
// exactly as rx_reg_read_parser does. Taking the low bits first and
// range-checking those lets high-order rubbish alias onto a legal register:
// a request with A2 = 0xFF and a low byte of 0x08 must be REJECTED, not
// executed as a write to IMG_CTRL.
//
//     addr[23:6] == 0        fits the 6-bit RGF port
//     addr[1:0]  == 2'b00    word aligned (registers are 4 bytes apart)
//
// An out-of-range address is rejected, never truncated. Aliasing 0x000104
// down to 0x04 would write IMG_TX_MON in response to a request that named a
// different register.
//
// -----------------------------------------------------------------------
// DATA BYTE ORDER -- THE ONE THING MOST LIKELY TO BE WRONG
// -----------------------------------------------------------------------
// The 32-bit value is assembled big-endian, most significant byte first:
//
//     rw_data = {DH1, DH0, DL1, DL0}
//             = {msg_in[63:56], msg_in[55:48], msg_in[23:16], msg_in[15:8]}
//
// which is bit-identical to the order tx_reply_ctrl already emits for a
// Register Read reply:
//
//     byte 1  pc_rdata[31:24]   V3      <- DH1
//     byte 2  pc_rdata[23:16]   V2      <- DH0
//     byte 3  pc_rdata[15: 8]   V1      <- DL1
//     byte 4  pc_rdata[ 7: 0]   V0      <- DL0
//
// So a Register Write followed by a Register Read of the same address returns
// the four payload bytes UNCHANGED AND IN THE SAME ORDER. That identity is
// what tb_reg_write's round-trip test asserts, and it is the only check that
// forces this parser and tx_reply_ctrl to agree: each was written from its own
// reading of the specification, and each could pass its own unit test while
// the pair disagreed on byte order.
//
// Note the two halves are NOT contiguous in the frame -- the reserved byte at
// 12 and the ',' / 'V' at 10 and 11 sit between DH0 and DL1. Writing this as
// a single slice msg_in[63:8] would silently capture those four bytes instead
// and is the specific transposition the unit test walks each payload byte to
// rule out.

`timescale 1ns/1ps

module rx_reg_write_parser
    import msg_pkg::*;
(
    input  logic [127:0] msg_in,

    output logic         rw_frame_ok,   // the seven fixed bytes are correct
    output logic         rw_addr_ok,    // in range and word aligned
    output logic         rw_valid,      // both of the above
    output logic         rw_addr_err,   // framing good, address unusable
    output logic [23:0]  rw_addr,       // raw 24-bit byte address
    output logic [5:0]   rw_rgf_addr,   // addr[5:0], valid only when rw_valid
    output logic [31:0]  rw_data        // {DH1,DH0,DL1,DL0}, valid on rw_valid
);

    // -----------------------------------------------------------------
    // Field extraction
    // -----------------------------------------------------------------
    assign rw_addr = msg_in[111:88];               // bytes 2, 3, 4

    assign rw_data = { msg_in[63:56],              // byte  8  DH1
                       msg_in[55:48],              // byte  9  DH0
                       msg_in[23:16],              // byte 13  DL1
                       msg_in[15: 8] };            // byte 14  DL0

    // -----------------------------------------------------------------
    // Framing. Seven fixed bytes; bytes 7 and 12 deliberately absent
    // from this expression -- see the A5 note in the header.
    // -----------------------------------------------------------------
    assign rw_frame_ok = (msg_in[127:120] == CHAR_OPEN_BRACE)  &&  // byte  0
                         (msg_in[119:112] == CHAR_W)           &&  // byte  1
                         (msg_in[ 87: 80] == CHAR_COMMA)       &&  // byte  5
                         (msg_in[ 79: 72] == CHAR_V)           &&  // byte  6
                         (msg_in[ 47: 40] == CHAR_COMMA)       &&  // byte 10
                         (msg_in[ 39: 32] == CHAR_V)           &&  // byte 11
                         (msg_in[  7:  0] == CHAR_CLOSE_BRACE);    // byte 15

    // -----------------------------------------------------------------
    // Address range. Checked on the whole 24-bit field before narrowing.
    // -----------------------------------------------------------------
    assign rw_addr_ok  = (rw_addr[23:6] == 18'd0) &&   // fits the 6-bit port
                         (rw_addr[1:0]  ==  2'd0);     // word aligned

    assign rw_valid    = rw_frame_ok &&  rw_addr_ok;
    assign rw_addr_err = rw_frame_ok && !rw_addr_ok;

    assign rw_rgf_addr = rw_addr[5:0];

endmodule : rx_reg_write_parser
