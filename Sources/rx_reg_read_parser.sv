// rx_reg_read_parser.sv
// ---------------------
// Combinational parser for the Register Read request.
//
//   {R<A2,A1,A0>}                                        6 bytes
//
//   byte 0  '{'   msg_in[127:120]
//   byte 1  'R'   msg_in[119:112]
//   byte 2  A2    msg_in[111:104]
//   byte 3  A1    msg_in[103: 96]
//   byte 4  A0    msg_in[ 95: 88]
//   byte 5  '}'   msg_in[ 87: 80]
//
// so the 24-bit address is the contiguous slice msg_in[111:88], big-endian --
// the same position and orientation rx_pixel_wr_parser uses for its address.
//
// -----------------------------------------------------------------------
// ADDRESS VALIDATION
// -----------------------------------------------------------------------
// The A field is a BYTE address into the register file, matching the map
// the design already uses: IMG_TX_MON = 0x04, IMG_CTRL = 0x08,
// CLK_CTRL = 0x10. The RGF port is 6 bits wide, so a request is accepted
// only when it genuinely fits and is word aligned:
//
//     addr[23:6] == 0        in range
//     addr[1:0]  == 2'b00    word aligned
//
// An out-of-range address is REJECTED, never truncated. Aliasing
// 0x000104 down to 0x04 would silently read -- and read-to-clear --
// IMG_TX_MON in response to a request that named a different register,
// which is worse than refusing it.
//
// A rejected request raises rr_addr_err so rx_classifier can report it
// through the existing classifier_error diagnostic, and emits no RGF
// command and no reply.

`timescale 1ns/1ps

module rx_reg_read_parser
    import msg_pkg::*;
(
    input  logic [127:0] msg_in,

    output logic         rr_frame_ok,   // the three fixed bytes are correct
    output logic         rr_addr_ok,    // in range and word aligned
    output logic         rr_valid,      // both of the above
    output logic         rr_addr_err,   // framing good, address unusable
    output logic [23:0]  rr_addr,       // raw 24-bit byte address
    output logic [5:0]   rr_rgf_addr    // addr[5:0], valid only when rr_valid
);

    assign rr_addr = msg_in[111:88];          // bytes 2, 3, 4

    // Only three fixed bytes exist in this message: '{', 'R' and '}'.
    assign rr_frame_ok = (msg_in[127:120] == CHAR_OPEN_BRACE)  &&
                         (msg_in[119:112] == CHAR_R)           &&
                         (msg_in[ 87: 80] == CHAR_CLOSE_BRACE);

    assign rr_addr_ok  = (rr_addr[23:6] == 18'd0) &&    // fits the 6-bit port
                         (rr_addr[1:0]  ==  2'd0);      // word aligned

    assign rr_valid    = rr_frame_ok && rr_addr_ok;
    assign rr_addr_err = rr_frame_ok && !rr_addr_ok;

    assign rr_rgf_addr = rr_addr[5:0];

endmodule : rx_reg_read_parser
