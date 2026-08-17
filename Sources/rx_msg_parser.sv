// rx_msg_parser.sv
// ----------------
// Identifies a completed frame and extracts its payload. Pure combinational,
// one mux, no state.
//
// =======================================================================
// DIVISION OF LABOUR WITH rx_mac
// =======================================================================
//   byte  0        '{'              rx_mac
//   byte  1        opcode           THIS MODULE
//   bytes 2..4     payload          THIS MODULE (extract)
//   byte  5        ',' or '}'       rx_mac
//   byte  6        opcode           THIS MODULE
//   bytes 7..9     payload          THIS MODULE (extract)
//   byte 10        ',' or '}'       rx_mac
//   byte 11        opcode           THIS MODULE
//   bytes 12..14   payload          THIS MODULE (extract)
//   byte 15        '}'              rx_mac
//
// Neither module inspects the other's bytes. rx_mac finds the frame
// boundaries; this module says what the frame IS.
//
// =======================================================================
// LENGTH IS AN INPUT, NOT A DECISION
// =======================================================================
// This module used to derive expected_len from the opcodes and feed it back
// to rx_mac, which counted up to it. rx_mac now finds the end of the frame
// itself from the delimiter positions, so byte_cnt ARRIVES here already
// correct and the decode is a lookup rather than a derivation:
//
//     byte_cnt  groups  opcodes at 1, 6, 11   message
//     --------  ------  -------------------   -------------------------
//         6       1     R                     Register Read
//        11       2     W  P                  Single Pixel Write
//        16       3     W  V  V               Register Write
//        16       3     R  C  P               Single Pixel Read
//        16       3     R  C  V               legacy {Rnnn,Cnnn,Vnnn}
//        16       3     I  H  W               Image Burst Write header
//        16       3     R  H  W               Image Burst Read
//        16       -     (bypass_active)       Image Burst Write data
//     anything else, or any other opcode set   MSG_UNKNOWN
//
// Two things fell out of that change:
//
// THE PRESENCE GATES ARE GONE. have_b1 / have_b5 / have_b6 / have_b11 existed
// because this module had to decode a PARTIALLY FILLED buffer without
// tripping over leftovers from the previous frame. It only ever decodes
// complete frames now, and byte_cnt says exactly how many bytes are real: a
// 6-byte frame never consults bytes 6 or 11, so their contents cannot matter.
// The stale-buffer hazard is designed out rather than guarded against.
//
// LENGTH AND OPCODE CAN NO LONGER DISAGREE. Previously a 'W' at byte 1
// IMPLIED a length, and a frame could arrive whose actual length differed
// from what its opcode implied -- the case that used to fall between
// rx_msg_decode and the parsers and vanish silently. Length is now
// established first, independently, and the opcodes are checked AGAINST it.
// A '{W...}' that is only 6 bytes long is simply MSG_UNKNOWN.
//
// =======================================================================
// OUTPUTS
// =======================================================================
//   msg_kind      what this message is, or MSG_UNKNOWN
//   field0/1/2    the three payload slots, raw and unnarrowed
//   burst_pixels  burst data only -- see below
//
// The fields come out RAW. This module checks the opcode alphabet and nothing
// else: whether an address fits the RGF window, whether a coordinate is inside
// the image, whether a burst extent overruns -- those are questions about what
// a field MEANS, and they belong to rx_classifier, which knows which message
// it is looking at.
//
// NO frame_ok OUTPUT. Framing is rx_mac's guarantee: it only raises
// frame_done for a frame that opened on '{', carried a legal delimiter at
// every delimiter position, and closed on '}'. Re-checking here would
// duplicate a fact that is already established, which is what let a
// tail-malformed frame disappear silently in the old split design. A frame
// whose SHAPE is wrong never reaches this module at all -- rx_mac raises
// frame_err instead.
//
// BURST DATA HAS ITS OWN OUTPUT, DELIBERATELY.
// field0/1/2 are bytes 2-4, 7-9 and 12-14: nine payload bytes. A burst data
// frame carries TWELVE, because in bypass the opcode slots at 1, 6 and 11 are
// payload rather than opcodes, and its pixels straddle the delimiters:
//
//   {<R0,G0,B0,R1>, <G1,B1,R2,G2>, <B2,R3,G3,B3>}
//
// R1 is byte 4 and G1 is byte 6 -- one pixel spans the comma at byte 5. That
// does not fit the three-field model and is not forced into it.
//
// =======================================================================
// VALIDITY WINDOW
// =======================================================================
// Outputs are combinational on frame_buf and byte_cnt, both of which change
// as a frame fills. They are ONLY meaningful while rx_mac asserts frame_done.
// Mid-fill, byte_cnt passes through 6 and 11 and this module may momentarily
// report a kind for a frame that is not finished; nothing samples it then.

`timescale 1ns/1ps

module rx_msg_parser
    import msg_format_pkg::*;
(
    // Completed frame from rx_mac. Valid while frame_done is high.
    input  logic [FRAME_W-1:0]     frame_buf,
    input  logic [BYTE_CNT_W-1:0]  byte_cnt,

    // Burst opcode bypass, from rx_burst_ctrl. While high the frame is pixel
    // data: bytes 1, 6 and 11 are payload, not opcodes.
    input  logic                   bypass_active,

    // What the message is. MSG_UNKNOWN means "not a legal message -- drop it".
    output msg_kind_t              msg_kind,

    // The three payload slots, raw. Meaningless for a burst data frame.
    output logic [PAYLOAD_W-1:0]   field0,
    output logic [PAYLOAD_W-1:0]   field1,
    output logic [PAYLOAD_W-1:0]   field2,

    // Burst data only.
    output logic [BURST_PIX_PER_MSG-1:0][BURST_PIX_W-1:0] burst_pixels
);

    // -----------------------------------------------------------------
    // The three opcode positions. Bytes 0, 5, 10 and 15 are rx_mac's and
    // are deliberately not read here.
    // -----------------------------------------------------------------
    logic [7:0] b1, b6, b11;

    assign b1  = frame_buf[byte_lsb(op_byte_idx(0)) +: 8];   // byte 1
    assign b6  = frame_buf[byte_lsb(op_byte_idx(1)) +: 8];   // byte 6
    assign b11 = frame_buf[byte_lsb(op_byte_idx(2)) +: 8];   // byte 11

    // =================================================================
    // THE MUX
    // =================================================================
    always_comb begin : decode
        msg_kind = MSG_UNKNOWN;

        if (bypass_active) begin
            // Burst data. No opcodes; the frame is 16 bytes of pixel payload.
            if (byte_cnt == BYTE_CNT_W'(MSG_BYTES_3GRP))
                msg_kind = MSG_BURST_DATA;
        end
        else begin
            case (byte_cnt)

                // ---- one group : {R<A2,A1,A0>} ----
                BYTE_CNT_W'(MSG_BYTES_1GRP):
                    if (b1 == CHAR_R) msg_kind = MSG_REG_READ;

                // ---- two groups : {W<A>, P<R,G,B>} ----
                BYTE_CNT_W'(MSG_BYTES_2GRP):
                    if (b1 == CHAR_W && b6 == CHAR_P) msg_kind = MSG_PIX_WRITE;

                // ---- three groups ----
                BYTE_CNT_W'(MSG_BYTES_3GRP): begin
                    if      (b1 == CHAR_W && b6 == CHAR_V && b11 == CHAR_V)
                        msg_kind = MSG_REG_WRITE;
                    else if (b1 == CHAR_R && b6 == CHAR_C && b11 == CHAR_P)
                        msg_kind = MSG_PIX_READ;
                    else if (b1 == CHAR_I && b6 == CHAR_H && b11 == CHAR_W)
                        msg_kind = MSG_BURST_HDR;
                    else if (b1 == CHAR_R && b6 == CHAR_H && b11 == CHAR_W)
                        msg_kind = MSG_BURST_READ;
                end

                default: msg_kind = MSG_UNKNOWN;
            endcase
        end
    end : decode

    // =================================================================
    // FIELD EXTRACTION
    //
    // Fixed positions -- the payload slots never move. Slots beyond the
    // frame's length hold stale bytes and are ignored by rx_classifier,
    // which reads only the fields its message kind defines.
    // =================================================================
    assign field0 = frame_buf[byte_lsb(payload_byte_idx(0) + PAYLOAD_BYTES - 1)
                              +: PAYLOAD_W];                 // bytes  2, 3, 4
    assign field1 = frame_buf[byte_lsb(payload_byte_idx(1) + PAYLOAD_BYTES - 1)
                              +: PAYLOAD_W];                 // bytes  7, 8, 9
    assign field2 = frame_buf[byte_lsb(payload_byte_idx(2) + PAYLOAD_BYTES - 1)
                              +: PAYLOAD_W];                 // bytes 12,13,14

    // Burst data. Twelve payload bytes, four pixels, straddling the
    // delimiters at bytes 5 and 10.
    //
    //   pixel 0  bytes  1, 2, 3
    //   pixel 1  bytes  4, 6, 7      <- spans the comma at byte 5
    //   pixel 2  bytes  8, 9, 11     <- spans the comma at byte 10
    //   pixel 3  bytes 12,13,14
    always_comb begin : extract_burst
        burst_pixels[0] = frame_buf[byte_lsb(3)  +: 24];
        burst_pixels[1] = {frame_buf[byte_lsb(4) +: 8],
                           frame_buf[byte_lsb(7) +: 16]};
        burst_pixels[2] = {frame_buf[byte_lsb(9) +: 16],
                           frame_buf[byte_lsb(11) +: 8]};
        burst_pixels[3] = frame_buf[byte_lsb(14) +: 24];
    end : extract_burst

endmodule : rx_msg_parser
