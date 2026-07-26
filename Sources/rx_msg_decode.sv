// rx_msg_decode.sv
// ----------------
// Stage 2A: purely combinational frame-length and message-kind decoder.
//
// This is the SINGLE SOURCE OF OPCODE TRUTH for the receive path. It
// looks at the live (partially filled) frame buffer plus the live byte
// count and answers one question: how many bytes long is the message
// currently being received? rx_mac consumes expected_len and counts to
// it; it holds no protocol knowledge of its own.
//
// -----------------------------------------------------------------------
// BYTE-COUNT TIMING CONTRACT
// -----------------------------------------------------------------------
//   byte_cnt   = number of bytes ALREADY COMMITTED to frame_buf.
//                NOT the next storage index. After byte index 5 has been
//                stored, byte_cnt == 6.
//   frame_buf  = valid in [127 : 128 - 8*byte_cnt]; the remainder holds
//                stale data from the previous frame (see gating below).
//   Byte n     = frame_buf[127 - 8*n -: 8]; byte 0 is the MSB.
//
// rx_mac evaluates this module's output in MAC_CHK_DONE, which is
// entered on the edge that commits the newest byte. So at the decision
// point frame_buf ALREADY INCLUDES the newest accepted byte and byte_cnt
// already reflects it. That is why this module needs neither rx_byte nor
// byte_valid: a live tap would be a second view of the same data on a
// different timing basis, and could not be observed any earlier.
//
// Worked timing for the decisive cases:
//   Register Read      decisive byte index 5 -> byte_cnt == 6 at
//                      MAC_CHK_DONE, expected_len = 6, 6 == 6, done.
//   Single Pixel Write decisive byte index 6 -> byte_cnt == 7 at
//                      MAC_CHK_DONE, expected_len = 11, keep collecting.
//   legacy vs          decisive byte index 11. Both forms are 16 bytes,
//   Single Pixel Read  so this split affects CLASSIFICATION ONLY and
//                      never expected_len. It resolves at byte_cnt == 12,
//                      well before the frame completes at 16, so
//                      msg_kind_prov is settled by the time rx_mac
//                      latches it in MAC_DONE.
//
// -----------------------------------------------------------------------
// STALE-BUFFER GATING -- correctness critical
// -----------------------------------------------------------------------
// rx_mac's msg_buf is not cleared byte-by-byte, so positions at or above
// byte_cnt still hold the PREVIOUS message. If the byte-6 rule were
// evaluated before byte 6 had actually been stored, a Register Write
// arriving straight after a Single Pixel Write would see the leftover
// 'P' and be truncated to 11 bytes -- an intermittent failure that
// depends entirely on what preceded the message.
//
// Every narrowing rule below is therefore gated on byte_cnt being large
// enough for the byte it inspects to be real. rx_mac additionally clears
// msg_buf in MAC_DONE as defence in depth, so stale bytes read as 0x00,
// which matches none of '}', 'P', 'V', 'C' or 'H'.
//
// -----------------------------------------------------------------------
// PROVISIONAL OUTPUT
// -----------------------------------------------------------------------
// msg_kind_prov is PROVISIONAL by construction: it changes as bytes
// arrive and is only meaningful once the frame is complete. Nothing
// outside rx_mac may consume it. rx_mac latches it into msg_kind_q in
// MAC_DONE, and the parsers and classifier act only on that latched
// value together with msg_data / msg_valid.

`timescale 1ns/1ps

module rx_msg_decode
    import msg_pkg::*;
    import rx_msg_pkg::*;
(
    // Live frame state from rx_mac (see timing contract above)
    input  logic [127:0]           frame_buf,
    input  logic [BYTE_CNT_W-1:0]  byte_cnt,

    // Burst opcode bypass. Tied 1'b0 for the whole of Stage 2A.
    // When high, the frame is a burst data message: 16 bytes, and byte 1
    // is a raw pixel value rather than an opcode, so opcode inspection
    // must be suppressed entirely.
    input  logic                   bypass_active,

    // Decoded frame length, in bytes. Defaults to MSG_BYTES_MAX and is
    // only narrowed once the deciding byte is provably present.
    output logic [BYTE_CNT_W-1:0]  expected_len,

    // Provisional classification -- see note above.
    output msg_kind_t              msg_kind_prov,

    // High once enough bytes exist to classify. No design consumer in
    // Stage 2A; exists so testbenches can assert it is high whenever
    // rx_mac reaches MAC_DONE.
    output logic                   kind_known
);

    // -----------------------------------------------------------------
    // Byte extraction. Only the framing-relevant positions are needed.
    // -----------------------------------------------------------------
    logic [7:0] b0, b1, b5, b6, b11;

    assign b0  = frame_buf[127:120];  // '{'
    assign b1  = frame_buf[119:112];  // group A opcode
    assign b5  = frame_buf[ 87: 80];  // ',' or '}'
    assign b6  = frame_buf[ 79: 72];  // group B opcode
    assign b11 = frame_buf[ 39: 32];  // group C opcode

    // -----------------------------------------------------------------
    // Presence gates. byte_cnt is a COUNT, so byte index n is committed
    // once byte_cnt >= n+1.
    // -----------------------------------------------------------------
    logic have_b1, have_b5, have_b6, have_b11;

    assign have_b1  = (byte_cnt >= BYTE_CNT_W'(2));   // byte index 1 stored
    assign have_b5  = (byte_cnt >= BYTE_CNT_W'(6));   // byte index 5 stored
    assign have_b6  = (byte_cnt >= BYTE_CNT_W'(7));   // byte index 6 stored
    assign have_b11 = (byte_cnt >= BYTE_CNT_W'(12));  // byte index 11 stored

    // Every real frame opens with '{', including burst data messages.
    logic frame_start_ok;
    assign frame_start_ok = b0 == CHAR_OPEN_BRACE;

    // -----------------------------------------------------------------
    // Decode
    //
    // kind_known means "classification has SETTLED" -- including settling
    // definitively on MSG_UNKNOWN. It is low only while a decision is
    // genuinely still pending on a byte that has not yet arrived.
    // -----------------------------------------------------------------
    always_comb begin : decode
        // Defaults: assume the longest frame until proven otherwise.
        // Defaulting long is the safe direction -- a too-short guess
        // truncates a valid message, a too-long guess simply keeps
        // collecting until the real rule fires.
        expected_len  = BYTE_CNT_W'(MSG_BYTES_MAX);
        msg_kind_prov = MSG_UNKNOWN;
        kind_known    = 1'b0;

        if (bypass_active) begin
            // Burst data: fixed 16 bytes, no opcode inspection at all.
            expected_len  = BYTE_CNT_W'(MSG_BYTES_MAX);
            msg_kind_prov = MSG_BURST_DATA;
            kind_known    = 1'b1;
        end
        else if (have_b1) begin
            if (!frame_start_ok) begin
                // No opening brace: settled, and settled as garbage.
                kind_known = 1'b1;
            end
            else begin
                unique case (b1)

                    // -------------------------------------------------
                    // 'R' -> Register Read (6)
                    //      | Image Burst Read (16)
                    //      | Single Pixel Read (16)
                    //      | legacy {Rnnn,Cnnn,Vnnn} (16)
                    //
                    // Byte 5 splits off the 6-byte form. Byte 6 splits
                    // off Image Burst Read. Byte 11 then separates
                    // legacy from Single Pixel Read -- both are 16 bytes,
                    // so this is a classification split only and does not
                    // change expected_len.
                    // -------------------------------------------------
                    CHAR_R: begin
                        if (have_b5 && (b5 == CHAR_CLOSE_BRACE)) begin
                            expected_len  = BYTE_CNT_W'(MSG_BYTES_REG_READ);
                            msg_kind_prov = MSG_REG_READ;
                            kind_known    = 1'b1;
                        end
                        else if (have_b6 && (b5 == CHAR_COMMA)) begin
                            expected_len = BYTE_CNT_W'(MSG_BYTES_MAX);

                            if (b6 == CHAR_H) begin
                                msg_kind_prov = MSG_BURST_READ;
                                kind_known    = 1'b1;
                            end
                            else if (b6 == CHAR_C) begin
                                // Still ambiguous until byte 11 lands.
                                // Length is already known to be 16, so
                                // leaving the kind pending costs nothing.
                                if (have_b11) begin
                                    kind_known = 1'b1;
                                    if (b11 == CHAR_V)
                                        msg_kind_prov = MSG_LEGACY_RGF;
                                    else if (b11 == CHAR_P)
                                        msg_kind_prov = MSG_PIX_READ;
                                    else
                                        msg_kind_prov = MSG_UNKNOWN;
                                end
                            end
                            else begin
                                msg_kind_prov = MSG_UNKNOWN;
                                kind_known    = 1'b1;
                            end
                        end
                    end

                    // -------------------------------------------------
                    // 'W' -> Single Pixel Write (11) | Register Write (16)
                    // Disambiguated at byte 6.
                    // -------------------------------------------------
                    CHAR_W: begin
                        if (have_b6) begin
                            kind_known = 1'b1;
                            if (b6 == CHAR_P) begin
                                expected_len  = BYTE_CNT_W'(MSG_BYTES_PIX_WRITE);
                                msg_kind_prov = MSG_PIX_WRITE;
                            end
                            else if (b6 == CHAR_V) begin
                                expected_len  = BYTE_CNT_W'(MSG_BYTES_MAX);
                                msg_kind_prov = MSG_REG_WRITE;
                            end
                            else begin
                                expected_len  = BYTE_CNT_W'(MSG_BYTES_MAX);
                                msg_kind_prov = MSG_UNKNOWN;
                            end
                        end
                    end

                    // -------------------------------------------------
                    // 'I' -> Image Burst Write header, always 16 bytes.
                    // -------------------------------------------------
                    CHAR_I: begin
                        expected_len  = BYTE_CNT_W'(MSG_BYTES_MAX);
                        msg_kind_prov = MSG_BURST_HDR;
                        kind_known    = 1'b1;
                    end

                    // Unrecognised opcode: settled as garbage, 16 bytes.
                    default: kind_known = 1'b1;
                endcase
            end
        end
    end : decode

endmodule : rx_msg_decode
