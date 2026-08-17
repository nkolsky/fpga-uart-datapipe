// tx_reply_ctrl.sv
// ----------------
// Builds and schedules host replies on the 130 MHz transmit domain.
//
// TWO reply kinds share one pending slot:
//
//   Register Read      6 bytes, composed here from a 32-bit value
//
//     byte 0  0x7B  '{'
//     byte 1  pc_rdata[31:24]   V3      big-endian, MSB first
//     byte 2  pc_rdata[23:16]   V2
//     byte 3  pc_rdata[15: 8]   V1
//     byte 4  pc_rdata[ 7: 0]   V0
//     byte 5  0x7D  '}'
//
//   Single Pixel Read  16 bytes, composed OUTSIDE this module by
//                      msg_composer and presented ready-made on pix_msg
//
// tx_mac packs byte 0 in msg_data[7:0], so frames are assembled LSB-first
// -- the same convention msg_composer already uses, which is exactly why
// its output can be handed straight through.
//
// -----------------------------------------------------------------------
// WHY THE LENGTH IS NOW LATCHED
// -----------------------------------------------------------------------
// reply_len used to be the constant 5'd6. With two kinds sharing the slot
// it must travel with the payload: a 16-byte pixel reply sent with a
// length of 6 would truncate to '{' plus three bytes of row data, and a
// 6-byte register reply sent with a length of 16 would append ten bytes of
// stale frame. Both are silent corruptions on the wire, so msg_q and len_q
// are written by the same assignment and can never disagree.
//
// -----------------------------------------------------------------------
// ARBITRATION -- why a reply waits
// -----------------------------------------------------------------------
// A reply is held until BOTH tx_seq_busy and mac_busy are low.
//
// An image transfer is 65536 fixed-size packets and the host reads it as
// one uninterrupted stream, counting bytes. A reply inserted between two
// pixel messages would desynchronise every packet after it -- the capture
// would fail with a framing error thousands of packets later, far from the
// cause. Deferring costs nothing: at UART rates a full image takes ~1.5 s
// and the host is not waiting on a register value during a capture.
//
// -----------------------------------------------------------------------
// TWO PRODUCERS, TWO DIFFERENT BACKPRESSURE CONTRACTS
// -----------------------------------------------------------------------
// The two inputs are deliberately NOT symmetric, because their sources are
// not symmetric.
//
//   rd_valid  is a one-cycle STROBE from cdc_cmd_sync. There is nothing at
//             the far end that can hold it. If it arrives while the slot is
//             occupied the only options are overwrite or drop, and dropping
//             the NEWER one preserves the answer the host is actually
//             waiting for. reply_overrun records that it happened.
//
//   pix_valid is a held LEVEL from pixel_rd_ctrl, via a level synchroniser.
//             It has real backpressure, so there is no need to drop
//             anything: the pixel reply simply WAITS until the slot frees,
//             however long that takes, and is then taken normally. This is
//             not an overrun and is not flagged as one.
//
// A pixel reply is accepted only when the slot is free AND no register
// reply is arriving in the same cycle. Giving rd_valid unconditional
// priority is what keeps the collision case out of the overrun path
// entirely: the strobe that cannot wait is served, and the level that can
// wait, waits.
//
// pix_accept is a single cycle, returned to the 100 MHz domain through a
// pulse synchroniser. ack_hold then suppresses any further acceptance
// until pix_valid has actually fallen, so one assertion of pix_valid
// produces exactly one latched reply no matter how long it stays high.
//
// -----------------------------------------------------------------------
// NO PULSE IS LOST
// -----------------------------------------------------------------------
// pending is a level, set by the incoming reply and cleared only when the
// MAC has actually taken the message (mac_busy observed high while this
// module is driving it). Backpressure of any duration simply extends the
// wait; nothing is dropped and nothing is re-sent.
//
// -----------------------------------------------------------------------
// A SECOND REGISTER REQUEST WHILE ONE IS PENDING
// -----------------------------------------------------------------------
// The second reply is REJECTED and the first is preserved, with
// reply_overrun raised as a sticky diagnostic. Overwriting would answer the
// newer request with a value the host never asked for at that point in the
// exchange, and silently discard an answer it is still waiting for. The
// protocol is strictly request/response, so a well-behaved host cannot
// create this condition -- it exists to make a misbehaving one visible.

`timescale 1ns/1ps

module tx_reply_ctrl
    import msg_pkg::*;
(
    input  logic         clk,            // pll_clk_out, 130 MHz
    input  logic         rst_n,          // sync_pll_rst_n

    // ---- Register Read value, already in this clock domain ------------
    input  logic         rd_valid,       // one-cycle strobe from cdc_cmd_sync
    input  logic [31:0]  rd_data,

    // ---- Single Pixel Read reply, held handshake ----------------------
    // pix_valid is pixel_rd_ctrl's rpy_valid recovered onto this clock by
    // a level synchroniser. pix_msg is the msg_composer output driven from
    // the 100 MHz payload registers; it is stable for many cycles before
    // pix_valid can be observed here and stays stable until pix_accept is
    // seen at the far side, so it needs no synchroniser of its own.
    input  logic         pix_valid,
    input  logic [127:0] pix_msg,
    output logic         pix_accept,     // one cycle: payload has been latched

    // ---- Image Burst Read reply, held handshake -----------------------
    // A THIRD producer, with the same contract as the pixel reply: a held
    // level with real backpressure, so it waits rather than being dropped.
    // The frame is already composed (burst_msg_composer) and is 16 bytes,
    // so no new length is introduced -- len_q simply carries 16 again.
    //
    // Unlike the other two this one is a STREAM: one burst emits
    // ceil(H*W/4) messages, up to 16,384 for a full frame. Each is handed
    // over individually through this same single slot, which is ample
    // because the UART needs ~19.7 us per message while the handshake
    // costs tens of nanoseconds.
    input  logic         brd_valid,
    input  logic [127:0] brd_msg,
    output logic         brd_accept,

    // ---- transmit path status -----------------------------------------
    input  logic         tx_seq_busy,    // image transfer in progress
    input  logic         mac_busy,       // tx_mac occupied
    input  logic         cts,            // active-low: 0 = clear to send

    // ---- request to drive tx_mac (muxed in chip_top) ------------------
    output logic         reply_req,      // hold high until accepted
    output logic [127:0] reply_msg,
    output logic [4:0]   reply_len,

    // ---- status --------------------------------------------------------
    output logic         reply_pending,  // a reply is queued or in flight
    output logic         reply_sent,     // one cycle, reply handed to the MAC
    output logic         reply_overrun   // sticky: a register reply was dropped
);

    localparam logic [4:0] REG_REPLY_BYTES = 5'd6;
    localparam logic [4:0] PIX_REPLY_BYTES = 5'd16;

    logic [127:0] msg_q;
    logic [4:0]   len_q;
    logic         pending;
    logic         driving;      // reply_req asserted, waiting for the MAC
    logic         ack_hold;     // pix_valid already serviced, awaiting its fall
    logic         brd_hold;     // same, for the burst stream

    assign reply_pending = pending;
    assign reply_msg     = msg_q;
    assign reply_len     = len_q;

    // Register Read frame, assembled LSB-first: byte 0 occupies bits [7:0].
    // Unchanged from the single-kind version -- the bytes and their order
    // are identical, they are simply latched now instead of being driven
    // continuously from value_q.
    logic [127:0] reg_frame;
    assign reg_frame = {80'd0,
                        CHAR_CLOSE_BRACE,     // byte 5  [47:40]
                        rd_data[ 7: 0],       // byte 4  [39:32]  V0
                        rd_data[15: 8],       // byte 3  [31:24]  V1
                        rd_data[23:16],       // byte 2  [23:16]  V2
                        rd_data[31:24],       // byte 1  [15: 8]  V3
                        CHAR_OPEN_BRACE};     // byte 0  [ 7: 0]

    // A pixel reply may be taken when it is genuinely on offer, has not
    // already been taken, the slot is free, and no register strobe is
    // competing for that slot this cycle.
    logic take_pix;
    assign take_pix = pix_valid && !ack_hold && !pending && !rd_valid;

    // Burst messages sit BELOW the single-pixel reply in priority. Both are
    // held levels so the loser simply waits, and in practice they cannot
    // contend: a single-pixel read cannot obtain memory while a burst owns
    // it, so its reply cannot exist mid-burst. The ordering is fixed anyway
    // so the outcome is deterministic rather than incidental.
    logic take_brd;
    assign take_brd = brd_valid && !brd_hold && !pending && !rd_valid && !take_pix;

    // Drive the MAC only when the transmit path is completely idle AND the
    // host is accepting bytes.
    //
    // cts is the host's RTS, active low. Adding it here rather than further
    // down the transmit path is deliberate: reply_req is a combinational
    // LEVEL held while `pending` is set, so a deasserted RTS merely extends
    // the wait -- pending stays set, msg_q and len_q stay stable, and the
    // frame is neither dropped nor re-sent. Gating tx_phy instead would have
    // met a one-cycle phy_valid pulse with no way to hold it, and declining
    // that pulse loses the byte outright.
    //
    // Backpressure reaches the memory side through the existing handshake:
    // pending stays high, so take_pix and take_brd both fail on !pending,
    // so pix_accept/brd_accept never fire and their producers hold. A burst
    // read stalls its walker rather than reading pixels onto the floor.
    assign reply_req = pending && !tx_seq_busy && !mac_busy && !cts;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            msg_q         <= '0;
            len_q         <= REG_REPLY_BYTES;
            pending       <= 1'b0;
            driving       <= 1'b0;
            ack_hold      <= 1'b0;
            reply_sent    <= 1'b0;
            reply_overrun <= 1'b0;
            pix_accept    <= 1'b0;
            brd_accept    <= 1'b0;
            brd_hold      <= 1'b0;
        end
        else begin
            reply_sent <= 1'b0;
            pix_accept <= 1'b0;
            brd_accept <= 1'b0;

            // ---- re-arm the handshakes once their levels retract ------
            if (!pix_valid)
                ack_hold <= 1'b0;
            if (!brd_valid)
                brd_hold <= 1'b0;

            // ---- accept a Register Read value ------------------------
            // Unconditional priority over the pixel path: this strobe
            // cannot be held at its source, the pixel level can.
            if (rd_valid) begin
                if (pending) begin
                    // Keep the first answer; flag the collision.
                    reply_overrun <= 1'b1;
                end
                else begin
                    msg_q   <= reg_frame;
                    len_q   <= REG_REPLY_BYTES;
                    pending <= 1'b1;
                end
            end
            // ---- otherwise accept a Single Pixel Read reply ----------
            else if (take_pix) begin
                msg_q      <= pix_msg;
                len_q      <= PIX_REPLY_BYTES;
                pending    <= 1'b1;
                pix_accept <= 1'b1;
                ack_hold   <= 1'b1;
            end
            // ---- otherwise accept an Image Burst Read message ---------
            else if (take_brd) begin
                msg_q      <= brd_msg;
                len_q      <= PIX_REPLY_BYTES;   // also 16 bytes
                pending    <= 1'b1;
                brd_accept <= 1'b1;
                brd_hold   <= 1'b1;
            end

            // ---- hand the frame to the MAC ---------------------------
            // reply_req is combinational, so it deasserts the moment
            // mac_busy rises. Latching `driving` on the request cycle and
            // clearing `pending` when the MAC actually goes busy is what
            // makes the handover survive any amount of backpressure.
            if (reply_req)
                driving <= 1'b1;

            if (driving && mac_busy) begin
                driving    <= 1'b0;
                pending    <= 1'b0;
                reply_sent <= 1'b1;
            end
        end
    end

`ifndef SYNTHESIS
    // A reply is never offered while the host is holding the link off.
    // This restates the reply_req assign, so it is a regression guard: it
    // fires if a later edit drops the cts term rather than catching a
    // dynamic condition.
    a_reply_respects_cts: assert property (
        @(posedge clk) disable iff (!rst_n)
        reply_req |-> !cts
    ) else $error("%m: reply offered while the host deasserted RTS");

    // There is deliberately NO assertion that pending survives while cts is
    // high. a_no_loss below already covers it -- pending may fall only on a
    // genuine handoff, whatever cts is doing -- and the naive form
    // ((pending && cts) |=> pending) would FALSE-FIRE on the legal case
    // where cts rises in the same cycle mac_busy does, after the MAC has
    // already captured the frame. That message is in flight and completes
    // correctly; it is the ~16-byte tail after a hold-off, not a loss.

    // A reply must never be offered while the image path is active.
    a_no_overlap: assert property (
        @(posedge clk) disable iff (!rst_n)
        reply_req |-> (!tx_seq_busy && !mac_busy)
    ) else $error("%m: reply offered during image transmission");

    // Once queued, a reply is never silently dropped. The condition is the
    // ACTUAL handoff -- (driving && mac_busy) -- not reply_sent, which is a
    // nonblocking output updated in the very cycle pending is legitimately
    // cleared and so cannot be used to qualify its own clearing.
    a_no_loss: assert property (
        @(posedge clk) disable iff (!rst_n)
        (pending && !(driving && mac_busy)) |=> pending
    ) else $error("%m: pending reply disappeared without being sent");

    // ...and pending may fall ONLY on a genuine handoff.
    a_fall_only_on_handoff: assert property (
        @(posedge clk) disable iff (!rst_n)
        $fell(pending) |-> reply_sent
    ) else $error("%m: pending cleared without a MAC handoff");

    // The held frame and its length are stable for the whole wait, and
    // stable TOGETHER -- a length that outlived its payload would put a
    // truncated or over-long frame on the wire.
    a_stable: assert property (
        @(posedge clk) disable iff (!rst_n)
        (pending && !(driving && mac_busy)) |=> ($stable(msg_q) &&
                                                 $stable(len_q))
    ) else $error("%m: pending reply changed while queued");

    // Only the two legal lengths ever reach the MAC.
    a_len_legal: assert property (
        @(posedge clk) disable iff (!rst_n)
        reply_req |-> (reply_len == REG_REPLY_BYTES ||
                       reply_len == PIX_REPLY_BYTES)
    ) else $error("%m: illegal reply length offered to the MAC");

    // The pixel handshake is one-for-one: an accept only ever fires while
    // the level is up, and never twice for the same assertion.
    a_accept_qualified: assert property (
        @(posedge clk) disable iff (!rst_n)
        pix_accept |-> pix_valid
    ) else $error("%m: pix_accept asserted without pix_valid");

    a_accept_once: assert property (
        @(posedge clk) disable iff (!rst_n)
        pix_accept |=> !pix_accept
    ) else $error("%m: pix_accept asserted twice in succession");

    // A pixel reply is never taken into an occupied slot.
    a_brd_accept_qualified: assert property (
        @(posedge clk) disable iff (!rst_n)
        brd_accept |-> brd_valid
    ) else $error("%m: brd_accept asserted without brd_valid");

    a_brd_accept_once: assert property (
        @(posedge clk) disable iff (!rst_n)
        brd_accept |=> !brd_accept
    ) else $error("%m: brd_accept asserted twice in succession");

    a_one_producer: assert property (
        @(posedge clk) disable iff (!rst_n)
        !(pix_accept && brd_accept)
    ) else $error("%m: two reply producers accepted in one cycle");

    a_pix_no_overwrite: assert property (
        @(posedge clk) disable iff (!rst_n)
        pix_accept |-> !$past(pending)
    ) else $error("%m: pixel reply overwrote a pending reply");
`endif

endmodule : tx_reply_ctrl
