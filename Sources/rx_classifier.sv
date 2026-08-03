// rx_classifier.sv
// ----------------
// Registered latch of parser outputs.
//
// Lab 9 requirement: "Classifier: Sampled -> Holds the opcode and values
// for the sequencer after a valid parsing indication"
//
// Latches parser fields on the cycle where msg_valid (from rx_mac) and
// the relevant parser's valid indication are both high.
//
// Outputs hold their values until the next valid message arrives.
// classifier_valid pulses for one cycle to notify the sequencer.
//
// -----------------------------------------------------------------------
// MESSAGE-KIND ROUTING
// -----------------------------------------------------------------------
// EIGHT parsers run in parallel on the same completed frame; msg_kind
// (rx_mac's LATCHED msg_kind_q, never the provisional decode output)
// selects which ONE is authoritative for a given frame. Six of the eight
// are gated by this module; the two burst parsers are not (see below):
//
//   rx_parser              legacy {Rnnn,Cnnn,Vnnn} -> RGF control path
//   rx_pixel_wr_parser     {W<A>, P<R,G,B>}        -> pixel write command
//   rx_reg_write_parser    {W<A>, V<..>, V<..>}    -> RGF write (32-bit)
//   rx_reg_read_parser     {R<A>}                  -> RGF read + reply
//   rx_pixel_rd_parser     {R<..>,C<..>,P<..>}     -> pixel_rd_ctrl
//   rx_burst_rd_parser     {R<A>,H<..>,W<..>}      -> burst_rd_ctrl
//   rx_burst_hdr_parser /  {I<..>,H<..>,W<..>} and -> rx_burst_ctrl
//   rx_burst_data_parser   the burst data frames      (NOT gated here --
//                          see the burst-mode section below: rx_burst_ctrl
//                          consumes those directly and this module gates
//                          ALL of its own outputs on !burst_active)
//
// THE KIND GATES ARE CORRECTNESS CRITICAL, NOT STYLISTIC:
//
//   1. classifier_valid must require MSG_LEGACY_RGF. chip_top computes
//      rgf_pc_wen = rx_classifier_valid && !rx_col_q[0]. If a Single
//      Pixel Write raised classifier_valid, the STALE row_q/col_q from
//      the previous legacy message would drive a bogus RGF write --
//      potentially into IMG_CTRL, starting a spurious image transfer.
//
//   2. classifier_error must be gated too. rx_parser asserts
//      parse_error on every {W...} frame because its framing check
//      fails, so an ungated error output would flag every pixel write
//      as a fault.
//
// -----------------------------------------------------------------------
// ERROR POLICY
// -----------------------------------------------------------------------
// classifier_error pulses ONLY for traffic that is malformed or
// unclassifiable. A structurally valid message never raises an error
// merely because there is no consumer for it.
//
//   raises an error:
//     MSG_LEGACY_RGF with parse_error      (bad legacy framing)
//     MSG_PIX_WRITE  with pw_parse_error   (bad pixel-write framing)
//     MSG_REG_WRITE  with rw_addr_err      (out-of-range register addr)
//     MSG_REG_READ   with rr_addr_err      (out-of-range register addr)
//     MSG_PIX_READ   with !pr_valid        (bad framing or coordinates)
//     MSG_BURST_READ with !br_valid        (bad framing or extent)
//     MSG_UNKNOWN                          (unclassifiable frame)
//
//   silently ignored: nothing. Every message kind the receive path can
//   classify now has a consumer.
//
// TWO REGISTER WRITE PATHS COEXIST, DELIBERATELY:
//   MSG_LEGACY_RGF  {Rnnn,Cnnn,Vnnn}   ASCII, 8-bit value, zero-extended
//   MSG_REG_WRITE   {W<A>,V<..>,V<..>} binary, full 32-bit value
// They are different msg_kinds and can never collide on one frame. The
// legacy path is retained unchanged for backwards compatibility with the
// existing PC-side tooling; the binary path is the one the project
// specification defines, and the only one that can write the upper 24 bits
// of a register.
//
// MSG_BURST_HDR and MSG_BURST_DATA are absent from both lists on purpose:
// they are consumed by rx_burst_ctrl directly off rx_mac, never through
// this module, which gates all of its outputs on !burst_active.
//
// KNOWN LIMITATION: rx_msg_decode inspects only bytes 0, 1, 5, 6 and 11.
// rx_reg_write_parser additionally checks bytes 0, 5, 10, 11 and 15, so a
// Register Write that is malformed at its tail now fails the parser -- but
// it fails with rw_addr_err LOW and rw_valid LOW, so it is dropped WITHOUT
// an error pulse. That matches how the legacy and pixel-write paths treat a
// framing failure. Only an address fault is reported, because only an
// address fault is unambiguously a well-formed request the design must
// refuse.

`timescale 1ns/1ps

module rx_classifier
    import rx_msg_pkg::*;
(
    input  logic        clk,
    input  logic        rst_n,

    // From rx_mac
    input  logic        msg_valid,    // one-cycle pulse: new message available
    input  msg_kind_t   msg_kind,     // rx_mac's LATCHED msg_kind_q

    // From rx_parser (combinational, stable on msg_valid) -- legacy path
    input  logic        parse_valid,  // message format is correct
    input  logic        parse_error,  // message format is wrong
    input  logic [9:0]  row,
    input  logic [9:0]  col,
    input  logic [23:0] pixel,

    // From rx_pixel_wr_parser (combinational, stable on msg_valid)
    input  logic        pw_parse_valid,
    input  logic        pw_parse_error,
    input  logic [23:0] pw_addr,
    input  logic [23:0] pw_pixel,

    // To sequencer -- legacy RGF control path (behaviour unchanged)
    output logic        classifier_valid,  // one-cycle pulse: new valid command latched
    output logic        classifier_error,  // one-cycle pulse: framing error received
    output logic [9:0]  row_q,             // latched row (holds until next valid msg)
    output logic [9:0]  col_q,             // latched col
    output logic [23:0] pixel_q,           // latched pixel {R,G,B}

    // Single Pixel Write command -> chip_top's command mux -> the 48-bit
    // command FIFO -> sram_wr_ctrl. Shares that FIFO with rx_burst_ctrl's
    // command output; the two are mutually exclusive by construction.
    output logic        cmd_valid,         // one-cycle pulse
    output logic [23:0] cmd_addr,          // raw 24-bit {A2,A1,A0}
    output logic [23:0] cmd_pixel,         // {R,G,B}

    // -----------------------------------------------------------------
    // STAGE 3 / M4: burst mode indication, from rx_burst_ctrl.
    //
    // OWNERSHIP: this module no longer drives bypass_active. That signal is
    // owned solely by rx_burst_ctrl, which holds the burst FSM. Two drivers
    // would be a real multi-driver conflict, so the output was REMOVED
    // rather than left tied off.
    //
    // burst_active arrives here as an INPUT, in the same 130 MHz domain,
    // and makes this module completely inert for the duration of a burst.
    // -----------------------------------------------------------------
    input  logic        burst_active,

    // -----------------------------------------------------------------
    // REGISTER READ: parsed request from rx_reg_read_parser, and the
    // resulting RGF command. rr_cmd_valid is the SECOND producer of RGF
    // commands; it is mutually exclusive with classifier_valid because a
    // frame has exactly one msg_kind_q and only one msg_valid pulse.
    // -----------------------------------------------------------------
    input  logic        rr_valid,
    input  logic        rr_addr_err,
    input  logic [5:0]  rr_rgf_addr,

    output logic        rr_cmd_valid,
    output logic [5:0]  rr_cmd_addr,

    // -----------------------------------------------------------------
    // REGISTER WRITE: parsed request from rx_reg_write_parser, and the
    // resulting RGF command. rw_cmd_valid is the THIRD producer of RGF
    // commands, alongside classifier_valid (legacy) and rr_cmd_valid
    // (Register Read). All three are mutually exclusive because a frame
    // has exactly one msg_kind_q and only one msg_valid pulse.
    //
    // Unlike the legacy path this carries a FULL 32-bit value: the legacy
    // frame decodes a single 8-bit ASCII field which chip_top
    // zero-extends, so it structurally cannot reach the upper bits of any
    // register. This one can.
    // -----------------------------------------------------------------
    input  logic        rw_valid,
    input  logic        rw_addr_err,
    input  logic [5:0]  rw_rgf_addr,
    input  logic [31:0] rw_data,

    output logic        rw_cmd_valid,
    output logic [5:0]  rw_cmd_addr,
    output logic [31:0] rw_cmd_data,

    // -----------------------------------------------------------------
    // SINGLE PIXEL READ: parsed request from rx_pixel_rd_parser.
    //
    // This is NOT an RGF producer. It leaves the module on its own
    // strobe and crosses to the 100 MHz memory domain to reach
    // pixel_rd_ctrl, so it does not join the rgf_src mux in chip_top and
    // cannot contend with the two producers that do.
    //
    // pr_valid already carries the FULL 24-bit coordinate range check
    // performed in the parser, so nothing here needs to re-examine the
    // raw fields.
    // -----------------------------------------------------------------
    input  logic        pr_valid,
    input  logic        pr_coord_err,

    input  logic [9:0]  pr_row,
    input  logic [9:0]  pr_col,

    output logic        pr_cmd_valid,
    output logic [9:0]  pr_cmd_row,
    output logic [9:0]  pr_cmd_col,

    // -----------------------------------------------------------------
    // IMAGE BURST READ: parsed request from rx_burst_rd_parser.
    //
    // Like the Single Pixel Read command this is NOT an RGF producer. It
    // leaves on its own strobe and crosses to the 100 MHz memory domain
    // to reach burst_rd_ctrl, so it never joins the rgf_src mux.
    //
    // br_valid already carries the full 24-bit range checks AND the
    // base+extent test, so nothing here re-examines the raw fields.
    // -----------------------------------------------------------------
    input  logic        br_valid,
    input  logic        br_err,

    input  logic [9:0]  br_base_row,
    input  logic [9:0]  br_base_col,
    input  logic [9:0]  br_height,
    input  logic [9:0]  br_width,

    output logic        br_cmd_valid,
    output logic [9:0]  br_cmd_base_row,
    output logic [9:0]  br_cmd_base_col,
    output logic [9:0]  br_cmd_height,
    output logic [9:0]  br_cmd_width
);

// -------------------------------------------------------------------------
// Kind-qualified hit / error terms
// -------------------------------------------------------------------------
logic legacy_hit, legacy_err;
logic pixwr_hit,  pixwr_err;
logic unknown_msg;

logic regrd_hit, regrd_err;
logic regwr_hit, regwr_err;
logic pixrd_hit, pixrd_err;
logic burstrd_hit, burstrd_err;

always_comb begin : qualify
    // `active` is the ONLY frame-acceptance condition. During a burst this
    // module emits nothing at all -- no command, no RGF strobe, no error.
    // rx_burst_ctrl owns every frame for the duration and reports its own
    // problems through err_data_invalid.
    automatic logic active = msg_valid && !burst_active;

    legacy_hit  = active && (msg_kind == MSG_LEGACY_RGF) && parse_valid;
    legacy_err  = active && (msg_kind == MSG_LEGACY_RGF) && parse_error;

    pixwr_hit   = active && (msg_kind == MSG_PIX_WRITE)  && pw_parse_valid;
    pixwr_err   = active && (msg_kind == MSG_PIX_WRITE)  && pw_parse_error;

    // Frames rx_msg_decode could not classify at all. Preserves the
    // pre-Stage-2A behaviour of pulsing an error on garbage input.
    unknown_msg = active && (msg_kind == MSG_UNKNOWN);

    // Register Read: accepted only with a usable address. A framing-good
    // request with an out-of-range or unaligned address is reported through
    // the existing classifier_error rather than silently truncated.
    regrd_hit = active && (msg_kind == MSG_REG_READ) && rr_valid;
    regrd_err = active && (msg_kind == MSG_REG_READ) && rr_addr_err;

    // Register Write: same shape as Register Read. Accepted only with a
    // usable address; a framing-good request with an out-of-range or
    // unaligned address is reported through classifier_error rather than
    // silently truncated onto a register the host did not name.
    //
    // rw_addr_err is used rather than !rw_valid because rx_msg_decode
    // reaches MSG_REG_WRITE on bytes 1 and 6 only, while the parser also
    // checks bytes 0, 5, 10, 11 and 15. A frame that decodes as a register
    // write but is malformed at its tail therefore has rw_addr_err LOW and
    // rw_valid LOW, and is dropped without an error pulse -- matching how
    // the legacy and pixel-write paths treat a framing failure, and
    // deliberately NOT matching the pixrd/burstrd convention, where the
    // parser is the only structural check that exists.
    regwr_hit = active && (msg_kind == MSG_REG_WRITE) && rw_valid;
    regwr_err = active && (msg_kind == MSG_REG_WRITE) && rw_addr_err;

    // Single Pixel Read: accepted only with coordinates that are inside
    // the image when judged on their FULL 24-bit fields.
    //
    // The error term is the complement of the hit rather than
    // pr_coord_err alone, and that is deliberate. rx_msg_decode reaches
    // MSG_PIX_READ on bytes 0, 1, 5, 6 and 11 only; the parser
    // additionally validates bytes 10 and 15. A frame that decodes as
    // MSG_PIX_READ but is malformed at its tail therefore has
    // pr_coord_err low AND pr_valid low, and using pr_coord_err by
    // itself would drop it in silence. !pr_valid covers both the bad
    // framing and the out-of-range coordinate, which is what the
    // diagnostic convention asks for.
    pixrd_hit = active && (msg_kind == MSG_PIX_READ) && pr_valid;
    pixrd_err = active && (msg_kind == MSG_PIX_READ) && !pr_valid;

    // Image Burst Read. Same shape as the Single Pixel Read terms, and
    // the error term is likewise !br_valid rather than br_err alone:
    // rx_msg_decode reaches MSG_BURST_READ on bytes 0, 1, 5 and 6 only,
    // while the parser additionally checks bytes 10, 11 and 15. A frame
    // that decodes as a burst read but is malformed at its tail has
    // br_err low AND br_valid low, so using br_err by itself would drop
    // it silently.
    //
    // NOTE ON EXISTING SUITES: MSG_BURST_READ frames were previously
    // ignored outright -- no hit, no error -- because unknown_msg only
    // fires on MSG_UNKNOWN. tb_rx_pipeline already sends one
    // (send_burst_read with A=0, H=256, W=256). That request is VALID, so
    // it now produces br_cmd_valid and still no error, and the suite's
    // expect_counts(0,0,0) over classifier_valid/error/cmd_valid is
    // unaffected. An INVALID burst read in some future test would newly
    // raise classifier_error, which is the intended behaviour.
    burstrd_hit = active && (msg_kind == MSG_BURST_READ) && br_valid;
    burstrd_err = active && (msg_kind == MSG_BURST_READ) && !br_valid;
end : qualify

// -------------------------------------------------------------------------
// Latch legacy fields on a valid legacy message.
//
// For a legacy frame this condition is identical to the original
// (msg_valid && parse_valid), because msg_kind is MSG_LEGACY_RGF exactly
// when rx_parser's framing check passes. The added term only prevents
// non-legacy frames from disturbing these registers.
// -------------------------------------------------------------------------
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        row_q   <= 10'd0;
        col_q   <= 10'd0;
        pixel_q <= 24'd0;
    end else if (legacy_hit) begin
        row_q   <= row;
        col_q   <= col;
        pixel_q <= pixel;
    end
end

// -------------------------------------------------------------------------
// Latch Single Pixel Write fields
// -------------------------------------------------------------------------
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        cmd_addr  <= 24'd0;
        cmd_pixel <= 24'd0;
    end else if (pixwr_hit) begin
        cmd_addr  <= pw_addr;
        cmd_pixel <= pw_pixel;
    end
end

// -------------------------------------------------------------------------
// Registered one-cycle output pulses (Moore style, matches rest of design)
// -------------------------------------------------------------------------
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        classifier_valid <= 1'b0;
        classifier_error <= 1'b0;
        cmd_valid        <= 1'b0;
        rr_cmd_valid     <= 1'b0;
        rr_cmd_addr      <= 6'd0;
        rw_cmd_valid     <= 1'b0;
        rw_cmd_addr      <= 6'd0;
        rw_cmd_data      <= 32'd0;
        pr_cmd_valid     <= 1'b0;
        pr_cmd_row       <= 10'd0;
        pr_cmd_col       <= 10'd0;
        br_cmd_valid     <= 1'b0;
        br_cmd_base_row  <= 10'd0;
        br_cmd_base_col  <= 10'd0;
        br_cmd_height    <= 10'd0;
        br_cmd_width     <= 10'd0;
    end else begin
        classifier_valid <= legacy_hit;
        classifier_error <= legacy_err || pixwr_err || regrd_err ||
                            regwr_err  || pixrd_err || burstrd_err ||
                            unknown_msg;
        cmd_valid        <= pixwr_hit;
        rr_cmd_valid     <= regrd_hit;
        if (regrd_hit) rr_cmd_addr <= rr_rgf_addr;

        // A rejected Register Write emits NO command, so nothing reaches
        // the RGF and no register changes -- only the error pulse above.
        // Address and data are written on a hit only, so a rejected
        // request cannot leave a stale-but-plausible {addr, data} pair
        // behind for the next producer to trip over.
        rw_cmd_valid     <= regwr_hit;
        if (regwr_hit) begin
            rw_cmd_addr <= rw_rgf_addr;
            rw_cmd_data <= rw_data;
        end

        // A rejected Single Pixel Read emits NO command, so no SRAM
        // access and no reply can follow from it -- only the error pulse
        // above. The coordinate registers are written on a hit only, so a
        // rejected request cannot even leave a stale-but-plausible
        // coordinate behind for the next requester to trip over.
        pr_cmd_valid <= pixrd_hit;
        if (pixrd_hit) begin
            pr_cmd_row <= pr_row;
            pr_cmd_col <= pr_col;
        end

        // A rejected Image Burst Read emits NO command, so no SRAM access
        // and no reply stream can follow -- only the error pulse above.
        // Geometry is written on a hit only, so a rejected request cannot
        // leave a plausible-looking region behind.
        br_cmd_valid <= burstrd_hit;
        if (burstrd_hit) begin
            br_cmd_base_row <= br_base_row;
            br_cmd_base_col <= br_base_col;
            br_cmd_height   <= br_height;
            br_cmd_width    <= br_width;
        end
    end
end

// -------------------------------------------------------------------------
// MUTUAL-EXCLUSION CONTRACT WITH rx_burst_ctrl
// -------------------------------------------------------------------------
// Two modules can drive a command towards the shared command FIFO: this one
// (Single Pixel Write) and rx_burst_ctrl (Image Burst Write). They are
// mutually exclusive, and the exclusion rests on two independent layers:
//
//   STRUCTURAL  While rx_burst_ctrl asserts bypass_active, rx_msg_decode
//               classifies EVERY frame as MSG_BURST_DATA. MSG_PIX_WRITE and
//               MSG_LEGACY_RGF therefore cannot occur, so this module would
//               emit nothing even without the gate above.
//
//   EXPLICIT    Every output is additionally gated on !burst_active, so the
//               exclusion survives a mis-wired bypass_active.
//
// The second layer is not redundant. Burst payload bytes are RAW BINARY, so
// a data frame can legitimately carry 'W' at byte 1 and 'P' at byte 6 -- a
// perfect Single Pixel Write -- or 'R'/'C'/'V' at bytes 1/6/11, a perfect
// legacy RGF command. If bypass_active ever failed, ordinary image data
// would start issuing register writes, and a frame that happened to address
// IMG_CTRL would launch an image read in the middle of a burst.
//
// CONTRACT:
//   burst_active = 1 -> this module is completely inert
//   burst_active = 0 -> rx_burst_ctrl is in B_IDLE and emits nothing
//   the two cmd_valid signals can never assert in the same cycle
// -------------------------------------------------------------------------
`ifndef SYNTHESIS
    // THE THREE RGF COMMAND PRODUCERS ARE MUTUALLY EXCLUSIVE.
    // classifier_valid comes from MSG_LEGACY_RGF, rr_cmd_valid from
    // MSG_REG_READ, rw_cmd_valid from MSG_REG_WRITE. A frame carries exactly
    // one msg_kind_q, so no two can coincide -- chip_top's producer mux
    // depends on this.
    a_one_rgf_producer: assert property (
        @(posedge clk) disable iff (!rst_n)
        $onehot0({classifier_valid, rr_cmd_valid, rw_cmd_valid})
    ) else $error("%m: more than one RGF command producer valid in the same cycle");

    // An accepted Register Write never coincides with its own error.
    a_regwr_hit_xor_err: assert property (
        @(posedge clk) disable iff (!rst_n)
        !(regwr_hit && regwr_err)
    ) else $error("%m: register write both accepted and rejected");

    // Nothing at all may leave this module during a burst.
    a_inert_during_burst: assert property (
        @(posedge clk) disable iff (!rst_n)
        burst_active |-> (!classifier_valid && !classifier_error &&
                          !cmd_valid && !pr_cmd_valid && !br_cmd_valid &&
                          !rr_cmd_valid && !rw_cmd_valid)
    ) else $error("%m: classifier active during a burst");

    // A frame carries exactly one msg_kind_q, so a Single Pixel Read can
    // never coincide with either RGF producer. chip_top relies on this:
    // pr_cmd_valid drives a separate CDC and must not be a third
    // contender for the rgf_src mux.
    a_burstrd_exclusive: assert property (
        @(posedge clk) disable iff (!rst_n)
        br_cmd_valid |-> (!classifier_valid && !rr_cmd_valid &&
                          !rw_cmd_valid && !cmd_valid && !pr_cmd_valid)
    ) else $error("%m: burst read command overlapped another command");

    a_burstrd_hit_xor_err: assert property (
        @(posedge clk) disable iff (!rst_n)
        !(burstrd_hit && burstrd_err)
    ) else $error("%m: burst read both accepted and rejected");

    a_pixrd_exclusive: assert property (
        @(posedge clk) disable iff (!rst_n)
        pr_cmd_valid |-> (!classifier_valid && !rr_cmd_valid &&
                          !rw_cmd_valid && !cmd_valid)
    ) else $error("%m: pixel read command overlapped another command");

    // An accepted Single Pixel Read never coincides with its own error.
    a_pixrd_hit_xor_err: assert property (
        @(posedge clk) disable iff (!rst_n)
        !(pixrd_hit && pixrd_err)
    ) else $error("%m: pixel read both accepted and rejected");

    // The structural layer, checked rather than assumed. If this fires,
    // bypass_active is mis-wired and only the explicit gate is protecting
    // the design.
    a_no_classify_during_burst: assert property (
        @(posedge clk) disable iff (!rst_n)
        (burst_active && msg_valid) |->
            (msg_kind != MSG_PIX_WRITE && msg_kind != MSG_LEGACY_RGF)
    ) else $error("%m: frame classified as an action type during a burst -- bypass_active mis-wired?");
`endif

endmodule : rx_classifier
