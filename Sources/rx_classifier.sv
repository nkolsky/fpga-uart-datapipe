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
// STAGE 2A: MESSAGE-KIND ROUTING
// -----------------------------------------------------------------------
// Two parsers now run in parallel on the same completed frame:
//
//   rx_parser            legacy {Rnnn,Cnnn,Vnnn}  -> RGF control path
//   rx_pixel_wr_parser   {W<A>, P<R,G,B>}         -> pixel write command
//
// msg_kind (rx_mac's LATCHED msg_kind_q, never the provisional decode
// output) selects which one is authoritative for a given frame.
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
// merely because Stage 2A has no consumer for it.
//
//   raises an error:
//     MSG_LEGACY_RGF with parse_error      (bad legacy framing)
//     MSG_PIX_WRITE  with pw_parse_error   (bad pixel-write framing)
//     MSG_UNKNOWN                          (unclassifiable frame)
//
//   silently ignored -- recognised, valid, no consumer yet:
//     MSG_REG_WRITE   MSG_REG_READ   MSG_PIX_READ
//     MSG_BURST_HDR   MSG_BURST_READ MSG_BURST_DATA
//
// The legacy protocol remains the active register-control path until the
// new register protocol is implemented and verified separately. A
// silently dropped valid message looks like a bug, so it is called out
// here deliberately.
//
// KNOWN LIMITATION: rx_msg_decode inspects only bytes 0, 1, 5, 6 and 11.
// A message of an unimplemented kind whose OTHER bytes are corrupt --
// say a Register Write with a bad terminator at byte 15 -- still
// classifies as MSG_REG_WRITE and is silently dropped rather than
// flagged. Full structural validation of those kinds needs their own
// parsers, which Stage 2A deliberately does not add. When those parsers
// arrive their error terms join the expression below in the same shape
// as legacy and pixel-write do now.

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

    // Single Pixel Write command. Stage 2A has no consumer for the data
    // buses yet -- the command FIFO arrives in Stage 2B.
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
    input  logic        burst_active
);

// -------------------------------------------------------------------------
// Kind-qualified hit / error terms
// -------------------------------------------------------------------------
logic legacy_hit, legacy_err;
logic pixwr_hit,  pixwr_err;
logic unknown_msg;

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
    end else begin
        classifier_valid <= legacy_hit;
        classifier_error <= legacy_err || pixwr_err || unknown_msg;
        cmd_valid        <= pixwr_hit;
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
    // Nothing at all may leave this module during a burst.
    a_inert_during_burst: assert property (
        @(posedge clk) disable iff (!rst_n)
        burst_active |-> (!classifier_valid && !classifier_error && !cmd_valid)
    ) else $error("%m: classifier active during a burst");

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
