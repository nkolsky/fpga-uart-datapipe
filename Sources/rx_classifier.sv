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

    // Burst opcode bypass, fed back to rx_msg_decode.
    // STAGE 2A: hardwired inactive. When burst write is implemented this
    // becomes a level driven by an H*W pixel countdown held in this
    // module. The port and the feedback path exist now so that adding
    // burst support requires no change to rx_mac or rx_msg_decode.
    output logic        bypass_active
);

// -------------------------------------------------------------------------
// Kind-qualified hit / error terms
// -------------------------------------------------------------------------
logic legacy_hit, legacy_err;
logic pixwr_hit,  pixwr_err;
logic unknown_msg;

always_comb begin : qualify
    legacy_hit  = msg_valid && (msg_kind == MSG_LEGACY_RGF) && parse_valid;
    legacy_err  = msg_valid && (msg_kind == MSG_LEGACY_RGF) && parse_error;

    pixwr_hit   = msg_valid && (msg_kind == MSG_PIX_WRITE)  && pw_parse_valid;
    pixwr_err   = msg_valid && (msg_kind == MSG_PIX_WRITE)  && pw_parse_error;

    // Frames rx_msg_decode could not classify at all. Preserves the
    // pre-Stage-2A behaviour of pulsing an error on garbage input.
    unknown_msg = msg_valid && (msg_kind == MSG_UNKNOWN);
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
// Burst bypass -- inactive for the whole of Stage 2A.
// -------------------------------------------------------------------------
assign bypass_active = 1'b0;

endmodule : rx_classifier
