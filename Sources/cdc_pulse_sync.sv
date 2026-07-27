// cdc_pulse_sync.sv
// -----------------
// Reusable single-bit PULSE clock-domain crossing, toggle style.
//
// Converts a one-cycle pulse in the source domain into a one-cycle pulse
// in the destination domain, safely and regardless of the frequency
// ratio or phase relationship between the two clocks.
//
// -----------------------------------------------------------------------
// WHY A PULSE CANNOT SIMPLY BE SAMPLED
// -----------------------------------------------------------------------
// A one-cycle pulse in a 130 MHz domain is 7.69 ns wide. Sampling it
// directly with a 100 MHz clock (10 ns period) can miss it entirely,
// because the destination edge is not guaranteed to fall inside the
// pulse window. Whether it lands depends on the phase relationship.
//
// In this design both clocks derive from the same MMCM, so that phase
// relationship is FIXED rather than random -- which is why the original
// direct connection appeared to work at all. But it is fixed only for a
// given placement: any re-place-and-route, timing change, or tool
// version can shift it and silently start dropping pulses. That is the
// definition of an unreliable design, and it is what this module fixes.
//
// -----------------------------------------------------------------------
// HOW THE TOGGLE SYNCHRONISER WORKS
// -----------------------------------------------------------------------
// The trick is to stop transmitting a pulse and transmit a LEVEL CHANGE
// instead. A level change cannot be missed: once the source toggle flips,
// it STAYS flipped until the next pulse, so the destination is guaranteed
// to observe the new value no matter when it samples.
//
//   1. SOURCE      src_toggle inverts on every src_pulse. The information
//                  is now carried by an edge, not by a pulse width, so
//                  there is no longer anything narrow to miss.
//
//   2. TRANSFER    src_toggle is sampled by a two-flop synchroniser in the
//                  destination domain. The first flop may go metastable;
//                  the second gives it a full destination clock period to
//                  resolve, which is what makes the crossing safe.
//
//   3. EDGE DETECT A third destination flop delays the synchronised
//                  toggle by one cycle. XOR of the two recreates exactly
//                  one destination-domain cycle of pulse per source pulse.
//
// One source pulse in, exactly one destination pulse out. Never zero,
// never two.
//
// -----------------------------------------------------------------------
// MINIMUM PULSE SPACING -- the one constraint the caller must honour
// -----------------------------------------------------------------------
// The source must not issue a second pulse before the first toggle has
// propagated, which takes roughly three destination clocks. Two pulses
// closer than that would flip the toggle twice inside one synchroniser
// window and the destination would see either one pulse or none.
//
// Required spacing: about 3 destination clocks, i.e. ~30 ns for a 100 MHz
// destination, or ~4 source cycles at 130 MHz.
//
// Both users of this module in chip_top are far outside that limit:
//   tx_img_done        fires once per whole image transfer (millions of
//                      cycles apart)
//   rx_parity_err_pulse at most once per received UART byte, ~1.35 us
//                      apart at 8.125 Mbaud -- roughly 175 destination
//                      clocks, a ~58x margin
//
// If a future user cannot guarantee this spacing, it needs a full
// four-phase request/acknowledge handshake instead, not this module.
//
// -----------------------------------------------------------------------
// TIMING CONSTRAINT -- REQUIRED, see the XDC note in chip_top
// -----------------------------------------------------------------------
// CLK100MHZ and the 130 MHz PLL output are RELATED clocks (same MMCM), so
// Vivado will attempt to time the src_toggle -> sync_ff[0] path as an
// ordinary synchronous path. For a 13:10 ratio the tightest launch/capture
// relationship is only 0.769 ns, which is not achievable and will be
// reported as a large violation.
//
// That path must therefore be constrained as a CDC path in the XDC. This
// is not cosmetic: without it the tool wastes effort on an impossible
// path and the timing report hides real violations.

`timescale 1ns/1ps

module cdc_pulse_sync #(
    // Number of destination-domain synchroniser flops. Must be >= 2.
    // Two is standard; three buys additional MTBF margin if the
    // destination clock is very fast or the design is high-reliability.
    parameter int SYNC_STAGES = 2
)(
    // ---- source domain ----------------------------------------------
    input  logic src_clk,
    input  logic src_rst_n,
    input  logic src_pulse,   // one source-clock cycle wide

    // ---- destination domain -----------------------------------------
    input  logic dst_clk,
    input  logic dst_rst_n,
    output logic dst_pulse    // one destination-clock cycle wide
);

    // -----------------------------------------------------------------
    // Stage 1: source-domain toggle.
    //
    // This is the whole idea: the pulse is converted into a level change
    // that persists until the next pulse, so it cannot be missed.
    // -----------------------------------------------------------------
    logic src_toggle;

    always_ff @(posedge src_clk or negedge src_rst_n) begin
        if (!src_rst_n)      src_toggle <= 1'b0;
        else if (src_pulse)  src_toggle <= ~src_toggle;
    end

    // -----------------------------------------------------------------
    // Stage 2: destination-domain synchroniser.
    //
    // ASYNC_REG tells Vivado these are synchroniser flops: it places them
    // in the same slice where possible, which maximises the settling time
    // available between stages and therefore the mean time between
    // failures. It also stops the tool from optimising the chain away or
    // retiming through it. Without this attribute the flops can be spread
    // across the die and the MTBF collapses.
    //
    // sync_ff[0] is the flop that may go metastable; every later stage
    // exists purely to let it settle.
    // -----------------------------------------------------------------
    (* ASYNC_REG = "TRUE" *) logic [SYNC_STAGES-1:0] sync_ff;

    // Stage 3: edge detect. NOT a synchroniser flop -- its input is
    // already resolved, so it deliberately carries no ASYNC_REG.
    logic sync_toggle_d;

    always_ff @(posedge dst_clk or negedge dst_rst_n) begin
        if (!dst_rst_n) begin
            sync_ff       <= '0;
            sync_toggle_d <= 1'b0;
        end else begin
            sync_ff       <= {sync_ff[SYNC_STAGES-2:0], src_toggle};
            sync_toggle_d <= sync_ff[SYNC_STAGES-1];
        end
    end

    // -----------------------------------------------------------------
    // Output: one destination cycle per observed toggle edge.
    //
    // Combinational XOR of two destination-domain flops, which is the
    // standard form. Both change on the same edge, so any transient is
    // sub-nanosecond and settles long before the next capture edge --
    // safe for driving a synchronous enable, which is how both users in
    // chip_top consume it. If a consumer ever needs a fully registered
    // output, add a flop here; it costs one cycle of latency and nothing
    // in this design is latency-sensitive.
    // -----------------------------------------------------------------
    assign dst_pulse = sync_ff[SYNC_STAGES-1] ^ sync_toggle_d;

    // -----------------------------------------------------------------
    // Reset note.
    //
    // src_toggle and the synchroniser chain both reset to 0, so a reset
    // leaves them agreeing and produces no spurious output pulse.
    //
    // If the two domains leave reset at meaningfully different times AND
    // a source pulse occurs inside that window, the destination will emit
    // the pulse when it releases -- which is the correct outcome, since
    // the event genuinely happened. In this design both resets come from
    // reset_synch instances driven by the same CPU_RESETN, so they
    // release within a couple of cycles of each other anyway.
    // -----------------------------------------------------------------

endmodule : cdc_pulse_sync
