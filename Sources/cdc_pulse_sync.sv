// cdc_pulse_sync.sv
// -----------------
// Single-bit pulse CDC using a toggle synchronizer.
//
// A 1-cycle source pulse becomes a 1-cycle destination pulse without relying
// on the pulse width landing inside a destination clock edge.

// -----------------------------------------------------------------------
// How it works
// -----------------------------------------------------------------------
// src_toggle flips on every source pulse. That level change is then sampled
// across the CDC boundary and converted back to a single destination pulse by
// XORing the synchronized toggle with its previous value.
//
// The key rule is: do not send a second source pulse before the first toggle
// has propagated through the 2-FF synchronizer. The caller must keep at least
// about 3 destination clocks between pulses.

// -----------------------------------------------------------------------
// Timing note
// -----------------------------------------------------------------------
// This path is a real CDC boundary, even if both clocks come from the same
// MMCM. Vivado still needs the proper CDC constraint in the XDC.

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
