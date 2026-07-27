// cdc_level_sync.sv
// -----------------
// Two-flop level clock-domain crossing.
//
// For LEVELS only -- signals that stay asserted long enough for the
// destination to sample them. Do NOT use it for pulses: a one-cycle pulse
// can be narrower than the destination clock period and be missed entirely.
// cdc_pulse_sync exists for that case, and converts the pulse to a toggle
// first.
//
// -----------------------------------------------------------------------
// WHY TWO FLOPS ARE SUFFICIENT HERE
// -----------------------------------------------------------------------
// A single flop sampling an asynchronous input can go metastable. The
// second flop gives it a full destination clock period to resolve, which is
// what buys the mean-time-between-failures. Nothing more is needed for a
// single-bit level, because there is no bus to skew: the destination either
// sees the old value or the new one, never a mixture.
//
// The cost is LATENCY, not correctness. The destination observes a change
// two to three destination clocks after it happens, and both edges are
// delayed. A consumer must therefore tolerate seeing the level late -- which
// is a design question at the point of use, not a property of this module.
//
// -----------------------------------------------------------------------
// ASYNC_REG
// -----------------------------------------------------------------------
// Marks these as synchroniser flops so Vivado places them in the same slice
// where possible, maximising the settling time between stages, and stops the
// tool retiming or merging through the chain. Without it the flops can be
// spread across the die and the MTBF collapses.
//
// -----------------------------------------------------------------------
// TIMING CONSTRAINT
// -----------------------------------------------------------------------
// Where the two clocks are RELATED -- as CLK100MHZ and the 130 MHz PLL
// output are, both from the same MMCM -- Vivado will try to time the
// src_level -> sync_ff[0] path as an ordinary synchronous path, against a
// launch/capture relationship of only 0.769 ns for a 13:10 ratio. That is
// not achievable and the path must be constrained as a CDC path in the XDC.
// The same note applies to cdc_pulse_sync and cdc_cmd_sync.

`timescale 1ns/1ps

module cdc_level_sync #(
    // Number of destination-domain synchroniser flops. Must be >= 2.
    parameter int SYNC_STAGES = 2,
    // Value presented while the destination is in reset.
    parameter bit RESET_VALUE = 1'b0
)(
    input  logic src_level,   // asynchronous to dst_clk
    input  logic dst_clk,
    input  logic dst_rst_n,
    output logic dst_level
);

    (* ASYNC_REG = "TRUE" *) logic [SYNC_STAGES-1:0] sync_ff;

    always_ff @(posedge dst_clk or negedge dst_rst_n) begin
        if (!dst_rst_n) sync_ff <= {SYNC_STAGES{RESET_VALUE}};
        else            sync_ff <= {sync_ff[SYNC_STAGES-2:0], src_level};
    end

    assign dst_level = sync_ff[SYNC_STAGES-1];

endmodule : cdc_level_sync
