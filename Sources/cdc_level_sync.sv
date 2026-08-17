// cdc_level_sync.sv
// -----------------
// Two-flop synchronizer for a level signal.
//
// Use this for signals that stay asserted long enough to be sampled in the
// destination domain. Do not use it for one-cycle pulses; use cdc_pulse_sync
// for those.
//
// -----------------------------------------------------------------------
// CDC behavior
// -----------------------------------------------------------------------
// The first flop may go metastable, but the second flop gives it one full
// destination clock to resolve. The output is therefore a delayed but stable
// version of the source level.
//
// -----------------------------------------------------------------------
// Timing note
// -----------------------------------------------------------------------
// Even with clocks derived from the same MMCM, this path is still a CDC path
// and the XDC should treat it as such.

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
