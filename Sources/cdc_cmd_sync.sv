// -----------------------------------------------------------------------------
// cdc_cmd_sync.sv
//
// Single-entry command CDC. The source captures the full command word when
// src_valid is asserted, then sends a single dst_valid pulse with the address,
// write flag, and payload together so the destination sees one atomic command.
//
// While dst_valid is low, dst_addr is forced to IDLE_ADDR so level-sensitive
// decoders do not act on stale data. The source is expected to space pulses far
// enough apart to avoid overwriting an in-flight transfer.
// -----------------------------------------------------------------------------

`timescale 1ns/1ps

module cdc_cmd_sync #(
    parameter int                ADDR_W                 = 8,
    parameter int                DATA_W                 = 32,
    // Address presented whenever dst_valid is low. Must decode to nothing
    // in the destination address map.
    parameter logic [ADDR_W-1:0] IDLE_ADDR              = '1,
    // Destination synchroniser depth, passed through to cdc_pulse_sync.
    parameter int                SYNC_STAGES            = 2,
    // Simulation-only spacing check, in source clocks.
    parameter int                MIN_SPACING_SRC_CYCLES = 8
)(
    // ---- source domain ----------------------------------------------
    input  logic              src_clk,
    input  logic              src_rst_n,
    input  logic              src_valid,     // one source-clock pulse
    input  logic              src_is_write,  // 1 = write, 0 = read
    input  logic [ADDR_W-1:0] src_addr,
    input  logic [DATA_W-1:0] src_wdata,

    // ---- destination domain -----------------------------------------
    output logic              dst_valid,     // one destination-clock pulse
    output logic              dst_is_write,
    output logic [ADDR_W-1:0] dst_addr,      // IDLE_ADDR while !dst_valid
    output logic [DATA_W-1:0] dst_wdata,     // held between transfers
    input  logic              dst_clk,
    input  logic              dst_rst_n
);

    // -----------------------------------------------------------------
    // Payload capture, source domain.
    //
    // Written on the same edge that flips the toggle below. That
    // simultaneity is what makes the transfer atomic -- see the header.
    // -----------------------------------------------------------------
    logic              cap_is_write;
    logic [ADDR_W-1:0] cap_addr;
    logic [DATA_W-1:0] cap_wdata;

    always_ff @(posedge src_clk or negedge src_rst_n) begin
        if (!src_rst_n) begin
            cap_is_write <= 1'b0;
            cap_addr     <= '0;
            cap_wdata    <= '0;
        end else if (src_valid) begin
            cap_is_write <= src_is_write;
            cap_addr     <= src_addr;
            cap_wdata    <= src_wdata;
        end
    end

    // -----------------------------------------------------------------
    // Event path.
    //
    // Reuses the hardware-validated toggle synchroniser from Change B.
    // One src_valid in, exactly one dst_valid out -- never zero, never
    // two -- regardless of the clock ratio or phase relationship.
    // -----------------------------------------------------------------
    cdc_pulse_sync #(
        .SYNC_STAGES (SYNC_STAGES)
    ) u_event_sync (
        .src_clk   (src_clk),
        .src_rst_n (src_rst_n),
        .src_pulse (src_valid),
        .dst_clk   (dst_clk),
        .dst_rst_n (dst_rst_n),
        .dst_pulse (dst_valid)
    );

    // -----------------------------------------------------------------
    // Destination outputs.
    //
    // dst_addr / dst_is_write are qualified by dst_valid so the
    // transaction is presented for exactly one destination cycle. This is
    // what makes a level-sensitive read decode fire once and only once.
    //
    // dst_wdata is deliberately NOT qualified: it simply holds the last
    // captured value. The destination only samples write data while its
    // write enable is asserted, which by construction is only during the
    // dst_valid cycle, so gating it would add a mux for no benefit.
    // -----------------------------------------------------------------
    assign dst_is_write = dst_valid ? cap_is_write : 1'b0;
    assign dst_addr     = dst_valid ? cap_addr     : IDLE_ADDR;
    assign dst_wdata    = cap_wdata;

    // -----------------------------------------------------------------
    // Simulation-only minimum-spacing check.
    //
    // Purely source-domain -- no cross-clock assertion, no sampling of
    // destination signals. gap_cnt saturates rather than wrapping, so a
    // long idle period cannot alias back into a false failure, and it
    // starts saturated so the first command after reset always passes.
    //
    // Zero synthesis footprint: the counter itself is inside the guard.
    // -----------------------------------------------------------------
`ifndef SYNTHESIS
    localparam int GAP_W = 16;
    logic [GAP_W-1:0] gap_cnt;

    always_ff @(posedge src_clk or negedge src_rst_n) begin
        if (!src_rst_n)                gap_cnt <= '1;
        else if (src_valid)            gap_cnt <= '0;
        else if (gap_cnt != '1)        gap_cnt <= gap_cnt + 1'b1;
    end

    a_min_spacing: assert property (
        @(posedge src_clk) disable iff (!src_rst_n)
        src_valid |-> (gap_cnt >= GAP_W'(MIN_SPACING_SRC_CYCLES))
    )
    else $error("%m: src_valid pulses only %0d source cycles apart, minimum is %0d. A command may be lost.",
                gap_cnt, MIN_SPACING_SRC_CYCLES);
`endif

endmodule : cdc_cmd_sync
