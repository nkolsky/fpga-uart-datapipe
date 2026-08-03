// -----------------------------------------------------------------------------
// clocking_subsystem.sv
//
// Clock and reset generation for chip_top.
//
// Contains:
//   - 100 MHz reset synchronizer
//   - clk_wiz_0 (100 MHz -> 130 MHz)
//   - 130 MHz reset synchronizer
//   - synthesis BUFGCTRL / simulation glitchless_clk_mux
//   - PLL-lock qualification for clock selection
//   - heartbeat counter driven by the selected clock
//
// IMPORTANT XDC HIERARCHY CHANGE:
//   u_clk_wiz_0/... becomes
//   u_clocking_subsystem/u_clk_wiz_0/...
// -----------------------------------------------------------------------------
`timescale 1ns/1ps

module clocking_subsystem (
    input  logic clk_100_in,
    input  logic async_reset_n,
    input  logic clk_sel,

    output logic clk_130_out,
    output logic rst_100_n,
    output logic rst_130_n,
    output logic pll_locked,
    output logic heartbeat
);

logic selected_clk;
logic sel_qual;
logic [26:0] heartbeat_cnt;

// Synchronize external reset release into the 100 MHz domain.
reset_synch u_reset_synch (
    .clk         (clk_100_in),
    .async_rst_n (async_reset_n),
    .sync_rst_n  (rst_100_n)
);

// Clocking Wizard: 100 MHz input -> 130 MHz output.
clk_wiz_0 u_clk_wiz_0 (
    .clk_out1 (clk_130_out),
    .resetn   (rst_100_n),
    .locked   (pll_locked),
    .clk_in1  (clk_100_in)
);

// Synchronize external reset release into the 130 MHz domain.
reset_synch u_pll_reset_synch (
    .clk         (clk_130_out),
    .async_rst_n (async_reset_n),
    .sync_rst_n  (rst_130_n)
);

// Do not select the PLL clock until the Clocking Wizard reports lock.
assign sel_qual = clk_sel & pll_locked;

`ifdef SYNTHESIS
    BUFGCTRL u_glitchless_clk_mux (
        .O       (selected_clk),
        .I0      (clk_100_in),
        .I1      (clk_130_out),
        .S0      (~sel_qual),
        .S1      (sel_qual),
        .CE0     (1'b1),
        .CE1     (1'b1),
        .IGNORE0 (1'b0),
        .IGNORE1 (1'b0)
    );
`else
    glitchless_clk_mux u_glitchless_clk_mux (
        .clk0     (clk_100_in),
        .clk1     (clk_130_out),
        .sel      (clk_sel),
        .pll_lock (pll_locked),
        .clk_out  (selected_clk)
    );
`endif

// Deliberately free-running, matching the original chip_top implementation.
always_ff @(posedge selected_clk) begin
    heartbeat_cnt <= heartbeat_cnt + 1'b1;
end

assign heartbeat = heartbeat_cnt[26];

endmodule : clocking_subsystem
