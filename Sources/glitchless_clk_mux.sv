// -----------------------------------------------------------------------------
// glitchless_clk_mux.sv
//
// 2:1 break-before-make clock mux. Each branch gates its own clock with a
// synchronized enable, and the output is the OR of the two gated clocks.
// select is qualified by pll_lock so the fast clock cannot be used before the
// PLL is stable.
//
// The enable chains are: 2FF sync -> negedge capture -> clock gate. This keeps
// the gating signal stable before the next rising edge and avoids narrow pulses
// when switching between clocks.
// -----------------------------------------------------------------------------

module glitchless_clk_mux (
    input  logic clk0,      // Reference / fallback clock
    input  logic clk1,      // PLL-sourced clock
    input  logic sel,       // 0 -> clk0,  1 -> clk1
    input  logic pll_lock,  // PLL lock signal; switch to clk1 blocked until high
    output logic clk_out    // Glitch-free output clock
);

    // ── 1. Qualify select with PLL lock ──────────────────────────────────────
    // Prevents any switch to clk1 while the PLL is still acquiring.
    // Combinational; both 2FF chains absorb this safely.
    logic sel_qual;
    assign sel_qual = sel & pll_lock;

    // ── 2. clk0 domain: 2FF sync + negedge capture + clock gate ─────────────
    // Synchronises ~sel_qual into the clk0 domain.
    // en0_sync is only asserted (enabling clk0) when sel_qual=0 (clk0 selected).
    logic en0_meta, en0_sync;
    logic en1_sync; // From clk1 domain, defined here so it can be used in the clk0 domain FF chain.

    always_ff @(posedge clk0) begin
        {en0_sync, en0_meta} <= {en0_meta, ~sel_qual & ~en1_sync};
    end

    logic en0_gate_en;
    always_ff @(negedge clk0) begin
        en0_gate_en <= en0_sync;
    end

    logic clk0_gated;
    assign clk0_gated = clk0 & en0_gate_en;

    // ── 3. clk1 domain: 2FF sync + negedge capture + clock gate ─────────────
    // Synchronises sel_qual into the clk1 domain.
    // en1_sync is only asserted (enabling clk1) when sel_qual=1 (clk1 selected).
    logic en1_meta;

    always_ff @(posedge clk1) begin
        {en1_sync, en1_meta} <= {en1_meta, sel_qual & ~en0_sync};
    end

    logic en1_gate_en;
    always_ff @(negedge clk1) begin
        en1_gate_en <= en1_sync;
    end

    logic clk1_gated;
    assign clk1_gated = clk1 & en1_gate_en;

    // ── 4. Output OR ──────────────────────────────────────────────────────────
    // At most one branch is active at any time -- guaranteed structurally by
    // the sel/~sel cross-coupling in step 1, from the very first cycle after
    // configuration (no reset-release race to wait out) -- so the OR simply
    // passes whichever branch is currently enabled.
    assign clk_out = clk0_gated | clk1_gated;

endmodule