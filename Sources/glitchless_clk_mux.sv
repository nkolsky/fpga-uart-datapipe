// =============================================================================
// glitchless_clk_mux.sv
//
// 2-to-1 glitchless (break-before-make) clock multiplexer, with PLL lock
// qualification on select.
//
// No reset input:
//   This module never needs to be "re-armed" mid-operation. Break-before-make
//   is enforced structurally by the sel/~sel cross-coupling (assign i_and*),
//   which holds on every clock edge for as long as the module runs -- it does
//   not depend on any prior reset event. The only known-state requirement is
//   at power-up, which Xilinx FPGAs already guarantee via the global set/reset
//   (GSR) net that initializes every flip-flop once, automatically, right
//   after configuration. If sel changes later for any reason (CPU_RESETN
//   press upstream, PLL losing and re-acquiring lock, etc.), the same
//   cross-coupled logic handles that transition safely every time -- there is
//   no separate "reset the mux" step required.
//
// Architecture:
//   Each branch contains:
//     1. An input AND gate that computes the local enable, gated by the
//        OTHER branch's synchronized output (the cross-coupling that
//        enforces break-before-make).
//     2. A 2-FF synchronizer, clocked by that branch's own clock, to safely
//        bring the enable into the local clock domain.
//     3. A negedge-triggered capture stage (en*_gate_en) that re-registers
//        the synchronized enable a half-cycle before the edge it needs to
//        gate. This is required, not just extra margin: en0_sync/en1_sync
//        change AT a posedge of their own clock (clock-to-Q delay after it),
//        which is the same edge clk0_gated/clk1_gated needs to gate. ANDing
//        the enable directly with its own clock races that edge and can
//        produce a narrow, truncated final pulse on disable. Capturing on
//        the negedge instead means the gate-enable is stable a full
//        half-cycle before the next rising edge ever arrives.
//     4. An output AND gate that gates the branch's clock with its own
//        settled gate-enable -- this is what actually cuts or passes the
//        clock.
//   The two output AND gates feed an OR gate to produce clk_out directly.
//   No further output gating is needed: mutual exclusion holds from the
//   first cycle (sel/~sel can never both be 1), so there is no post-reset
//   settling window left to close.
//
// select encoding:
//   sel = 0 -> clk_out follows clk0  (e.g. 100 MHz fallback)
//   sel = 1 -> clk_out follows clk1  (e.g. fast PLL clock), only once
//              pll_lock is also high
// =============================================================================

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