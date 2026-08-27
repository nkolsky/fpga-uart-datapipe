`timescale 1ns/1ps
// round_robin_arbiter.sv
// ======================
// Round-robin arbiter with grant lock. Rotate-mask implementation: rotate the
// request vector right by the pointer, run a plain fixed-priority pick, rotate
// the grant back.
//
//   rotr     = {2{req}} >> rr_ptr        rotate requests into priority order
//   fixed    = rotr & ~(rotr - 1)        isolate the lowest set bit
//   next_gnt = ({2{fixed}} << rr_ptr)    rotate the grant back
//
// This form scales with N. The unique-case version does not -- it needs a
// hand-written priority chain per pointer value.
//
// LOCK -- why a bare arbiter cannot drive a bus
//
// Fixed priority preempts: a higher-priority requester asserting mid-transfer
// takes the resource away, and on a bus that corrupts an in-flight burst.
// The lock re-arbitrates ONLY when no grant is held, or when the holder has
// dropped its request:
//
//   if ((gnt == 0) || ((gnt & req) == 0)) gnt <= next_gnt;
//
// So a requester holds its grant for as long as it holds req. For an AHB
// burst that means asserting req from the NONSEQ beat until the final data
// phase completes -- the grant cannot be taken away mid-burst, which is the
// rule the arbiter has to respect but cannot enforce on its own.
//
// STARVATION -- why lock alone is not enough
//
// Lock stops preemption but not starvation: with fixed priority the top
// requester wins every fresh arbitration and the others may never run. The
// rotating pointer is what bounds the wait. Each requester in turn becomes
// highest priority, so the worst case is N-1 turns.
//
// THE POINTER WRAP -- the trap for N = 3
//
// PTR_W is $clog2(N), so N=3 gives a 2-bit pointer with range 0..3. Left to
// roll over naturally it would visit 3, and for a 3-bit vector
// {2{req}} >> 3 is identical to >> 0 -- requester 0 would get two turns in
// every four while 1 and 2 got one each. The pointer therefore wraps at N-1,
// not at its natural maximum. Only matters when N is not a power of two,
// which is exactly this design's case.

module round_robin_arbiter #(
    parameter int N = 3
)(
    input  logic         clk,
    input  logic         rst_n,
    input  logic [N-1:0] req,
    output logic [N-1:0] gnt
);

    localparam int PTR_W = (N <= 1) ? 1 : $clog2(N);

    logic [PTR_W-1:0] rr_ptr;

    // -----------------------------------------------------------------
    // Rotate, pick, rotate back
    // -----------------------------------------------------------------
    // The upper half of each rotated vector is the part shifted past the
    // window and is intentionally discarded -- that is what makes this a
    // rotate rather than a shift.
    /* verilator lint_off UNUSEDSIGNAL */
    logic [2*N-1:0] req_rot;
    /* verilator lint_on UNUSEDSIGNAL */
    logic [N-1:0]   rotr;
    logic [N-1:0]   fixed;
    /* verilator lint_off UNUSEDSIGNAL */
    logic [2*N-1:0] gnt_rot;
    /* verilator lint_on UNUSEDSIGNAL */
    logic [N-1:0]   next_gnt;

    assign req_rot  = {2{req}} >> rr_ptr;
    assign rotr     = req_rot[N-1:0];

    // Lowest set bit. Zero when no request, which correctly yields no grant.
    assign fixed    = rotr & ~(rotr - {{(N-1){1'b0}}, 1'b1});

    assign gnt_rot  = {2{fixed}} << rr_ptr;
    assign next_gnt = gnt_rot[2*N-1:N];

    // -----------------------------------------------------------------
    // Grant register, with lock
    // -----------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)                                  gnt <= '0;
        else if ((gnt == '0) || ((gnt & req) == '0)) gnt <= next_gnt;
    end

    // -----------------------------------------------------------------
    // Pointer
    //
    // Advances only once the currently privileged requester has actually
    // taken its turn -- so a requester keeps its privilege until it uses it,
    // rather than losing it while idle.
    // -----------------------------------------------------------------
    logic privileged_served;
    assign privileged_served = gnt[rr_ptr];

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)                    rr_ptr <= '0;
        else if (privileged_served)    rr_ptr <= (rr_ptr == PTR_W'(N-1))
                                                 ? '0
                                                 : rr_ptr + PTR_W'(1);
    end

`ifndef SYNTHESIS
    initial begin
        if (N < 2) $error("%m: N must be at least 2");
    end

    // One resource, one owner.
    a_onehot: assert property (
        @(posedge clk) disable iff (!rst_n)
        $onehot0(gnt)
    ) else $error("%m: grant is not one-hot");

    // A NEW grant must match a request that was live WHEN THE ARBITRATION WAS
    // MADE -- hence $past(req), not req.
    //
    // Two things make the obvious form wrong. gnt is registered, so it is
    // computed from the previous cycle's request vector; and it stays
    // asserted for one cycle after a holder drops req, before the next
    // arbitration lands. So a requester can withdraw in the very cycle it is
    // being granted, and the grant briefly points at nobody. Harmless -- the
    // resource is not in use and the next cycle re-arbitrates -- but
    // (gnt != 0) |-> (gnt & req) fires on it, as it did here.
    a_new_gnt_was_requested: assert property (
        @(posedge clk) disable iff (!rst_n)
        ((gnt != '0) && (gnt != $past(gnt))) |-> ((gnt & $past(req)) != '0)
    ) else $error("%m: issued a grant nobody had asked for");

    // NO PREEMPTION. While the holder keeps req asserted, the grant must not
    // move -- this is the property that protects a burst.
    a_no_preempt: assert property (
        @(posedge clk) disable iff (!rst_n)
        ((gnt != '0) && ((gnt & req) != '0)) |=> $stable(gnt)
    ) else $error("%m: grant moved while the holder still had req asserted");

    // The pointer must stay inside the requester range, or the rotate breaks.
    a_ptr_in_range: assert property (
        @(posedge clk) disable iff (!rst_n)
        rr_ptr < PTR_W'(N)
    ) else $error("%m: rr_ptr (%0d) out of range for N=%0d", rr_ptr, N);
`endif

endmodule : round_robin_arbiter
