`timescale 1ns/1ps
// tb_round_robin_arbiter.sv
// =========================
// Unit test for round_robin_arbiter at N=3, the case this design uses.
//
// T4 is the one that matters. With all three requesters asking continuously,
// service must be EQUAL. A pointer that rolls over at its natural 2-bit
// maximum instead of at N-1 still grants, still locks, still looks correct on
// a waveform -- it just gives requester 0 two turns in every four. Only a
// fairness count over many rounds finds it.
//
// T3 is the other one: no preemption. A held request must keep its grant even
// while a higher-priority requester is asking, or an AHB burst gets cut in
// half.

module tb_round_robin_arbiter;

    localparam int N = 3;

    logic clk = 1'b0;
    logic rst_n;
    always #5 clk = ~clk;

    logic [N-1:0] req, gnt;

    int errors = 0, checks = 0;
    task automatic chk(input string what, input logic cond);
        checks++;
        if (!cond) begin errors++; $display("  FAIL  %s   (t=%0t)", what, $time); end
        else $display("  pass  %s", what);
    endtask

    round_robin_arbiter #(.N(N)) u_dut (.clk(clk), .rst_n(rst_n), .req(req), .gnt(gnt));

    // service counters
    int served [N];
    int gnt_cycles = 0;
    int s0, sR, sB;

    // Explicit registered copy rather than $past(gnt[i]): with a loop
    // variable as the index, $past does not behave as an edge detector here
    // and the counter ends up counting CYCLES instead of grants. A three-cycle
    // hold then scores three, which is how this was found.
    logic [N-1:0] gnt_q;
    always_ff @(posedge clk) gnt_q <= gnt;

    always @(posedge clk) if (rst_n) begin
        if (gnt != '0) gnt_cycles++;
        for (int i = 0; i < N; i++)
            if (gnt[i] && !gnt_q[i]) served[i]++;        // count grant EDGES
    end

    // Hold a request for `len` cycles, then drop it -- models a burst.
    task automatic hold(input int idx, input int len);
        @(negedge clk);
        req[idx] = 1'b1;
        repeat (len) @(negedge clk);
        req[idx] = 1'b0;
        @(negedge clk);
    endtask

    initial begin
        rst_n = 1'b0;
        req   = '0;
        for (int i = 0; i < N; i++) served[i] = 0;
        repeat (4) @(negedge clk);
        rst_n = 1'b1;
        repeat (2) @(negedge clk);

        $display("\n=== T1: no request, no grant ===");
        repeat (4) @(negedge clk);
        chk("grant stays clear with no request", gnt === 3'b000);

        $display("\n=== T2: a lone requester is granted ===");
        // Stimulus changes on the NEGEDGE throughout. Driving req at the
        // posedge races the arbiter's own sampling, and a_no_preempt fires on
        // a grant that was in fact correct -- which is how this was found.
        @(negedge clk); req = 3'b010;
        repeat (2) @(negedge clk);
        chk("requester 1 granted", gnt === 3'b010);
        chk("grant is one-hot",    $onehot0(gnt));
        @(negedge clk); req = '0;
        repeat (2) @(negedge clk);

        $display("\n=== T3: NO PREEMPTION -- a held grant survives a rival ===");
        @(negedge clk); req = 3'b100;       // requester 2 asks
        repeat (2) @(negedge clk);
        chk("requester 2 granted", gnt === 3'b100);
        @(negedge clk); req = 3'b101;       // requester 0 joins mid-transfer
        repeat (4) @(negedge clk);
        chk("grant stayed with 2 while it held req", gnt === 3'b100);
        @(negedge clk); req = 3'b001;       // 2 finishes
        repeat (2) @(negedge clk);
        chk("grant moved on only after release", gnt === 3'b001);
        @(negedge clk); req = '0;
        repeat (2) @(negedge clk);

        $display("\n=== T4: THE n=3 TEST -- equal service under full load ===");
        for (int i = 0; i < N; i++) served[i] = 0;
        // All three ask continuously; each holds its grant 3 cycles then
        // re-asks, so the pointer gets many rounds to show any bias.
        fork
            begin : drv
                for (int round = 0; round < 30; round++) begin
                    @(negedge clk); req = 3'b111;
                    // whoever holds the grant drops out for a cycle, then
                    // re-joins -- keeps all three permanently contending
                    repeat (3) @(negedge clk);
                    @(negedge clk); req = req & ~gnt;
                end
                @(negedge clk); req = '0;
            end : drv
        join
        repeat (4) @(negedge clk);
        $display("      served: R=%0d G=%0d B=%0d", served[0], served[1], served[2]);
        chk("all three were served",  served[0] > 0 && served[1] > 0 && served[2] > 0);
        // Perfect rotation would be exactly equal; allow one round of skew
        // for where the sequence starts and stops.
        chk("R and G within one turn", (served[0] - served[1] <= 1) &&
                                       (served[1] - served[0] <= 1));
        chk("G and B within one turn", (served[1] - served[2] <= 1) &&
                                       (served[2] - served[1] <= 1));
        chk("R and B within one turn", (served[0] - served[2] <= 1) &&
                                       (served[2] - served[0] <= 1));

        $display("\n=== T5: each requester is served when it asks alone ===");
        @(negedge clk); req = '0;
        repeat (3) @(negedge clk);
        // Snapshot rather than reset: served[] is written by the observer
        // process, so clearing it from the stimulus process is a dual-driver
        // race -- which is what made an earlier version of this test lie.
        s0 = served[0]; hold(0, 3); chk("R served", served[0] == s0 + 1);
        s0 = served[1]; hold(1, 3); chk("G served", served[1] == s0 + 1);
        s0 = served[2]; hold(2, 3); chk("B served", served[2] == s0 + 1);
        s0 = served[0]; hold(0, 3); chk("R served again", served[0] == s0 + 1);

        $display("\n=== T6: a requester that never asks is never granted ===");
        s0 = served[1]; sR = served[0]; sB = served[2];
        for (int round = 0; round < 12; round++) begin
            @(negedge clk); req = 3'b101;   // only R and B
            repeat (3) @(negedge clk);
            @(negedge clk); req = req & ~gnt;
        end
        @(negedge clk); req = '0;
        repeat (3) @(negedge clk);
        chk("G never granted",     served[1] == s0);
        chk("R and B both served", served[0] > sR && served[2] > sB);

        $display("\n----------------------------------------");
        $display("checks: %0d   errors: %0d", checks, errors);
        if (errors == 0) $display("RESULT: PASS");
        else             $display("RESULT: FAIL");
        $display("----------------------------------------\n");
        $finish;
    end

    initial begin
        #500000;
        $display("TIMEOUT");
        $finish;
    end

endmodule : tb_round_robin_arbiter
