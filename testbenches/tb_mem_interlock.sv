// tb_mem_interlock.sv
// -------------------
// PERMANENT REGRESSION TESTBENCH for mem_interlock.
//
// Rerun this whenever mem_interlock changes. Every check is self-checking
// with an explicit PASS/FAIL summary -- no waveform inspection required.
//
// -----------------------------------------------------------------------
// WHY THIS EXISTS
// -----------------------------------------------------------------------
// Two successive defects were introduced into this module during Stage 2C
// and NEITHER was reachable from the Lab 10 hardware regression:
//
//   1. read_go asserted while rom_seq_busy was already high. rom_sequencer
//      only samples `start` in IDLE, so the pulse did nothing -- but
//      start_pending was cleared anyway and the request vanished.
//
//   2. A mid-transfer duplicate start was QUEUED and later launched on
//      tx_img_done. That is the same event that SETS IMG_TX_MON.complete,
//      so the second transfer would have run with complete set and never
//      cleared by the host -- routing around the exact condition the RGF
//      interlock exists to enforce.
//
// Both were found by reading the RTL, not by testing it. That is what this
// file is meant to replace.
//
// -----------------------------------------------------------------------
// REFERENCE BEHAVIOUR (mem_interlock.sv, Stage 2C)
// -----------------------------------------------------------------------
//   wr_pending  = !cmd_empty || wr_busy
//   read_go     = start_pending && !wr_pending && !img_in_flight   (comb)
//   read_active = read_go || rom_seq_busy                          (comb)
//   wr_allowed  = !read_active                                     (comb)
//
//   img_in_flight : set by read_go, cleared by img_done
//   start_pending : set by (start_req && !img_in_flight && !read_go),
//                   cleared by read_go
//
// Because read_go is combinational off registered state, a start_req in
// cycle N produces read_go in cycle N+1.
//
// -----------------------------------------------------------------------
// TIMING DISCIPLINE
// -----------------------------------------------------------------------
// Stimulus is driven on the NEGATIVE edge. Internal state is read via
// hierarchical reference for the white-box checks that scenarios 2 and 7
// explicitly require.
//
// SAMPLING read_go -- read this before adding checks.
//
// read_go is COMBINATIONAL: start_pending && !wr_pending && !img_in_flight.
// It therefore responds within the same negedge-to-posedge window as the
// input that released it, and is deasserted again by the very next posedge
// (which clears start_pending and sets img_in_flight).
//
// So a check written as
//
//     set_writes(0);  step(1);  chk(read_go, ...);   // WRONG
//
// steps over the capturing posedge and always reads 0. The correct form is
//
//     set_writes(0);  #1ps;  chk(read_go, ...);      // right
//
// The #1ps is a settling delta only -- it lets the combinational network
// resolve after the negedge assignment without advancing to another edge.
//
// Checks that sample read_go a full cycle after pulse_start() are safe,
// because pulse_start() returns one negedge after asserting start_req, by
// which time start_pending is already set and read_go is high for that
// whole cycle. Only checks that sample in the SAME cycle as the releasing
// assignment need the delta.

`timescale 1ns/1ps

module tb_mem_interlock;

    // -----------------------------------------------------------------
    // Clock and DUT
    // -----------------------------------------------------------------
    logic clk = 1'b0;
    always #5 clk = ~clk;              // 100 MHz, the memory domain

    logic rst_n;
    logic start_req    = 1'b0;
    logic rom_seq_busy = 1'b0;
    logic img_done     = 1'b0;
    logic cmd_empty    = 1'b1;         // idle = FIFO empty
    logic wr_busy      = 1'b0;
    logic burst_active = 1'b0;         // M4: burst in progress on the RX side
    logic read_go;
    logic wr_allowed;

    mem_interlock dut (
        .clk          (clk),
        .rst_n        (rst_n),
        .start_req    (start_req),
        .rom_seq_busy (rom_seq_busy),
        .img_done     (img_done),
        .read_go      (read_go),
        .cmd_empty    (cmd_empty),
        .wr_busy      (wr_busy),
        .burst_active (burst_active),
        .wr_allowed   (wr_allowed)
    );

    // Convenience aliases onto internal state (white-box observation only;
    // the testbench never drives these).
    wire start_pending = dut.start_pending;
    wire img_in_flight = dut.img_in_flight;
    wire wr_pending    = dut.wr_pending;

    // -----------------------------------------------------------------
    // Bookkeeping
    // -----------------------------------------------------------------
    int    errors      = 0;
    int    checks      = 0;
    int    read_go_cnt = 0;
    string phase       = "init";

    // read_go is combinational; count it where the consumer samples it.
    always @(posedge clk) if (rst_n && read_go) read_go_cnt++;

    task automatic clear_go_cnt(); read_go_cnt = 0; endtask

    // -----------------------------------------------------------------
    // Check primitives
    // -----------------------------------------------------------------
    task automatic chk(input bit cond, input string what);
        checks++;
        if (!cond) begin
            errors++;
            $display("  [%0t] FAIL (%s): %s", $time, phase, what);
        end
    endtask

    task automatic chk_eq(input int got, input int exp, input string what);
        checks++;
        if (got !== exp) begin
            errors++;
            $display("  [%0t] FAIL (%s): %s -- got %0d, expected %0d",
                     $time, phase, what, got, exp);
        end
    endtask

    task automatic banner(input string name);
        phase = name;
        $display("--- %s", name);
    endtask

    // -----------------------------------------------------------------
    // Stimulus helpers
    // -----------------------------------------------------------------
    task automatic step(input int n = 1);
        repeat (n) @(negedge clk);
    endtask

    task automatic pulse_start();
        @(negedge clk); start_req = 1'b1;
        @(negedge clk); start_req = 1'b0;
    endtask

    task automatic pulse_img_done();
        @(negedge clk); img_done = 1'b1;
        @(negedge clk); img_done = 1'b0;
    endtask

    task automatic set_writes(input bit pending);
        @(negedge clk);
        cmd_empty = ~pending;          // FIFO non-empty == writes pending
    endtask

    task automatic do_reset();
        rst_n        = 1'b0;
        start_req    = 1'b0;
        rom_seq_busy = 1'b0;
        img_done     = 1'b0;
        cmd_empty    = 1'b1;
        wr_busy      = 1'b0;
        burst_active = 1'b0;
        step(4);
        rst_n = 1'b1;
        step(2);
        clear_go_cnt();
    endtask

    // Behavioural stand-in for rom_sequencer: busy asserts the cycle after
    // read_go (rom_sequencer registers busy <= (next_state != IDLE)), runs
    // for `walk` cycles, then drops.
    task automatic mimic_rom_walk(input int walk);
        @(negedge clk);
        rom_seq_busy = 1'b1;
        step(walk);
        rom_seq_busy = 1'b0;
        @(negedge clk);
    endtask

    // -----------------------------------------------------------------
    // Continuous invariants -- these hold for the WHOLE run, so a defect
    // introduced anywhere is caught even outside its own scenario.
    // -----------------------------------------------------------------
    always @(posedge clk) begin
        if (rst_n) begin
            // Core mutual exclusion: the SRAM write port and rom_sequencer's
            // read port must never both be enabled.
            if (wr_allowed && rom_seq_busy) begin
                errors++;
                $display("  [%0t] FAIL (invariant/%s): wr_allowed high while rom_seq_busy",
                         $time, phase);
            end
            // A burst must never be interrupted by an image read.
            if (read_go && burst_active) begin
                errors++;
                $display("  [%0t] FAIL (invariant/%s): read_go during an active burst",
                         $time, phase);
            end
            // ...but a burst must not block its own writes.
            if (burst_active && !read_go && !rom_seq_busy && !wr_allowed) begin
                errors++;
                $display("  [%0t] FAIL (invariant/%s): writes blocked during a burst",
                         $time, phase);
            end
            // A read must never launch with writes pending or in flight.
            if (read_go && wr_pending) begin
                errors++;
                $display("  [%0t] FAIL (invariant/%s): read_go with wr_pending",
                         $time, phase);
            end
            // Defect 1: read_go must never coincide with an active walk.
            if (read_go && rom_seq_busy) begin
                errors++;
                $display("  [%0t] FAIL (invariant/%s): read_go while rom_seq_busy",
                         $time, phase);
            end
            // A read must never launch while one is already in flight.
            if (read_go && img_in_flight) begin
                errors++;
                $display("  [%0t] FAIL (invariant/%s): read_go while img_in_flight",
                         $time, phase);
            end
            // wr_allowed is defined as the exact complement of read_active.
            if (wr_allowed !== !(read_go || rom_seq_busy)) begin
                errors++;
                $display("  [%0t] FAIL (invariant/%s): wr_allowed != !(read_go||rom_seq_busy)",
                         $time, phase);
            end
        end
    end

    // -----------------------------------------------------------------
    // Test body
    // -----------------------------------------------------------------
    initial begin
        $display("=================================================");
        $display(" REGRESSION: mem_interlock");
        $display("=================================================");

        do_reset();

        // =============================================================
        banner("1 - normal image start");
        // No transfer active, no writes pending. One start_req must produce
        // exactly one read_go, one cycle later, and img_in_flight must then
        // stay asserted until img_done.
        // =============================================================
        chk(!img_in_flight, "img_in_flight low before start");
        chk(!start_pending, "start_pending low before start");
        chk(wr_allowed,     "wr_allowed high when idle");

        pulse_start();                       // start_req during cycle N
        chk(start_pending, "start_pending set the cycle after start_req");
        chk(read_go,       "read_go asserted the cycle after start_req");

        step(1);                             // advance past the read_go cycle
        chk(!read_go,       "read_go deasserted after one cycle");
        chk(img_in_flight,  "img_in_flight set by read_go");
        chk(!start_pending, "start_pending cleared by read_go");
        chk_eq(read_go_cnt, 1, "exactly one read_go");

        // img_in_flight must persist across the whole transfer, not just
        // the address walk.
        mimic_rom_walk(20);
        chk(img_in_flight, "img_in_flight still set after address walk ends");
        step(30);                            // FIFO/UART drain period
        chk(img_in_flight, "img_in_flight still set during drain");
        chk_eq(read_go_cnt, 1, "still exactly one read_go");

        // =============================================================
        banner("2 - duplicate start during an active transfer");
        // img_in_flight is still 1 from scenario 1. A start_req here must
        // be REJECTED outright: start_pending must not latch, and no second
        // read_go may ever occur -- not now, and not later on img_done.
        // =============================================================
        chk(img_in_flight, "precondition: transfer still in flight");

        pulse_start();
        chk(!start_pending, "start_pending NOT latched during transfer");
        chk(!read_go,       "no read_go during transfer");

        step(10);
        chk(!start_pending, "start_pending still clear");
        chk_eq(read_go_cnt, 1, "no second read_go");

        // The critical part: completing the transfer must NOT launch the
        // rejected request. This is the defect that queued duplicates.
        pulse_img_done();
        step(10);
        chk(!img_in_flight, "img_in_flight cleared by img_done");
        chk_eq(read_go_cnt, 1, "rejected duplicate did NOT launch on img_done");

        // =============================================================
        banner("3 - start deferred while writes drain");
        // The one legitimate use of start_pending. A start arriving with no
        // transfer in flight but writes still pending must be HELD, then
        // launched exactly once when wr_pending clears.
        // =============================================================
        clear_go_cnt();
        set_writes(1);                       // command FIFO non-empty
        step(2);
        chk(wr_pending,  "precondition: wr_pending high");
        chk(wr_allowed,  "writes permitted while no read active");

        pulse_start();
        chk(start_pending, "start_pending latched (deferred)");
        chk(!read_go,      "read_go blocked by wr_pending");

        step(20);
        chk(start_pending, "request still held while writes pending");
        chk_eq(read_go_cnt, 0, "no read_go while writes pending");

        // read_go is COMBINATIONAL off wr_pending, so it asserts the instant
        // cmd_empty is released -- during the same negedge-to-posedge window,
        // not on the following cycle. Sample it here, after a settling delta;
        // waiting a full @(negedge clk) would step over the posedge that
        // clears start_pending and deasserts read_go again.
        set_writes(0);                       // drain completes, returns at negedge
        #1ps;
        chk(read_go, "read_go fires once writes drain");

        step(1);                             // cross the capturing posedge
        chk(!read_go,       "read_go is one cycle wide");
        chk(img_in_flight,  "img_in_flight set");
        chk_eq(read_go_cnt, 1, "exactly one read_go after drain");

        // Tidy up: finish this transfer.
        mimic_rom_walk(10);
        pulse_img_done();
        step(4);
        chk(!img_in_flight, "img_in_flight cleared");

        // =============================================================
        banner("4 - start coincident with read_go");
        // Arrange a deferred request, release the writes so read_go fires,
        // and drive start_req in that same cycle. The !read_go term in the
        // set condition must prevent a second request being created.
        // =============================================================
        clear_go_cnt();
        set_writes(1);
        step(2);
        pulse_start();                       // deferred by wr_pending
        chk(start_pending, "request deferred");

        // Release writes and drive start_req in the SAME cycle read_go is
        // high. Because read_go is combinational off wr_pending, it asserts
        // the moment cmd_empty is released -- so both must be driven on the
        // same negedge and held across the following posedge, which is the
        // edge that evaluates the start_pending set condition:
        //
        //     start_req && !img_in_flight && !read_go
        //
        // with read_go high, the !read_go term must block the set.
        @(negedge clk);
        cmd_empty = 1'b1;                    // read_go goes high immediately
        start_req = 1'b1;                    // coincident start, same cycle
        #1ps;
        chk(read_go,       "read_go high in the coincidence cycle");
        chk(start_req,     "start_req high in the same cycle");

        @(negedge clk);                      // cross the posedge that sees both
        start_req = 1'b0;

        step(4);
        chk(!start_pending, "coincident start_req did NOT create a new request");
        chk(img_in_flight,  "transfer launched");
        chk_eq(read_go_cnt, 1, "exactly one read_go, no extra");

        // And it must not surface later either.
        pulse_img_done();
        step(10);
        chk_eq(read_go_cnt, 1, "no deferred launch on img_done");
        chk(!img_in_flight, "img_in_flight cleared");

        // =============================================================
        banner("5 - write exclusion during SRAM read");
        // wr_allowed must be low for the whole of read_go and the address
        // walk, and must return high once the walk ends -- writes only need
        // to stand off for the walk, not for the drain, because a word that
        // has been read is already safe in the pixel FIFO.
        // =============================================================
        clear_go_cnt();
        chk(wr_allowed, "wr_allowed high before start");

        pulse_start();
        chk(read_go,     "read_go asserted");
        chk(!wr_allowed, "wr_allowed low during read_go");

        @(negedge clk);
        rom_seq_busy = 1'b1;                 // walk begins, contiguous with read_go
        step(1);
        chk(!wr_allowed, "wr_allowed low during address walk");
        step(20);
        chk(!wr_allowed, "wr_allowed still low mid-walk");

        rom_seq_busy = 1'b0;
        @(negedge clk);
        step(1);
        chk(wr_allowed,    "wr_allowed returns high after the walk");
        chk(img_in_flight, "but the transfer is still in flight");

        pulse_img_done();
        step(4);

        // =============================================================
        banner("6 - image completion and relaunch");
        // img_done clears img_in_flight, and a subsequent valid start_req
        // launches a genuinely new transfer.
        // =============================================================
        clear_go_cnt();
        chk(!img_in_flight, "idle after previous completion");

        pulse_start();
        step(1);
        chk_eq(read_go_cnt, 1, "second transfer launched");
        chk(img_in_flight,  "img_in_flight set again");

        mimic_rom_walk(15);
        pulse_img_done();
        step(4);
        chk(!img_in_flight, "img_in_flight cleared again");

        pulse_start();
        step(1);
        chk_eq(read_go_cnt, 2, "third transfer launched");
        mimic_rom_walk(15);
        pulse_img_done();
        step(4);

        // =============================================================
        banner("7 - reset");
        // Reset from a fully loaded state must clear both flags and return
        // the outputs to idle.
        // =============================================================
        // Load the module up: transfer in flight AND a pending request.
        clear_go_cnt();
        pulse_start();
        step(2);
        chk(img_in_flight, "transfer in flight before reset");
        set_writes(1);
        rom_seq_busy = 1'b1;
        step(2);
        chk(!wr_allowed, "wr_allowed low before reset");

        @(negedge clk);
        rst_n = 1'b0;
        step(2);
        chk(!start_pending, "reset clears start_pending");
        chk(!img_in_flight, "reset clears img_in_flight");
        chk(!read_go,       "read_go low in reset");

        // Release reset with the inputs back at idle and confirm recovery.
        @(negedge clk);
        rom_seq_busy = 1'b0;
        cmd_empty    = 1'b1;
        wr_busy      = 1'b0;
        rst_n        = 1'b1;
        step(2);
        chk(wr_allowed,     "wr_allowed high after reset release");
        chk(!img_in_flight, "img_in_flight still clear");

        clear_go_cnt();
        pulse_start();
        step(1);
        chk_eq(read_go_cnt, 1, "module operates normally after reset");
        chk(img_in_flight, "transfer launched post-reset");
        mimic_rom_walk(10);
        pulse_img_done();
        step(4);

        // =============================================================
        banner("8 - wr_busy also gates read_go");
        // wr_pending is (!cmd_empty || wr_busy). Scenario 3 exercised the
        // cmd_empty term; this covers the wr_busy term, which is what holds
        // off a read while a command is mid-pipeline in sram_wr_ctrl.
        // =============================================================
        clear_go_cnt();
        @(negedge clk); wr_busy = 1'b1;
        step(2);
        chk(wr_pending, "wr_pending high via wr_busy");

        pulse_start();
        chk(start_pending, "request deferred by wr_busy");
        chk(!read_go,      "read_go blocked by wr_busy");
        step(10);
        chk_eq(read_go_cnt, 0, "no read_go while wr_busy");

        @(negedge clk); wr_busy = 1'b0;
        step(2);
        chk_eq(read_go_cnt, 1, "read_go fires once wr_busy clears");
        mimic_rom_walk(10);
        pulse_img_done();
        step(4);

        // =============================================================
        banner("9 - start_req during a burst becomes pending");
        // The request must be latched, not rejected: no transfer is in
        // flight, so this is the legitimate deferral case.
        // =============================================================
        clear_go_cnt();
        @(negedge clk); burst_active = 1'b1;
        step(2);
        chk(wr_pending, "wr_pending high via burst_active");
        chk(wr_allowed, "writes still permitted during the burst");

        pulse_start();
        chk(start_pending, "start_pending latched during the burst");
        chk(!read_go,      "read_go blocked by burst_active");
        step(20);
        chk(start_pending, "request still held");
        chk_eq(read_go_cnt, 0, "no read_go during the burst");

        // =============================================================
        banner("10 - read_go stays low in the inter-message gaps");
        // The decisive case. Between burst data frames the command FIFO
        // genuinely empties and the write pipeline genuinely idles, so
        // cmd_empty and !wr_busy are both true -- exactly the condition
        // that would have fired read_go before M4. burst_active is the
        // only thing holding it off.
        // =============================================================
        chk(cmd_empty,     "gap: cmd_empty is high");
        chk(!wr_busy,      "gap: wr_busy is low");
        chk(burst_active,  "gap: burst_active is high");
        chk(!read_go,      "gap: read_go STILL low");
        chk(start_pending, "gap: request still deferred");

        // Several gaps in a row, mimicking frame/gap/frame/gap traffic.
        for (int f = 0; f < 4; f++) begin
            @(negedge clk); cmd_empty = 1'b0; wr_busy = 1'b1;   // frame lands
            step(3);
            @(negedge clk); cmd_empty = 1'b1; wr_busy = 1'b0;   // drains
            step(6);
            chk(!read_go, $sformatf("gap %0d: read_go still low", f));
        end
        chk_eq(read_go_cnt, 0, "no read_go across any gap");

        // =============================================================
        banner("11 - dropping burst_active alone is not sufficient");
        // With commands still queued, or the write pipeline still busy,
        // the read must remain deferred even after the burst ends.
        // =============================================================
        @(negedge clk); cmd_empty = 1'b0;        // commands still queued
        step(2);
        @(negedge clk); burst_active = 1'b0;     // burst ends
        step(6);
        chk(!read_go,      "burst over but commands queued -> still blocked");
        chk(start_pending, "request still held");
        chk_eq(read_go_cnt, 0, "no read_go yet");

        @(negedge clk); cmd_empty = 1'b1; wr_busy = 1'b1;   // FIFO drained,
        step(6);                                             // pipeline busy
        chk(!read_go,      "FIFO empty but wr_busy -> still blocked");
        chk_eq(read_go_cnt, 0, "still no read_go");

        // =============================================================
        banner("12 - exactly one read_go once all three clear");
        // =============================================================
        @(negedge clk); wr_busy = 1'b0;
        #1ps;
        chk(read_go, "read_go fires once burst, FIFO and pipeline are all clear");
        step(1);
        chk(!read_go, "read_go is one cycle wide");
        chk_eq(read_go_cnt, 1, "exactly one read_go");
        chk(!start_pending, "start_pending consumed");

        mimic_rom_walk(12);
        chk(wr_allowed, "writes resume after the address walk");
        pulse_img_done();
        step(4);

        // =============================================================
        banner("13 - reset clears a deferred request during a burst");
        // =============================================================
        clear_go_cnt();
        @(negedge clk); burst_active = 1'b1;
        step(2);
        pulse_start();
        chk(start_pending, "request deferred during the burst");

        @(negedge clk); rst_n = 1'b0;
        step(2);
        chk(!start_pending, "reset cleared the deferred request");
        chk(!img_in_flight, "reset cleared img_in_flight");
        chk(!read_go,       "read_go low in reset");

        @(negedge clk); burst_active = 1'b0; rst_n = 1'b1;
        step(3);
        chk_eq(read_go_cnt, 0, "the deferred request did not survive reset");

        // Normal operation resumes.
        clear_go_cnt();
        pulse_start();
        step(1);
        chk_eq(read_go_cnt, 1, "operates normally after reset");
        mimic_rom_walk(10);
        pulse_img_done();
        step(4);

        // =============================================================
        banner("14 - duplicate start during a burst-deferred transfer");
        // Preserves the Stage 2C rejection semantics: once a transfer is in
        // flight a second start is rejected, burst or no burst.
        // =============================================================
        clear_go_cnt();
        pulse_start();
        step(2);
        chk(img_in_flight, "transfer launched");

        @(negedge clk); burst_active = 1'b1;
        step(2);
        pulse_start();                        // duplicate, mid-transfer
        step(4);
        chk(!start_pending, "duplicate rejected even with a burst active");
        chk_eq(read_go_cnt, 1, "no second read_go");

        @(negedge clk); burst_active = 1'b0;
        pulse_img_done();
        step(8);
        chk_eq(read_go_cnt, 1, "rejected duplicate did not launch on img_done");
        chk(!img_in_flight, "transfer completed");

        // =============================================================
        $display("-------------------------------------------------");
        $display(" checks executed : %0d", checks);
        $display(" errors          : %0d", errors);
        $display(" RESULT: %s", (errors == 0) ? "PASS" : "FAIL");
        $display("=================================================");
        if (errors != 0) $fatal(1, "tb_mem_interlock FAILED");
        $finish;
    end

    // Safety net so a hang cannot masquerade as a pass.
    initial begin
        #500us;
        $display(" RESULT: FAIL -- testbench timeout");
        $fatal(1, "tb_mem_interlock timed out");
    end

endmodule : tb_mem_interlock
