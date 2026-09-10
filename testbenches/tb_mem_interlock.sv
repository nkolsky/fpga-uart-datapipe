// tb_mem_interlock.sv
// ===================
// Three clients, one memory port: the image reader, the read-port borrower
// (single pixel and burst reads), and the WRITE path -- which now needs the
// read port too, because rgb_sram has no per-byte write enable and a single
// pixel update is a read-modify-write.
//
// THE PROPERTY THIS EXISTS TO PROVE: a write must never lose the port
// between its read and its write. If a reader takes the port mid-update, the
// word is corrupted.
//
// Build: TOP = tb_mem_interlock
//        SRCS = memory_pkg.sv mem_interlock.sv tb_mem_interlock.sv

`timescale 1ns/1ps

module tb_mem_interlock;

    typedef enum logic [2:0] {
        P_INIT      = 3'd0,
        P_WRITE     = 3'd1,   // write gets the port when nothing else wants it
        P_HOLD      = 3'd2,   // grant held for a whole update
        P_READBLOCK = 3'd3,   // image read blocks writes
        P_WRFIRST   = 3'd4,   // pending write blocks a read from starting
        P_PIXRD     = 3'd5,   // single-pixel read excludes both
        P_BURST     = 3'd6,   // a burst may write, and blocks reads starting
        P_DONE      = 3'd7
    } tb_phase_e;

    tb_phase_e phase = P_INIT;

    logic clk = 1'b0, rst_n = 1'b0;
    always #5 clk = ~clk;

    logic start_req    = 1'b0;
    logic rom_seq_busy = 1'b0;
    logic img_done     = 1'b0;
    logic read_go;

    logic wr_port_req  = 1'b0;
    logic wr_busy      = 1'b0;
    logic burst_active = 1'b0;
    logic wr_port_grant;

    logic pix_rd_req  = 1'b0;
    logic pix_rd_done = 1'b0;
    logic pix_rd_gnt, pix_rd_owner;

    mem_interlock u_dut (
        .clk(clk), .rst_n(rst_n),
        .start_req(start_req), .rom_seq_busy(rom_seq_busy),
        .img_done(img_done), .read_go(read_go),
        .wr_port_req(wr_port_req), .wr_busy(wr_busy),
        .burst_active(burst_active), .wr_port_grant(wr_port_grant),
        .pix_rd_req(pix_rd_req), .pix_rd_done(pix_rd_done),
        .pix_rd_gnt(pix_rd_gnt), .pix_rd_owner(pix_rd_owner)
    );

    // read_go is a ONE-CYCLE PULSE -- start_pending clears when it fires --
    // so it must be counted, not sampled. An earlier version of this
    // testbench checked the level a few cycles later and saw zero every time.
    int n_read_go;
    always @(posedge clk) if (rst_n && read_go) n_read_go++;

    int checks, fails;

    task automatic ck(input string what, input int got, input int exp);
        checks++;
        if (got !== exp) begin
            $display("  FAIL [%s] %-30s got=%0d exp=%0d",
                     phase.name(), what, got, exp);
            fails++;
        end
    endtask

    task automatic pulse_start();
        @(negedge clk); start_req = 1'b1;
        @(negedge clk); start_req = 1'b0;
    endtask

    task automatic reset_dut();
        @(negedge clk); rst_n = 1'b0;
        rom_seq_busy = 0; wr_port_req = 0; wr_busy = 0;
        burst_active = 0; pix_rd_req = 0; pix_rd_done = 0; img_done = 0;
        @(negedge clk); rst_n = 1'b1;
        repeat (2) @(negedge clk);
        n_read_go = 0;
    endtask

    // -------------------------------------------------------------------
    initial begin
        $dumpfile("tb_mem_interlock_wv.fst");
        $dumpvars(0, tb_mem_interlock);

        checks = 0; fails = 0;

        repeat (4) @(negedge clk);
        rst_n = 1'b1;
        repeat (2) @(negedge clk);

        // ===============================================================
        phase = P_WRITE;
        // Nothing else wants the port.
        // ===============================================================
        ck("granted when idle", int'(wr_port_grant), 1);

        @(negedge clk); wr_port_req = 1'b1;
        repeat (2) @(negedge clk);
        ck("still granted with a request", int'(wr_port_grant), 1);
        @(negedge clk); wr_port_req = 1'b0;

        // ===============================================================
        phase = P_HOLD;
        //
        // THE CRITICAL CASE. An update is in progress -- wr_busy high,
        // meaning sram_rmw is between its read and its write -- and the
        // image reader asks for the port. The grant MUST hold.
        // ===============================================================
        reset_dut();
        @(negedge clk); wr_port_req = 1'b1; wr_busy = 1'b1;
        repeat (2) @(negedge clk);
        ck("granted", int'(wr_port_grant), 1);

        pulse_start();                       // image read requested mid-update
        repeat (10) @(negedge clk);
        ck("grant held through update", int'(wr_port_grant), 1);
        ck("read did not launch",       n_read_go, 0);

        // a pixel read asking as well must also be refused
        @(negedge clk); pix_rd_req = 1'b1;
        repeat (4) @(negedge clk);
        ck("pixel read refused mid-update", int'(pix_rd_gnt), 0);
        ck("grant still held",              int'(wr_port_grant), 1);
        @(negedge clk); pix_rd_req = 1'b0;

        // update finishes -> the queued read may now go
        @(negedge clk); wr_busy = 1'b0; wr_port_req = 1'b0;
        repeat (4) @(negedge clk);
        ck("read launches after the update", n_read_go, 1);

        // ===============================================================
        phase = P_READBLOCK;
        // While rom_sequencer walks addresses, writes stand off.
        // ===============================================================
        reset_dut();
        @(negedge clk); rom_seq_busy = 1'b1;
        repeat (2) @(negedge clk);
        ck("write refused during a read", int'(wr_port_grant), 0);

        @(negedge clk); wr_port_req = 1'b1;
        repeat (4) @(negedge clk);
        ck("still refused with a request", int'(wr_port_grant), 0);

        @(negedge clk); rom_seq_busy = 1'b0;
        repeat (2) @(negedge clk);
        ck("granted once the read ends", int'(wr_port_grant), 1);
        @(negedge clk); wr_port_req = 1'b0;

        // ===============================================================
        phase = P_WRFIRST;
        // A pending write stops a read from launching, so the image reader
        // never starts on a half-written image.
        // ===============================================================
        reset_dut();
        @(negedge clk); wr_port_req = 1'b1;
        pulse_start();
        repeat (6) @(negedge clk);
        ck("read blocked by a pending write", n_read_go, 0);

        @(negedge clk); wr_port_req = 1'b0;
        repeat (4) @(negedge clk);
        ck("read launches once writes clear", n_read_go, 1);

        // ===============================================================
        phase = P_PIXRD;
        // The read-port borrower excludes both other clients.
        // ===============================================================
        reset_dut();
        @(negedge clk); pix_rd_req = 1'b1;
        repeat (3) @(negedge clk);
        ck("borrower granted", int'(pix_rd_owner), 1);
        ck("write refused",    int'(wr_port_grant), 0);

        pulse_start();
        repeat (4) @(negedge clk);
        ck("image read blocked", n_read_go, 0);

        @(negedge clk); pix_rd_req = 1'b0; pix_rd_done = 1'b1;
        @(negedge clk); pix_rd_done = 1'b0;
        repeat (3) @(negedge clk);
        ck("write granted again", int'(wr_port_grant), 1);

        // ===============================================================
        phase = P_BURST;
        // A burst must be able to write, and must stop a read from starting
        // in the quiet gaps between its data messages.
        // ===============================================================
        reset_dut();
        @(negedge clk); burst_active = 1'b1;
        repeat (2) @(negedge clk);
        ck("burst may write", int'(wr_port_grant), 1);

        pulse_start();
        repeat (6) @(negedge clk);
        ck("read blocked during a burst", n_read_go, 0);
        ck("burst still writing",         int'(wr_port_grant), 1);

        @(negedge clk); burst_active = 1'b0;
        repeat (4) @(negedge clk);
        ck("read launches after the burst", n_read_go, 1);

        // ===============================================================
        phase = P_DONE;
        // ===============================================================
        repeat (10) @(negedge clk);

        $display("");
        $display("=====================================================");
        $display("  checks run : %0d", checks);
        $display("  failures   : %0d", fails);
        $display("  RESULT     : %s", (fails == 0) ? "PASS" : "FAIL");
        $display("=====================================================");
        $finish;
    end

    initial begin
        #500000;
        $display("TIMEOUT in phase '%s'", phase.name());
        $finish;
    end

endmodule
