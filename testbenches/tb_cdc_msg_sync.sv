// tb_cdc_msg_sync.sv
// ==================
// Message crossing: no loss, no duplication, correct order, and a real stall
// when the destination blocks.
//
// The two clocks are deliberately unrelated in frequency and phase, since a
// crossing that only works at a tidy ratio is not a crossing.
//
// Build: TOP = tb_cdc_msg_sync
//        SRCS = cdc_msg_sync.sv tb_cdc_msg_sync.sv

`timescale 1ns/1ps

module tb_cdc_msg_sync;

    localparam int W = 100;

    typedef enum logic [2:0] {
        P_INIT    = 3'd0,
        P_SINGLE  = 3'd1,
        P_STREAM  = 3'd2,
        P_STALL   = 3'd3,
        P_SLOWDST = 3'd4,
        P_DONE    = 3'd5
    } tb_phase_e;

    tb_phase_e phase = P_INIT;

    // 130 MHz-ish source, 100 MHz-ish destination. Not a round ratio.
    logic src_clk = 1'b0;  always #3.846 src_clk = ~src_clk;
    logic dst_clk = 1'b0;  always #5.000 dst_clk = ~dst_clk;

    logic src_rst_n = 1'b0, dst_rst_n = 1'b0;

    logic         src_valid = 1'b0, src_ready;
    logic [W-1:0] src_data  = '0;

    logic         dst_valid, dst_ready;
    logic [W-1:0] dst_data;

    cdc_msg_sync #(.WIDTH(W)) u_cdc (
        .src_clk(src_clk), .src_rst_n(src_rst_n),
        .src_valid(src_valid), .src_ready(src_ready), .src_data(src_data),
        .dst_clk(dst_clk), .dst_rst_n(dst_rst_n),
        .dst_valid(dst_valid), .dst_ready(dst_ready), .dst_data(dst_data)
    );

    // -------------------------------------------------------------------
    // Reference: everything sent, everything received.
    // -------------------------------------------------------------------
    logic [W-1:0] sent_q [$];
    logic [W-1:0] recv_q [$];

    always @(posedge dst_clk) if (dst_rst_n && dst_valid && dst_ready)
        recv_q.push_back(dst_data);

    int checks, fails;

    task automatic ck(input string what, input int got, input int exp);
        checks++;
        if (got !== exp) begin
            $display("  FAIL [%s] %-22s got=%0d exp=%0d",
                     phase.name(), what, got, exp);
            fails++;
        end
    endtask

    // Compare the two queues in order, then clear both.
    task automatic compare_and_clear(input string label);
        checks++;
        if (sent_q.size() != recv_q.size()) begin
            $display("  FAIL [%s] %s count sent=%0d recv=%0d",
                     phase.name(), label, sent_q.size(), recv_q.size());
            fails++;
        end else begin
            for (int i = 0; i < sent_q.size(); i++) begin
                if (sent_q[i] !== recv_q[i]) begin
                    $display("  FAIL [%s] %s msg %0d sent=%h recv=%h",
                             phase.name(), label, i, sent_q[i], recv_q[i]);
                    fails++;
                end
            end
        end
        $display("    %s: %0d sent, %0d received", label,
                 sent_q.size(), recv_q.size());
        sent_q.delete();
        recv_q.delete();
    endtask

    task automatic send(input logic [W-1:0] d);
        @(negedge src_clk);
        src_data = d; src_valid = 1'b1;
        while (!src_ready) @(negedge src_clk);
        @(posedge src_clk);
        @(negedge src_clk);
        src_valid = 1'b0;
        sent_q.push_back(d);
    endtask

    // Destination readiness has EXACTLY ONE DRIVER. An earlier version set
    // it from the stimulus block as well as from here; the two assignments
    // raced, dst_ready glitched, and the DUT's a_dst_holds assertion fired --
    // correctly, on a testbench fault. The stimulus now only sets the two
    // mode flags below.
    logic       dst_block = 1'b0;    // hold ready low, as a long read would
    logic       slow_dst  = 1'b0;    // accept one cycle in sixteen
    logic [3:0] slow_cnt  = '0;

    always @(posedge dst_clk) begin
        slow_cnt <= slow_cnt + 4'd1;
        if      (dst_block) dst_ready <= 1'b0;
        else if (slow_dst)  dst_ready <= (slow_cnt == 4'd0);
        else                dst_ready <= 1'b1;
    end

    // -------------------------------------------------------------------
    initial begin
        $dumpfile("tb_cdc_msg_sync_wv.fst");
        $dumpvars(0, tb_cdc_msg_sync);

        checks = 0; fails = 0;

        repeat (5) @(posedge src_clk);
        src_rst_n = 1'b1; dst_rst_n = 1'b1;
        repeat (5) @(posedge src_clk);

        // ===============================================================
        phase = P_SINGLE;
        // ===============================================================
        send({4'd4, 96'hAAAA_BBBB_CCCC_DDDD_EEEE_FFFF});
        repeat (20) @(posedge dst_clk);
        compare_and_clear("one message");

        // ===============================================================
        phase = P_STREAM;
        // Back to back, source going as fast as src_ready allows. This is
        // far faster than real traffic and is the point: the handshake, not
        // a rate assumption, is what keeps them in step.
        // ===============================================================
        for (int i = 0; i < 20; i++)
            send({4'(i % 9), 92'd0, 4'(i)});
        repeat (40) @(posedge dst_clk);
        compare_and_clear("20 back to back");

        // ===============================================================
        phase = P_STALL;
        // The destination blocks for a long time -- as mem_interlock does
        // when an image read holds the SRAM port. The source must NOT be
        // able to push a second message through, and nothing may be lost.
        // ===============================================================
        dst_block = 1'b1;
        fork
            begin
                send({4'd6, 96'h1111_1111_1111_1111_1111_1111});
                send({4'd7, 96'h2222_2222_2222_2222_2222_2222});
                send({4'd8, 96'h3333_3333_3333_3333_3333_3333});
            end
            begin
                // While blocked, the source must go not-ready and stay there.
                repeat (200) @(posedge dst_clk);
                ck("src stalled while dst blocked", int'(src_ready), 0);
                ck("only one msg presented",        int'(dst_valid), 1);
                dst_block = 1'b0;
            end
        join
        repeat (60) @(posedge dst_clk);
        compare_and_clear("3 through a long stall");

        // ===============================================================
        phase = P_SLOWDST;
        // Destination takes a message only every so often.
        // ===============================================================
        slow_dst = 1'b1;
        for (int i = 0; i < 8; i++)
            send({4'd2, 92'd0, 4'(i)});
        slow_dst = 1'b0;
        repeat (80) @(posedge dst_clk);
        compare_and_clear("8 with a slow consumer");

        // ===============================================================
        phase = P_DONE;
        // ===============================================================
        repeat (10) @(posedge dst_clk);

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
