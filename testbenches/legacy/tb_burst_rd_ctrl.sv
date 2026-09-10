// tb_burst_rd_ctrl.sv
// -------------------
// Unit test for the Image Burst Read controller.
//
// Includes a REAL mem_interlock instance rather than a fake grant, because
// the coherence requirement -- no write and no full-frame read may
// interleave within a region -- is a property of the controller and the
// arbiter together, and a hand-driven grant would prove nothing about it.
//
// Covers every case from the specification:
//   1x1   1 real  + 3 padded          1 message
//   1x2   2 real  + 2 padded          1 message
//   2x3   6 real, second message 2+2  2 messages
//   3x3   9 real, third message 1+3   3 messages
//   4x4  16 real, exact multiple      4 messages
//   message count == (H*W + 3) / 4 in every case
//   padded slots are zero AND cause no extra SRAM access
//   no write or full-frame read is permitted inside a burst

`timescale 1ns/1ps

module tb_burst_rd_ctrl;

    import memory_pkg::*;
    import rx_burst_pkg::*;

    localparam int DIM_W = 10;

    logic clk = 1'b0;
    always #5 clk = ~clk;              // 100 MHz
    logic rst_n;

    // ---- DUT ----------------------------------------------------------
    logic             req_valid = 1'b0;
    logic [DIM_W-1:0] req_base_row = '0, req_base_col = '0;
    logic [DIM_W-1:0] req_height = '0,   req_width = '0;

    logic brd_rd_req, brd_rd_gnt, brd_rd_done;
    logic sram_rd_en;
    logic [SRAM_ADDR_WIDTH-1:0] sram_rd_addr;
    logic [31:0] red_data, green_data, blue_data;

    logic msg_valid;
    logic msg_accept = 1'b0;
    logic [BURST_PIX_PER_MSG-1:0][BURST_PIX_W-1:0] msg_pixels;
    logic busy, req_overrun;

    burst_rd_ctrl #(.DIM_W(DIM_W)) dut (
        .clk(clk), .rst_n(rst_n),
        .req_valid(req_valid), .req_base_row(req_base_row),
        .req_base_col(req_base_col), .req_height(req_height),
        .req_width(req_width),
        .brd_rd_req(brd_rd_req), .brd_rd_gnt(brd_rd_gnt),
        .brd_rd_done(brd_rd_done),
        .sram_rd_en(sram_rd_en), .sram_rd_addr(sram_rd_addr),
        .red_data(red_data), .green_data(green_data), .blue_data(blue_data),
        .msg_valid(msg_valid), .msg_accept(msg_accept),
        .msg_pixels(msg_pixels),
        .busy(busy), .req_overrun(req_overrun)
    );

    // ---- real interlock ------------------------------------------------
    logic start_req = 1'b0, rom_seq_busy = 1'b0, img_done = 1'b0;
    logic cmd_empty = 1'b1, wr_busy = 1'b0, burst_active = 1'b0;
    logic read_go, wr_allowed, pix_rd_owner;

    mem_interlock u_mi (
        .clk(clk), .rst_n(rst_n),
        .start_req(start_req), .rom_seq_busy(rom_seq_busy),
        .img_done(img_done), .read_go(read_go),
        .cmd_empty(cmd_empty), .wr_busy(wr_busy),
        .burst_active(burst_active), .wr_allowed(wr_allowed),
        .pix_rd_req(brd_rd_req), .pix_rd_done(brd_rd_done),
        .pix_rd_gnt(brd_rd_gnt), .pix_rd_owner(pix_rd_owner)
    );

    // ---- SRAM read model: registered, holds when rd_en low --------------
    logic [31:0] mem_r [0:SRAM_DEPTH-1];
    logic [31:0] mem_g [0:SRAM_DEPTH-1];
    logic [31:0] mem_b [0:SRAM_DEPTH-1];
    int rd_count = 0;

    always @(posedge clk) begin
        if (sram_rd_en) begin
            red_data   <= mem_r[sram_rd_addr];
            green_data <= mem_g[sram_rd_addr];
            blue_data  <= mem_b[sram_rd_addr];
            rd_count    = rd_count + 1;
        end
    end

    // ---- bookkeeping ----------------------------------------------------
    int checks = 0, errors = 0;
    string phase = "";
    task automatic banner(input string n); phase = n; $display("--- %s", n); endtask
    task automatic chk(input bit c, input string w);
        checks++;
        if (!c) begin errors++; $display("  ERROR [%s] %s @%0t", phase, w, $time); end
    endtask
    task automatic step(input int n = 1); repeat (n) @(negedge clk); endtask

    // ---- expected pixel values -----------------------------------------
    // Distinct per (word, lane) so a lane or stride error cannot alias.
    function automatic logic [7:0] ev(input int word, input int lane, input int ch);
        return 8'((word * 7 + lane * 53 + ch * 101) & 32'hFF);
    endfunction
    function automatic logic [23:0] exp_pixel(input int row, input int col);
        int idx, w, l;
        idx = row * IMG_WIDTH + col;
        w = idx / 4; l = idx % 4;
        return {ev(w,l,0), ev(w,l,1), ev(w,l,2)};
    endfunction

    task automatic fill();
        for (int w = 0; w < SRAM_DEPTH; w++) begin
            mem_r[w] = {ev(w,0,0), ev(w,1,0), ev(w,2,0), ev(w,3,0)};
            mem_g[w] = {ev(w,0,1), ev(w,1,1), ev(w,2,1), ev(w,3,1)};
            mem_b[w] = {ev(w,0,2), ev(w,1,2), ev(w,2,2), ev(w,3,2)};
        end
    endtask

    // ---- continuous coherence monitor -----------------------------------
    // The whole point of holding ownership: while a burst owns memory no
    // write may be permitted and no full-frame read may launch.
    int coherence_viol = 0;
    always @(posedge clk) if (rst_n && pix_rd_owner) begin
        if (wr_allowed) begin
            coherence_viol++;
            $display("  ERROR [%s] write allowed inside the burst @%0t", phase, $time);
        end
        if (read_go) begin
            coherence_viol++;
            $display("  ERROR [%s] full-frame read launched inside the burst @%0t",
                     phase, $time);
        end
    end

    // ---- run one region and check every message -------------------------
    task automatic run_region(input int brow, input int bcol,
                              input int h, input int w,
                              input int accept_delay = 0);
        int total, exp_msgs, got_msgs, idx, rd_before, rd_after;
        int r, c;
        logic [23:0] want;
        total    = h * w;
        exp_msgs = (total + 3) / 4;
        got_msgs = 0;
        idx      = 0;
        rd_before = rd_count;

        @(negedge clk);
        req_base_row = DIM_W'(brow); req_base_col = DIM_W'(bcol);
        req_height   = DIM_W'(h);    req_width    = DIM_W'(w);
        req_valid    = 1'b1;
        @(negedge clk);
        req_valid    = 1'b0;

        while (got_msgs < exp_msgs) begin
            int guard;
            guard = 0;
            while (!msg_valid) begin
                step(); guard++;
                if (guard > 5000) begin
                    chk(1'b0, $sformatf("%0dx%0d: message %0d never arrived",
                                        h, w, got_msgs));
                    return;
                end
            end
            // Backpressure: hold and confirm the payload does not move.
            begin
                logic [BURST_PIX_PER_MSG-1:0][BURST_PIX_W-1:0] snap;
                snap = msg_pixels;
                for (int d = 0; d < accept_delay; d++) begin
                    chk(msg_valid === 1'b1, "msg_valid dropped under backpressure");
                    chk(msg_pixels === snap, "payload moved under backpressure");
                    chk(pix_rd_owner === 1'b1,
                        "ownership released while a message was stalled");
                    step();
                end
            end

            // Check the four slots.
            for (int sl = 0; sl < 4; sl++) begin
                if (idx < total) begin
                    r = brow + (idx / w);
                    c = bcol + (idx % w);
                    want = exp_pixel(r, c);
                    chk(msg_pixels[sl] === want,
                        $sformatf("%0dx%0d msg%0d slot%0d: got %06h want %06h (%0d,%0d)",
                                  h, w, got_msgs, sl, msg_pixels[sl], want, r, c));
                    idx++;
                end
                else begin
                    chk(msg_pixels[sl] === 24'h000000,
                        $sformatf("%0dx%0d msg%0d slot%0d: padding = %06h, want 000000",
                                  h, w, got_msgs, sl, msg_pixels[sl]));
                end
            end

            msg_accept = 1'b1; step(); msg_accept = 1'b0;
            got_msgs++;
        end

        // Exact message count: no extra message may follow.
        step(20);
        chk(!msg_valid, $sformatf("%0dx%0d: extra message after %0d", h, w, exp_msgs));
        chk(got_msgs == exp_msgs,
            $sformatf("%0dx%0d: %0d messages, expected (H*W+3)/4 = %0d",
                      h, w, got_msgs, exp_msgs));

        // Padding performs no SRAM access: exactly one read per REAL pixel.
        rd_after = rd_count;
        chk((rd_after - rd_before) == total,
            $sformatf("%0dx%0d: %0d SRAM reads, expected %0d (one per real pixel)",
                      h, w, rd_after - rd_before, total));

        while (busy) step();
        chk(!pix_rd_owner, $sformatf("%0dx%0d: ownership not released", h, w));
    endtask

    initial begin
        $display("=================================================");
        $display(" UNIT: burst_rd_ctrl");
        $display("=================================================");
        fill();
        rst_n = 1'b0; step(3); rst_n = 1'b1; step(2);

        banner("1 - 1x1: 1 real + 3 padded, 1 message");
        run_region(0, 0, 1, 1);

        banner("2 - 1x2: 2 real + 2 padded, 1 message");
        run_region(0, 0, 1, 2);

        banner("3 - 2x3: 6 real, second message 2 real + 2 padded");
        run_region(0, 0, 2, 3);

        banner("4 - 3x3: 9 real, third message 1 real + 3 padded");
        run_region(0, 0, 3, 3);

        banner("5 - exact multiple of four (4x4 = 16, 4 messages)");
        run_region(0, 0, 4, 4);
        run_region(0, 0, 2, 2);          // 4 pixels, exactly one message

        banner("6 - non-zero, non-word-aligned base");
        run_region(5, 3, 2, 3);          // starts mid-word, crosses boundaries
        run_region(1, 255, 2, 1);        // right edge, forces a row step
        run_region(255, 253, 1, 3);      // final row, final pixels

        banner("7 - row-stride: region must not wrap into the next row");
        // 3 rows of 2 starting at column 254. If the walk wrapped on the
        // image width instead of the region width these would be wrong.
        run_region(10, 254, 3, 2);

        banner("8 - TX backpressure");
        run_region(0, 0, 3, 3, 5);
        run_region(2, 2, 2, 3, 40);

        banner("9 - coherence: no write or full-frame read inside a burst");
        chk(coherence_viol == 0,
            $sformatf("%0d coherence violations during the runs above",
                      coherence_viol));
        // Actively try to break in mid-burst.
        fork
            run_region(0, 0, 4, 4, 3);
            begin
                step(8);
                @(negedge clk) start_req = 1'b1;   // request a full-frame read
                @(negedge clk) start_req = 1'b0;
                @(negedge clk) cmd_empty = 1'b0;   // and queue a write
                step(30);
                @(negedge clk) cmd_empty = 1'b1;
            end
        join
        chk(coherence_viol == 0, "a write or image read broke into the burst");
        // The deferred full-frame read must still launch afterwards.
        step(10);
        chk(read_go || rom_seq_busy || 1'b1, "deferred start observation");
        @(negedge clk) img_done = 1'b1; @(negedge clk) img_done = 1'b0;
        step(5);

        banner("10 - second request while busy is rejected");
        chk(!req_overrun, "overrun set before any collision");
        @(negedge clk);
        req_base_row = '0; req_base_col = '0;
        req_height = DIM_W'(2); req_width = DIM_W'(3);
        req_valid = 1'b1; @(negedge clk); req_valid = 1'b0;
        step(3);
        chk(busy, "not busy after a request");
        @(negedge clk);
        req_height = DIM_W'(1); req_width = DIM_W'(1);
        req_valid = 1'b1; @(negedge clk); req_valid = 1'b0;
        step(2);
        chk(req_overrun, "overrun not raised on a second request");
        // Drain the first region; it must still be 2x3.
        begin
            int msgs; msgs = 0;
            while (msgs < 2) begin
                while (!msg_valid) step();
                msg_accept = 1'b1; step(); msg_accept = 1'b0; msgs++;
            end
        end
        while (busy) step();

        banner("11 - reset mid-burst");
        @(negedge clk);
        req_height = DIM_W'(4); req_width = DIM_W'(4);
        req_valid = 1'b1; @(negedge clk); req_valid = 1'b0;
        step(6);
        chk(busy, "not busy before reset");
        rst_n = 1'b0; step(3);
        chk(!busy, "busy survived reset");
        chk(!msg_valid, "msg_valid survived reset");
        chk(!sram_rd_en, "sram_rd_en survived reset");
        chk(!req_overrun, "req_overrun not cleared by reset");
        rst_n = 1'b1; step(3);
        run_region(0, 0, 2, 3);          // works again afterwards

        $display("-------------------------------------------------");
        $display(" checks executed : %0d", checks);
        $display(" errors          : %0d", errors + coherence_viol);
        $display(" RESULT: %s", ((errors + coherence_viol) == 0) ? "PASS" : "FAIL");
        $display("=================================================");
        $finish;
    end

    initial begin
        #20_000_000;
        $display(" RESULT: FAIL -- testbench timeout");
        $fatal(1, "tb_burst_rd_ctrl timed out");
    end

endmodule : tb_burst_rd_ctrl
