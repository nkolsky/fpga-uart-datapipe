`timescale 1ns/1ps
// tb_burst_read_path.sv
// =====================
// Step 6b: the whole read path, every module real.
//
//   img_burst_reader -> round_robin_arbiter
//                    -> ahb_master -> ahb_decoder -> 3x ahb_slave_sram
//                                                 -> 3x rgb_sram
//                    -> 3x async_fifo
//
// The property that matters is the same as 6a's: EQUIVALENCE. Replacing a
// parallel read with per-channel INCR4 bursts must not change what ends up
// in the FIFOs. Each channel's FIFO must receive that channel's words, in
// address order, with none missing or duplicated.
//
// T3 is the one worth having. Under the old parallel read the three FIFOs
// held identical occupancy; under per-channel bursts red runs ahead of blue.
// If they stayed in lockstep the bursts would not be per-channel at all.

module tb_burst_read_path;

    import ahb_pkg::*;

    localparam int IMG_W  = 8;
    localparam int IMG_H  = 8;
    localparam int NWORDS = (IMG_W * IMG_H) / 4;    // 16 words per channel
    localparam int SRAM_ADDR_W = 14;

    logic clk = 1'b0;
    logic rst_n;
    always #5 clk = ~clk;

    int errors = 0, checks = 0;
    task automatic chk(input string what, input logic cond);
        checks++;
        if (!cond) begin errors++; $display("  FAIL  %s   (t=%0t)", what, $time); end
        else $display("  pass  %s", what);
    endtask

    // ------------------------------------------------------------------
    // Burst engine
    // ------------------------------------------------------------------
    logic                  start, seq_done, rdr_busy;
    logic [NUM_SLAVES-1:0] fifo_almost_full, fifo_wr_en;
    logic [DATA_W-1:0]     fifo_wr_data;
    logic                  req_valid, req_write, req_burst;
    logic [SEL_W-1:0]      req_channel;
    logic [WORD_W-1:0]     req_word;
    logic                  ahb_busy, rd_valid;
    logic [DATA_W-1:0]     rd_data;
    logic [1:0]            rd_beat;
    logic [NUM_SLAVES-1:0] arb_req, arb_gnt;

    img_burst_reader #(.IMG_WIDTH(IMG_W), .IMG_HEIGHT(IMG_H)) u_reader (
        .clk(clk), .rst_n(rst_n),
        .start(start), .seq_done(seq_done), .busy(rdr_busy),
        .fifo_almost_full(fifo_almost_full),
        .fifo_wr_en(fifo_wr_en), .fifo_wr_data(fifo_wr_data),
        .req_valid(req_valid), .req_write(req_write), .req_burst(req_burst),
        .req_channel(req_channel), .req_word(req_word), .ahb_busy(ahb_busy),
        .rd_valid(rd_valid), .rd_data(rd_data),
        .arb_req(arb_req), .arb_gnt(arb_gnt)
    );

    round_robin_arbiter #(.N(NUM_SLAVES)) u_arb (
        .clk(clk), .rst_n(rst_n), .req(arb_req), .gnt(arb_gnt)
    );

    // ------------------------------------------------------------------
    // AHB fabric
    // ------------------------------------------------------------------
    logic [ADDR_W-1:0] haddr;
    logic              hwrite;
    logic [2:0]        hsize, hburst;
    logic [1:0]        htrans;
    logic [DATA_W-1:0] hwdata, hrdata;
    logic              hready, hresp, err_sticky;
    logic [NUM_SLAVES-1:0] hsel, s_hreadyout, s_hresp;
    logic [DATA_W-1:0]     s_hrdata [NUM_SLAVES];
    logic                  decode_err;

    ahb_master u_master (
        .hclk(clk), .hresetn(rst_n),
        .req_valid(req_valid), .req_write(req_write), .req_burst(req_burst),
        .req_channel(req_channel), .req_word(req_word), .busy(ahb_busy),
        .rd_valid(rd_valid), .rd_data(rd_data), .rd_beat(rd_beat),
        .wr_data(32'h0), .wr_ack(), .wr_beat(),
        .haddr(haddr), .hwrite(hwrite), .hsize(hsize), .hburst(hburst),
        .htrans(htrans), .hwdata(hwdata),
        .hready(hready), .hrdata(hrdata), .hresp(hresp),
        .err_sticky(err_sticky)
    );

    ahb_decoder u_decoder (
        .hclk(clk), .hresetn(rst_n),
        .haddr(haddr), .htrans(htrans),
        .m_hready(hready), .m_hrdata(hrdata), .m_hresp(hresp),
        .hsel(hsel),
        .s_hreadyout(s_hreadyout), .s_hresp(s_hresp), .s_hrdata(s_hrdata),
        .decode_err(decode_err)
    );

    logic                   sr_rd_en   [NUM_SLAVES];
    logic                   sr_wr_en   [NUM_SLAVES];
    logic [SRAM_ADDR_W-1:0] sr_rd_addr [NUM_SLAVES];
    logic [SRAM_ADDR_W-1:0] sr_wr_addr [NUM_SLAVES];
    logic [DATA_W-1:0]      sr_rd_data [NUM_SLAVES];
    logic [DATA_W-1:0]      sr_wr_data [NUM_SLAVES];
    logic [DATA_W/8-1:0]    sr_wr_be   [NUM_SLAVES];

    for (genvar i = 0; i < NUM_SLAVES; i++) begin : g_ch
        ahb_slave_sram #(.SRAM_ADDR_W(SRAM_ADDR_W)) u_slave (
            .hclk(clk), .hresetn(rst_n),
            .hsel(hsel[i]), .hready(hready),
            .haddr(haddr), .hwrite(hwrite), .hsize(hsize),
            .hburst(hburst), .htrans(htrans), .hwdata(hwdata),
            .hreadyout(s_hreadyout[i]), .hrdata(s_hrdata[i]), .hresp(s_hresp[i]),
            .sram_rd_en(sr_rd_en[i]), .sram_rd_addr(sr_rd_addr[i]),
            .sram_rd_data(sr_rd_data[i]),
            .sram_wr_en(sr_wr_en[i]), .sram_wr_be(sr_wr_be[i]),
            .sram_wr_addr(sr_wr_addr[i]), .sram_wr_data(sr_wr_data[i])
        );
        rgb_sram #(.DATA_WIDTH(DATA_W), .DEPTH(16384), .INIT_FILE(""))
        u_sram (
            .clk(clk),
            .rd_en(sr_rd_en[i]), .rd_addr(sr_rd_addr[i]), .rd_data(sr_rd_data[i]),
            .wr_en(sr_wr_en[i]), .wr_be(sr_wr_be[i]),
            .wr_addr(sr_wr_addr[i]), .wr_data(sr_wr_data[i])
        );
    end : g_ch

    // ------------------------------------------------------------------
    // Three channel FIFOs. Read side is drained by the checker.
    // ------------------------------------------------------------------
    logic [NUM_SLAVES-1:0] f_full, f_af, f_empty, f_ae, f_ecdc;
    logic [DATA_W-1:0]     f_rd [NUM_SLAVES];
    logic [NUM_SLAVES-1:0] f_pop;

    for (genvar i = 0; i < NUM_SLAVES; i++) begin : g_fifo
        async_fifo u_fifo (
            .wr_clk(clk), .wr_rst_n(rst_n),
            .wr_en(fifo_wr_en[i]), .wr_data(fifo_wr_data),
            .full(f_full[i]), .almost_full(f_af[i]),
            .rd_clk(clk), .rd_rst_n(rst_n),
            .rd_en(f_pop[i]), .rd_data(f_rd[i]),
            .empty(f_empty[i]), .almost_empty(f_ae[i]), .empty_cdc(f_ecdc[i])
        );
    end : g_fifo

    assign fifo_almost_full = f_af;

    // ------------------------------------------------------------------
    // Checker: drain each FIFO and compare against the SRAM contents
    // ------------------------------------------------------------------
    logic [DATA_W-1:0] ref_mem [NUM_SLAVES][NWORDS];
    int                got_n   [NUM_SLAVES];
    logic [DATA_W-1:0] got_w   [NUM_SLAVES][NWORDS];
    int                bad_order = 0;

    // The consumer stalls periodically. Draining flat out never lets
    // almost_full assert, so the engine's back-pressure path goes untested --
    // which is exactly how a mutation that ignores it slipped through the
    // first version of this file.
    int   drain_ctr = 0;
    logic drain_en;
    always_ff @(posedge clk) drain_ctr <= drain_ctr + 1;
    // Long stalls, not a light duty cycle. The consumer pops up to one entry
    // per cycle while the engine produces roughly one per three (four beats
    // per burst, shared round-robin across three channels), so a short stall
    // never lets occupancy climb. 128 cycles held, 128 draining, does.
    assign drain_en = drain_ctr[7];

    for (genvar i = 0; i < NUM_SLAVES; i++)
        assign f_pop[i] = !f_empty[i] && drain_en;

    // Did back-pressure actually engage, and did anything overflow?
    logic af_seen  = 1'b0;
    logic ovf_seen = 1'b0;
    always_ff @(posedge clk) if (rst_n) begin
        if (|f_af)                     af_seen  <= 1'b1;
        if (|(fifo_wr_en & f_full))    ovf_seen <= 1'b1;
    end

    logic [NUM_SLAVES-1:0] pop_q;
    always_ff @(posedge clk) pop_q <= f_pop;

    always @(posedge clk) if (rst_n) begin
        for (int i = 0; i < NUM_SLAVES; i++)
            if (pop_q[i]) begin
                if (got_n[i] < NWORDS) got_w[i][got_n[i]] <= f_rd[i];
                got_n[i] <= got_n[i] + 1;
            end
    end

    // Observers for the divergence test
    int max_skew = 0;
    always @(posedge clk) if (rst_n && rdr_busy) begin
        automatic int mn = got_n[0], mx = got_n[0];
        for (int i = 1; i < NUM_SLAVES; i++) begin
            if (got_n[i] < mn) mn = got_n[i];
            if (got_n[i] > mx) mx = got_n[i];
        end
        if (mx - mn > max_skew) max_skew <= mx - mn;
    end

    int burst_count [NUM_SLAVES];
    always @(posedge clk) if (rst_n && req_valid) burst_count[req_channel]++;

    initial begin
        // Seed the SRAMs with per-channel patterns, straight into the arrays.
        for (int c = 0; c < NUM_SLAVES; c++) begin
            for (int w = 0; w < NWORDS; w++) begin
                automatic logic [31:0] v = {8'(c), 8'(w), 8'(c*16+w), 8'(w ^ 8'hA5)};
                ref_mem[c][w] = v;
            end
            got_n[c]      = 0;
            burst_count[c] = 0;
        end
        for (int w = 0; w < NWORDS; w++) begin
            g_ch[0].u_sram.mem[w] = ref_mem[0][w];
            g_ch[1].u_sram.mem[w] = ref_mem[1][w];
            g_ch[2].u_sram.mem[w] = ref_mem[2][w];
        end

        rst_n = 1'b0; start = 1'b0;
        repeat (4) @(negedge clk);
        rst_n = 1'b1;
        repeat (2) @(negedge clk);

        $display("\n=== T1: the whole image drains ===");
        @(negedge clk); start = 1'b1;
        @(negedge clk); start = 1'b0;
        begin
            int guard = 0;
            while (!seq_done && guard < 40000) begin @(negedge clk); guard++; end
        end
        repeat (20) @(negedge clk);
        chk("engine reported done", seq_done === 1'b1 || !rdr_busy);
        chk("R got every word", got_n[0] == NWORDS);
        chk("G got every word", got_n[1] == NWORDS);
        chk("B got every word", got_n[2] == NWORDS);

        $display("\n=== T2: EQUIVALENCE -- each channel's words, in order ===");
        bad_order = -1;
        for (int c = 0; c < NUM_SLAVES; c++)
            for (int w = 0; w < NWORDS; w++)
                if (got_w[c][w] !== ref_mem[c][w] && bad_order < 0) begin
                    bad_order = c*100 + w;
                    $display("      channel %0d word %0d: got %08h expected %08h",
                             c, w, got_w[c][w], ref_mem[c][w]);
                end
        chk("every word matches, in address order", bad_order < 0);

        $display("\n=== T3: the channels DIVERGED -- bursts really are per channel ===");
        $display("      max FIFO skew during the drain: %0d entries", max_skew);
        chk("channels ran ahead of each other", max_skew > 0);

        $display("\n=== T4: burst accounting ===");
        $display("      bursts issued: R=%0d G=%0d B=%0d",
                 burst_count[0], burst_count[1], burst_count[2]);
        chk("R issued NWORDS/4 bursts", burst_count[0] == NWORDS/4);
        chk("G issued NWORDS/4 bursts", burst_count[1] == NWORDS/4);
        chk("B issued NWORDS/4 bursts", burst_count[2] == NWORDS/4);

        $display("\n=== T5: back-pressure held ===");
        chk("almost_full actually asserted (path exercised)", af_seen === 1'b1);
        chk("no FIFO was ever written while full",            ovf_seen === 1'b0);

        $display("\n=== T6: no bus errors ===");
        chk("no HRESP=ERROR",   err_sticky === 1'b0);
        chk("no decode errors", decode_err === 1'b0);

        $display("\n----------------------------------------");
        $display("checks: %0d   errors: %0d", checks, errors);
        if (errors == 0) $display("RESULT: PASS");
        else             $display("RESULT: FAIL");
        $display("----------------------------------------\n");
        $finish;
    end

    initial begin
        #4000000;
        $display("TIMEOUT -- R=%0d G=%0d B=%0d of %0d",
                 got_n[0], got_n[1], got_n[2], NWORDS);
        $finish;
    end

endmodule : tb_burst_read_path
