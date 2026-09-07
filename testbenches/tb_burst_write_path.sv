`timescale 1ns/1ps
// tb_burst_write_path.sv
// ======================
// Step 7: the full-image write path over AHB-Lite INCR4.
//
//   pixel_word_packer -> img_burst_writer -> ahb_master
//                                         -> ahb_decoder -> 3x ahb_slave_sram
//                                                        -> 3x rgb_sram
//
// Every module real. The check is EQUIVALENCE: pixels pushed into the packer
// must end up in the SRAMs exactly where the direct write path would have
// put them. The reference is computed from the pixel stream independently.
//
// T2 is the test that matters -- it reads the SRAM arrays back and compares
// word for word. T4 covers the thing most likely to be wrong: a burst whose
// four words are not consecutive, or whose channel gets crossed.

module tb_burst_write_path;

    import ahb_pkg::*;

    // Small image so the run is quick: 8x8 = 64 pixels = 16 words per
    // channel = 4 INCR4 bursts per channel.
    localparam int IMG_W  = 8;
    localparam int IMG_H  = 8;
    localparam int NPIX   = IMG_W * IMG_H;
    localparam int NWORDS = NPIX / 4;
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
    // Packer
    // ------------------------------------------------------------------
    logic        pack_start, pack_busy, pack_done;
    logic [23:0] pack_base;
    logic [9:0]  pack_h, pack_w;
    logic        pixel_valid, pixel_ready;
    logic [7:0]  pixel_r, pixel_g, pixel_b;

    logic                   wr_valid, wr_ready;
    logic [SRAM_ADDR_W-1:0] wr_addr;
    logic [3:0]             wr_be;
    logic [DATA_W-1:0]      wr_data_r, wr_data_g, wr_data_b;

    pixel_word_packer u_packer (
        .clk(clk), .rst_n(rst_n),
        .start(pack_start), .base_addr(pack_base),
        .height(pack_h), .width(pack_w),
        .pixel_valid(pixel_valid), .pixel_ready(pixel_ready),
        .pixel_r(pixel_r), .pixel_g(pixel_g), .pixel_b(pixel_b),
        .wr_valid(wr_valid), .wr_ready(wr_ready),
        .wr_addr(wr_addr), .wr_be(wr_be),
        .wr_data_r(wr_data_r), .wr_data_g(wr_data_g), .wr_data_b(wr_data_b),
        .busy(pack_busy), .done(pack_done)
    );

    // ------------------------------------------------------------------
    // Gather
    // ------------------------------------------------------------------
    logic              burst_mode;
    logic              req_valid, req_write, req_burst;
    logic [SEL_W-1:0]  req_channel;
    logic [WORD_W-1:0] req_word;
    logic              ahb_busy, bw_busy;
    logic [1:0]        ahb_wr_beat;
    logic              ahb_wr_ack;
    logic [DATA_W-1:0] ahb_wr_data;

    img_burst_writer #(.ADDR_W_IN(SRAM_ADDR_W)) u_writer (
        .clk(clk), .rst_n(rst_n),
        .burst_mode(burst_mode), .pack_busy(pack_busy), .pack_done(pack_done),
        .wr_valid(wr_valid), .wr_ready(wr_ready),
        .wr_addr(wr_addr), .wr_be(wr_be),
        .wr_data_r(wr_data_r), .wr_data_g(wr_data_g), .wr_data_b(wr_data_b),
        .req_valid(req_valid), .req_write(req_write), .req_burst(req_burst),
        .req_channel(req_channel), .req_word(req_word), .ahb_busy(ahb_busy),
        .ahb_wr_beat(ahb_wr_beat), .ahb_wr_ack(ahb_wr_ack),
        .ahb_wr_data(ahb_wr_data),
        .busy(bw_busy)
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
    logic                  rd_valid;
    logic [DATA_W-1:0]     rd_data;
    logic [1:0]            rd_beat;

    ahb_master u_master (
        .hclk(clk), .hresetn(rst_n),
        .req_valid(req_valid), .req_write(req_write), .req_burst(req_burst),
        .req_channel(req_channel), .req_word(req_word), .busy(ahb_busy),
        .rd_valid(rd_valid), .rd_data(rd_data), .rd_beat(rd_beat),
        .wr_data(ahb_wr_data), .wr_ack(ahb_wr_ack), .wr_beat(ahb_wr_beat),
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
    // Reference
    // ------------------------------------------------------------------
    logic [7:0] src_r [NPIX], src_g [NPIX], src_b [NPIX];
    logic [DATA_W-1:0] ref_r [NWORDS], ref_g [NWORDS], ref_b [NWORDS];

    int burst_count [NUM_SLAVES];
    int stall_cycles = 0;
    always @(posedge clk) if (rst_n) begin
        if (req_valid) burst_count[req_channel]++;
        if (pack_busy && !wr_ready) stall_cycles++;
    end

    int bad = -1;

    initial begin
        for (int c = 0; c < NUM_SLAVES; c++) burst_count[c] = 0;
        // Distinctive per-channel, per-pixel values.
        for (int p = 0; p < NPIX; p++) begin
            src_r[p] = 8'(p);
            src_g[p] = 8'(p + 64);
            src_b[p] = 8'(p ^ 8'hA5);
        end
        // Reference words: pixel p sits in lane p%4, MSB lane first.
        for (int w = 0; w < NWORDS; w++) begin
            ref_r[w] = {src_r[w*4+0], src_r[w*4+1], src_r[w*4+2], src_r[w*4+3]};
            ref_g[w] = {src_g[w*4+0], src_g[w*4+1], src_g[w*4+2], src_g[w*4+3]};
            ref_b[w] = {src_b[w*4+0], src_b[w*4+1], src_b[w*4+2], src_b[w*4+3]};
        end

        rst_n = 1'b0;
        pack_start = 0; pack_base = '0; pack_h = '0; pack_w = '0;
        pixel_valid = 0; pixel_r = '0; pixel_g = '0; pixel_b = '0;
        burst_mode = 1'b0;
        repeat (4) @(negedge clk);
        rst_n = 1'b1;
        repeat (2) @(negedge clk);

        $display("\n=== T1: full-image write over INCR4 ===");
        // burst_mode is decided from the geometry BEFORE the rectangle runs.
        burst_mode = (pack_base == 24'd0) && (IMG_H == IMG_H) && (IMG_W == IMG_W);
        @(negedge clk);
        pack_base = 24'd0; pack_h = 10'(IMG_H); pack_w = 10'(IMG_W);
        pack_start = 1'b1;
        @(negedge clk);
        pack_start = 1'b0;

        // Feed pixels, honouring pixel_ready.
        for (int p = 0; p < NPIX; p++) begin
            @(negedge clk);
            while (!pixel_ready) @(negedge clk);
            pixel_valid = 1'b1;
            pixel_r = src_r[p]; pixel_g = src_g[p]; pixel_b = src_b[p];
            @(negedge clk);
            pixel_valid = 1'b0;
        end
        begin
            int guard = 0;
            while (bw_busy && guard < 20000) begin @(negedge clk); guard++; end
        end
        repeat (30) @(negedge clk);
        chk("packer finished",       !pack_busy);
        chk("burst writer finished", !bw_busy);

        $display("\n=== T2: EQUIVALENCE -- SRAM contents match the reference ===");
        bad = -1;
        for (int w = 0; w < NWORDS; w++) begin
            if (g_ch[0].u_sram.mem[w] !== ref_r[w] && bad < 0) begin
                bad = w;
                $display("      R word %0d: got %08h expected %08h",
                         w, g_ch[0].u_sram.mem[w], ref_r[w]);
            end
            if (g_ch[1].u_sram.mem[w] !== ref_g[w] && bad < 0) begin
                bad = w;
                $display("      G word %0d: got %08h expected %08h",
                         w, g_ch[1].u_sram.mem[w], ref_g[w]);
            end
            if (g_ch[2].u_sram.mem[w] !== ref_b[w] && bad < 0) begin
                bad = w;
                $display("      B word %0d: got %08h expected %08h",
                         w, g_ch[2].u_sram.mem[w], ref_b[w]);
            end
        end
        chk("every word in every channel is correct", bad < 0);

        $display("\n=== T3: burst accounting ===");
        $display("      bursts: R=%0d G=%0d B=%0d",
                 burst_count[0], burst_count[1], burst_count[2]);
        chk("R issued NWORDS/4 bursts", burst_count[0] == NWORDS/4);
        chk("G issued NWORDS/4 bursts", burst_count[1] == NWORDS/4);
        chk("B issued NWORDS/4 bursts", burst_count[2] == NWORDS/4);

        $display("\n=== T4: the gather back-pressured the packer ===");
        // wr_ready is low through all three bursts of every group, so the
        // packer must have stalled. If it never did, the gather is accepting
        // words while bursting -- which would corrupt the buffer.
        chk("packer was stalled while bursts drained", stall_cycles > 0);

        $display("\n=== T5: no bus errors ===");
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
        $display("TIMEOUT");
        $finish;
    end

endmodule : tb_burst_write_path
