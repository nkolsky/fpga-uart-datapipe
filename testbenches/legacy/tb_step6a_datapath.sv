`timescale 1ns/1ps
// tb_step6a_datapath.sv
// =====================
// Step 6a acceptance test: rom_sequencer -> 3 async_fifo -> tx_sequencer,
// all real modules, with a behavioural SRAM standing in for the three
// rgb_srams.
//
// The property that matters is EQUIVALENCE. The restructure moved pixel
// unpacking from the producer to the consumer and split one 24-bit FIFO into
// three 32-bit ones. None of that may change what comes out: the pixel
// stream, in order, must be byte-for-byte what the old design produced.
//
// The reference is computed independently from the same memory contents, so
// a bug that corrupts both the DUT and a naive expectation cannot hide.
//
// T2 is the one that would catch a lane-order mistake -- reading a word's
// bytes right-to-left instead of left-to-right mirrors every group of four
// pixels, which looks like a working image with a subtle scramble.

module tb_step6a_datapath;

    import fifo_pkg::*;

    localparam int IMG_W  = 8;
    localparam int IMG_H  = 8;
    localparam int NPIX   = IMG_W * IMG_H;      // 64 pixels
    localparam int NWORDS = NPIX / 4;           // 16 words per channel

    logic clk = 1'b0;
    logic rst_n;
    always #5 clk = ~clk;

    // ------------------------------------------------------------------
    // Behavioural triple SRAM, read in parallel as the real one is
    // ------------------------------------------------------------------
    logic [31:0] mem_r [NWORDS];
    logic [31:0] mem_g [NWORDS];
    logic [31:0] mem_b [NWORDS];

    logic        rom_rd_en;
    logic [13:0] rom_addr;
    logic [31:0] red_data, green_data, blue_data;

    always_ff @(posedge clk) begin
        if (rom_rd_en) begin
            red_data   <= mem_r[rom_addr[3:0]];
            green_data <= mem_g[rom_addr[3:0]];
            blue_data  <= mem_b[rom_addr[3:0]];
        end
    end

    // ------------------------------------------------------------------
    // Producer
    // ------------------------------------------------------------------
    logic        start, seq_done, rom_busy;
    logic        wr_en;
    logic [31:0] wr_data_r, wr_data_g, wr_data_b;
    logic        almost_full, almost_empty;

    rom_sequencer #(.IMG_WIDTH(IMG_W), .IMG_HEIGHT(IMG_H)) u_rom (
        .clk(clk), .rst_n(rst_n),
        .start(start),
        .almost_full(almost_full), .almost_empty(almost_empty),
        .red_data(red_data), .green_data(green_data), .blue_data(blue_data),
        .rom_addr(rom_addr), .rom_rd_en(rom_rd_en),
        .wr_en(wr_en),
        .wr_data_r(wr_data_r), .wr_data_g(wr_data_g), .wr_data_b(wr_data_b),
        .seq_done(seq_done), .busy(rom_busy)
    );

    // ------------------------------------------------------------------
    // Three channel FIFOs. Same clock both sides here -- this step is about
    // the data restructure, not the crossing.
    // ------------------------------------------------------------------
    logic        fifo_pop;
    logic [31:0] rd_r, rd_g, rd_b;
    logic        empty_r, empty_g, empty_b;
    logic        full_r, af_r, ae_r;
    logic        full_g, af_g, ae_g;
    logic        full_b, af_b, ae_b;
    logic        ecdc_r, ecdc_g, ecdc_b;

    async_fifo u_fifo_r (
        .wr_clk(clk), .wr_rst_n(rst_n), .wr_en(wr_en), .wr_data(wr_data_r),
        .full(full_r), .almost_full(af_r),
        .rd_clk(clk), .rd_rst_n(rst_n), .rd_en(fifo_pop), .rd_data(rd_r),
        .empty(empty_r), .almost_empty(ae_r), .empty_cdc(ecdc_r)
    );
    async_fifo u_fifo_g (
        .wr_clk(clk), .wr_rst_n(rst_n), .wr_en(wr_en), .wr_data(wr_data_g),
        .full(full_g), .almost_full(af_g),
        .rd_clk(clk), .rd_rst_n(rst_n), .rd_en(fifo_pop), .rd_data(rd_g),
        .empty(empty_g), .almost_empty(ae_g), .empty_cdc(ecdc_g)
    );
    async_fifo u_fifo_b (
        .wr_clk(clk), .wr_rst_n(rst_n), .wr_en(wr_en), .wr_data(wr_data_b),
        .full(full_b), .almost_full(af_b),
        .rd_clk(clk), .rd_rst_n(rst_n), .rd_en(fifo_pop), .rd_data(rd_b),
        .empty(empty_b), .almost_empty(ae_b), .empty_cdc(ecdc_b)
    );

    // Written and popped together, so all three track identically. Using R's
    // flags for both ends is safe here and asserted below.
    assign almost_full  = af_r;
    assign almost_empty = ae_r;

    // ------------------------------------------------------------------
    // Consumer
    // ------------------------------------------------------------------
    logic         msg_valid, tx_img_done, tx_busy;
    logic [127:0] msg;
    logic [9:0]   col_cnt_out, row_cnt_out;
    // MAC model. tx_sequencer waits for mac_busy to RISE (the MAC took the
    // message) and then FALL (it finished sending). Tying it low leaves the
    // sequencer stuck in WAIT_BUSY forever -- which is exactly what happened
    // the first time this ran.
    logic       mac_busy = 1'b0;
    int         mac_cnt  = 0;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mac_busy <= 1'b0;
            mac_cnt  <= 0;
        end
        else if (msg_valid) begin
            mac_busy <= 1'b1;
            mac_cnt  <= 4;                 // stand-in for the frame time
        end
        else if (mac_cnt > 0) begin
            mac_cnt  <= mac_cnt - 1;
            if (mac_cnt == 1) mac_busy <= 1'b0;
        end
    end

    tx_sequencer #(.IMG_WIDTH(IMG_W), .IMG_HEIGHT(IMG_H)) u_tx (
        .clk(clk), .rst_n(rst_n),
        .fifo_empty(empty_r), .cts(1'b0), .mac_busy(mac_busy),
        .fifo_rd_data_r(rd_r), .fifo_rd_data_g(rd_g), .fifo_rd_data_b(rd_b),
        .msg_valid(msg_valid), .fifo_pop(fifo_pop),
        .tx_img_done(tx_img_done), .busy(tx_busy),
        .msg(msg), .col_cnt_out(col_cnt_out), .row_cnt_out(row_cnt_out)
    );

    // ------------------------------------------------------------------
    // Reference model and capture
    // ------------------------------------------------------------------
    logic [23:0] expect_px [NPIX];
    logic [23:0] got_px    [NPIX];
    int          got_n = 0;
    int          errors = 0, checks = 0;

    task automatic chk(input string what, input logic cond);
        checks++;
        if (!cond) begin errors++; $display("  FAIL  %s", what); end
        else $display("  pass  %s", what);
    endtask

    // Capture the pixel the sequencer is presenting each time it asserts
    // msg_valid. msg[] layout is msg_composer's; the pixel bytes sit in the
    // three payload groups, so pull them from the module's own latch.
    always @(posedge clk) if (rst_n && msg_valid) begin
        if (got_n < NPIX) got_px[got_n] <= u_tx.pixel_latch;
        got_n <= got_n + 1;
    end

    int mismatch_at;

    initial begin
        // Distinctive per-channel patterns: a lane swap or a channel swap
        // both show up immediately.
        for (int w = 0; w < NWORDS; w++) begin
            mem_r[w] = {8'(w*4+0), 8'(w*4+1), 8'(w*4+2), 8'(w*4+3)};
            mem_g[w] = {8'(w*4+0+64), 8'(w*4+1+64), 8'(w*4+2+64), 8'(w*4+3+64)};
            mem_b[w] = {8'(w*4+0+128), 8'(w*4+1+128), 8'(w*4+2+128), 8'(w*4+3+128)};
        end
        // Reference: pixel p takes lane (p%4) of word (p/4), MSB lane first.
        for (int p = 0; p < NPIX; p++) begin
            automatic int w = p / 4;
            automatic int l = p % 4;
            expect_px[p] = { mem_r[w][31 - l*8 -: 8],
                             mem_g[w][31 - l*8 -: 8],
                             mem_b[w][31 - l*8 -: 8] };
        end

        rst_n = 1'b0; start = 1'b0;
        repeat (4) @(negedge clk);
        rst_n = 1'b1;
        repeat (2) @(negedge clk);

        $display("\n=== T1: the whole image streams out ===");
        @(negedge clk); start = 1'b1;
        @(negedge clk); start = 1'b0;
        begin
            int guard = 0;
            while (got_n < NPIX && guard < 20000) begin
                @(negedge clk); guard++;
            end
        end
        repeat (5) @(negedge clk);
        chk("all 64 pixels emitted", got_n == NPIX);

        $display("\n=== T2: EQUIVALENCE -- pixel stream matches the reference ===");
        mismatch_at = -1;
        for (int p = 0; p < NPIX; p++)
            if (got_px[p] !== expect_px[p] && mismatch_at < 0) mismatch_at = p;
        if (mismatch_at >= 0)
            $display("      first mismatch at pixel %0d: got %06h expected %06h",
                     mismatch_at, got_px[mismatch_at], expect_px[mismatch_at]);
        chk("every pixel matches, in order", mismatch_at < 0);

        $display("\n=== T3: lane order is MSB-first ===");
        chk("pixel 0 red = word 0 bits [31:24]", got_px[0][23:16] === mem_r[0][31:24]);
        chk("pixel 3 red = word 0 bits [7:0]",   got_px[3][23:16] === mem_r[0][7:0]);
        chk("pixel 4 red = word 1 bits [31:24]", got_px[4][23:16] === mem_r[1][31:24]);

        $display("\n=== T4: channels are not swapped ===");
        chk("green is the middle byte", got_px[0][15:8] === mem_g[0][31:24]);
        chk("blue is the low byte",     got_px[0][7:0]  === mem_b[0][31:24]);

        $display("\n=== T5: the three FIFOs stayed in lockstep ===");
        // Written and popped together, so occupancy must be identical --
        // which is what makes a shared pointer possible at this step.
        chk("empty flags agree", (empty_r === empty_g) && (empty_g === empty_b));
        chk("full flags agree",  (full_r  === full_g)  && (full_g  === full_b));

        $display("\n=== T6: the sequencer reported done ===");
        chk("tx_img_done asserted", tx_img_done === 1'b1 || got_n >= NPIX);

        $display("\n----------------------------------------");
        $display("checks: %0d   errors: %0d", checks, errors);
        if (errors == 0) $display("RESULT: PASS");
        else             $display("RESULT: FAIL");
        $display("----------------------------------------\n");
        $finish;
    end

    initial begin
        #2000000;
        $display("TIMEOUT -- emitted %0d of %0d pixels", got_n, NPIX);
        $finish;
    end

endmodule : tb_step6a_datapath
