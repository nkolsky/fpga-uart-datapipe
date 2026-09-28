// tb_pixel_word_packer.sv
// =======================
// Checks the packer turns pixel streams into the RIGHT masked word writes:
// aligned bursts in one access, unaligned ones split correctly, and narrow
// rectangles flushed at every row end.
//
// Build: TOP = tb_pixel_word_packer
//        SRCS = memory_pkg.sv pixel_word_packer.sv tb_pixel_word_packer.sv
// -DSIMULATION gives 8x8 geometry: 64 pixels, 16 words.

`timescale 1ns/1ps

module tb_pixel_word_packer;

    import memory_pkg::*;

    localparam int ADDR_W = SRAM_ADDR_WIDTH;
    localparam int DATA_W = SRAM_DATA_WIDTH;
    localparam int NLANE  = PIXELS_PER_WORD;

    typedef enum logic [2:0] {
        P_INIT      = 3'd0,
        P_SINGLE    = 3'd1,
        P_ALIGNED   = 3'd2,
        P_UNALIGNED = 3'd3,
        P_RECT      = 3'd4,
        P_NARROW    = 3'd5,
        P_CARRY     = 3'd6,
        P_DONE      = 3'd7
    } tb_phase_e;

    tb_phase_e phase = P_INIT;

    logic clk = 1'b0, rst_n = 1'b0;
    always #5 clk = ~clk;

    logic        start = 1'b0;
    logic [23:0] base_addr = '0;
    logic [9:0]  height = '0, width = '0;

    logic        pixel_valid = 1'b0, pixel_ready;
    logic [7:0]  pixel_r = '0, pixel_g = '0, pixel_b = '0;

    logic              wr_valid;
    logic              wr_ready = 1'b1;
    logic [ADDR_W-1:0] wr_addr;
    logic [NLANE-1:0]  wr_be;
    logic [DATA_W-1:0] wr_data_r, wr_data_g, wr_data_b;
    logic              busy, done;

    pixel_word_packer u_pack (
        .clk(clk), .rst_n(rst_n),
        .start(start), .base_addr(base_addr), .height(height), .width(width),
        .pixel_valid(pixel_valid), .pixel_ready(pixel_ready),
        .pixel_r(pixel_r), .pixel_g(pixel_g), .pixel_b(pixel_b),
        .wr_valid(wr_valid), .wr_ready(wr_ready),
        .wr_addr(wr_addr), .wr_be(wr_be),
        .wr_data_r(wr_data_r), .wr_data_g(wr_data_g), .wr_data_b(wr_data_b),
        .busy(busy), .done(done)
    );

    // -------------------------------------------------------------------
    // Collect every issued write
    // -------------------------------------------------------------------
    int                n_wr;
    logic [ADDR_W-1:0] wa_q  [$];
    logic [NLANE-1:0]  be_q  [$];
    logic [DATA_W-1:0] dr_q  [$];

    always @(posedge clk) if (rst_n && wr_valid && wr_ready) begin
        n_wr++;
        wa_q.push_back(wr_addr);
        be_q.push_back(wr_be);
        dr_q.push_back(wr_data_r);
    end

    // Pixels the packer actually took. A burst data message always carries
    // four pixels, so when H*W is not a multiple of four the final message
    // has padding that must be REFUSED, not written.
    int n_pix_taken;
    always @(posedge clk) if (rst_n && pixel_valid && pixel_ready) n_pix_taken++;

    task automatic clear_wr();
        n_wr = 0; n_pix_taken = 0; wa_q.delete(); be_q.delete(); dr_q.delete();
    endtask

    int checks, fails;

    task automatic ck(input string what, input int got, input int exp);
        checks++;
        if (got !== exp) begin
            $display("  FAIL [%s] %-24s got=%0d exp=%0d",
                     phase.name(), what, got, exp);
            fails++;
        end
    endtask

    task automatic ckb(input string what, input logic [31:0] got,
                                          input logic [31:0] exp);
        checks++;
        if (got !== exp) begin
            $display("  FAIL [%s] %-24s got=%b exp=%b",
                     phase.name(), what, got, exp);
            fails++;
        end
    endtask

    task automatic ckh(input string what, input logic [31:0] got,
                                          input logic [31:0] exp);
        checks++;
        if (got !== exp) begin
            $display("  FAIL [%s] %-24s got=%h exp=%h",
                     phase.name(), what, got, exp);
            fails++;
        end
    endtask

    // -------------------------------------------------------------------
    // Stimulus helpers
    // -------------------------------------------------------------------
    task automatic begin_rect(input int base, input int h, input int w);
        @(negedge clk);
        base_addr = 24'(base); height = 10'(h); width = 10'(w);
        start = 1'b1;
        @(negedge clk);
        start = 1'b0;
    endtask

    // Drive on the negative edge, let the positive edge take it. Checking
    // ready AFTER the transfer is wrong: for a 1x1 rectangle busy drops on
    // the same edge that accepts the pixel, so ready is already gone.
    task automatic send_pixel(input logic [7:0] r, g, b);
        @(negedge clk);
        pixel_r = r; pixel_g = g; pixel_b = b;
        pixel_valid = 1'b1;
        while (!pixel_ready) @(negedge clk);   // ready is stable at negedge
        @(posedge clk);                        // transfer happens here
        @(negedge clk);
        pixel_valid = 1'b0;
    endtask

    // Deliver one burst data message: up to four pixels, stopping as soon
    // as the rectangle is complete. This is exactly what the real feeder
    // must do -- blindly pushing four pixels would hang on the padding.
    task automatic send_msg4(input logic [7:0] p0, p1, p2, p3);
        logic [7:0] v [4];
        v = '{p0, p1, p2, p3};
        for (int i = 0; i < 4; i++) begin
            if (!busy) break;
            send_pixel(v[i], 8'h00, 8'h00);
        end
    endtask

    task automatic settle();
        repeat (6) @(negedge clk);
    endtask

    task automatic show();
        $display("    %0d write(s):", n_wr);
        for (int i = 0; i < n_wr; i++)
            $display("        word=%0d be=%b data_r=%h", wa_q[i], be_q[i], dr_q[i]);
    endtask

    // -------------------------------------------------------------------
    initial begin
        $dumpfile("tb_pixel_word_packer_wv.fst");
        $dumpvars(0, tb_pixel_word_packer);

        checks = 0; fails = 0;
        clear_wr();

        repeat (4) @(negedge clk);
        rst_n = 1'b1;
        repeat (2) @(negedge clk);

        $display("");
        $display("geometry %0dx%0d  words=%0d  lanes/word=%0d",
                 IMG_WIDTH, IMG_HEIGHT, SRAM_DEPTH, NLANE);

        // ===============================================================
        phase = P_SINGLE;
        // A single pixel write is a 1x1 rectangle. Pixel 6 -> word 1,
        // lane 2 -> be bit 1.
        // ===============================================================
        clear_wr();
        begin_rect(6, 1, 1);
        send_pixel(8'h11, 8'h22, 8'h33);
        settle();
        $display("  single pixel at linear 6:");
        show();
        ck ("writes",  n_wr, 1);
        ck ("word",    int'(wa_q[0]), 1);
        ckb("be",      32'(be_q[0]), 32'b0010);
        ckh("data_r",  32'(dr_q[0] & 32'h0000_FF00), 32'h0000_1100);

        // ===============================================================
        phase = P_ALIGNED;
        // Four pixels from linear 4 -> exactly word 1. ONE write, be=1111.
        // This is the case the old design split into four writes.
        // ===============================================================
        clear_wr();
        begin_rect(4, 1, 4);
        send_pixel(8'hA0, 8'hB0, 8'hC0);
        send_pixel(8'hA1, 8'hB1, 8'hC1);
        send_pixel(8'hA2, 8'hB2, 8'hC2);
        send_pixel(8'hA3, 8'hB3, 8'hC3);
        settle();
        $display("  aligned 4 pixels from linear 4:");
        show();
        ck ("writes", n_wr, 1);
        ck ("word",   int'(wa_q[0]), 1);
        ckb("be",     32'(be_q[0]), 32'b1111);
        ckh("data_r", 32'(dr_q[0]), 32'hA0A1A2A3);

        // ===============================================================
        phase = P_UNALIGNED;
        // Four pixels from linear 5 straddle words 1 and 2:
        //   linear 5,6,7 -> word 1 lanes 1,2,3 -> be 0111
        //   linear 8     -> word 2 lane  0     -> be 1000
        // ===============================================================
        clear_wr();
        begin_rect(5, 1, 4);
        send_pixel(8'hD0, 8'h00, 8'h00);
        send_pixel(8'hD1, 8'h00, 8'h00);
        send_pixel(8'hD2, 8'h00, 8'h00);
        send_pixel(8'hD3, 8'h00, 8'h00);
        settle();
        $display("  unaligned 4 pixels from linear 5:");
        show();
        ck ("writes",   n_wr, 2);
        ck ("word 1st", int'(wa_q[0]), 1);
        ckb("be 1st",   32'(be_q[0]), 32'b0111);
        ck ("word 2nd", int'(wa_q[1]), 2);
        ckb("be 2nd",   32'(be_q[1]), 32'b1000);

        // ===============================================================
        phase = P_RECT;
        // 2 rows x 4 cols at the origin of an 8-wide image.
        //   row 0 -> linear 0..3  -> word 0, be 1111
        //   row 1 -> linear 8..11 -> word 2, be 1111
        // The address JUMPS between rows. Without a row-end flush this
        // would run 0..7 and corrupt word 1.
        // ===============================================================
        clear_wr();
        begin_rect(0, 2, 4);
        for (int i = 0; i < 8; i++)
            send_pixel(8'hE0 + 8'(i), 8'h00, 8'h00);
        settle();
        $display("  2x4 rectangle at origin:");
        show();
        ck ("writes",   n_wr, 2);
        ck ("word row0", int'(wa_q[0]), 0);
        ckb("be row0",   32'(be_q[0]), 32'b1111);
        ckh("data row0", 32'(dr_q[0]), 32'hE0E1E2E3);
        ck ("word row1", int'(wa_q[1]), 2);
        ckb("be row1",   32'(be_q[1]), 32'b1111);
        ckh("data row1", 32'(dr_q[1]), 32'hE4E5E6E7);

        // ===============================================================
        phase = P_NARROW;
        // 2 rows x 2 cols starting at linear 1 (row 0, col 1).
        //   row 0 -> linear 1,2  -> word 0, be 0110
        //   row 1 -> linear 9,10 -> word 2, be 0110
        // Two pixels per row, so each row flushes a PARTIAL word.
        // ===============================================================
        clear_wr();
        begin_rect(1, 2, 2);
        for (int i = 0; i < 4; i++)
            send_pixel(8'hF0 + 8'(i), 8'h00, 8'h00);
        settle();
        $display("  2x2 rectangle at linear 1:");
        show();
        ck ("writes",   n_wr, 2);
        ck ("word row0", int'(wa_q[0]), 0);
        ckb("be row0",   32'(be_q[0]), 32'b0110);
        ck ("word row1", int'(wa_q[1]), 2);
        ckb("be row1",   32'(be_q[1]), 32'b0110);

        // ===============================================================
        phase = P_CARRY;
        // 1 x 6 rectangle from linear 2, delivered as TWO messages of four
        // pixels. The interesting part is the message boundary:
        //
        //   msg 1: linear 2,3   -> word 0 lanes 2,3 -> flush, be=0011
        //          linear 4,5   -> word 1 lanes 0,1 -> HELD in the
        //                          accumulator when the message ends
        //   msg 2: linear 6,7   -> word 1 lanes 2,3 -> flush, be=1111
        //          + 2 padding  -> REFUSED, rectangle already complete
        //
        // The accumulator belongs to the RECTANGLE, not the message. If it
        // were cleared per message, pixels 4 and 5 would be lost and word 1
        // would come out as be=0011 with two dead lanes.
        // ===============================================================
        clear_wr();
        begin_rect(2, 1, 6);
        send_msg4(8'h50, 8'h51, 8'h52, 8'h53);
        repeat (4) @(negedge clk);          // gap between messages
        send_msg4(8'h54, 8'h55, 8'hEE, 8'hEE);   // last two are padding
        settle();
        $display("  1x6 from linear 2, two messages:");
        show();
        ck ("pixels taken", n_pix_taken, 6);
        ck ("writes",       n_wr, 2);
        ck ("word 1st",     int'(wa_q[0]), 0);
        ckb("be 1st",       32'(be_q[0]), 32'b0011);
        ckh("data 1st",     32'(dr_q[0]), 32'h00005051);
        ck ("word 2nd",     int'(wa_q[1]), 1);
        ckb("be 2nd",       32'(be_q[1]), 32'b1111);
        ckh("data 2nd",     32'(dr_q[1]), 32'h52535455);

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
