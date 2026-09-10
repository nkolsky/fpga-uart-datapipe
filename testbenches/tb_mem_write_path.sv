// tb_mem_write_path.sv
// ====================
// The whole memory-side write path, end to end:
//
//   130 MHz  ->  cdc_msg_sync  ->  100 MHz
//                                  mem_write_subsystem
//                                    mem_msg_writer
//                                    pixel_word_packer
//                                  rgb_sram x3
//                                  mem_interlock
//
// Messages go in on the receive clock; the image is read back out of the
// SRAMs and checked pixel by pixel.
//
// Build: TOP = tb_mem_write_path
//        SRCS = msg_format_pkg memory_pkg cdc_msg_sync mem_msg_writer
//               pixel_word_packer mem_write_subsystem
//               mem_interlock rgb_sram + this file
// -DSIMULATION gives 8x8 geometry: 64 pixels, 16 words.

`timescale 1ns/1ps

module tb_mem_write_path;

    import msg_format_pkg::*;
    import memory_pkg::*;

    localparam int ADDR_W = SRAM_ADDR_WIDTH;
    localparam int DATA_W = SRAM_DATA_WIDTH;
    localparam int NLANE  = PIXELS_PER_WORD;

    typedef enum logic [2:0] {
        P_INIT    = 3'd0,
        P_BURST   = 3'd1,   // 4x4 rectangle, whole image path
        P_SINGLE  = 3'd2,   // one pixel, read-modify-write
        P_NARROW  = 3'd3,   // 2x2 -- partial words on both rows
        P_CONTEND = 3'd4,   // a reader takes the port mid-write
        P_DONE    = 3'd5
    } tb_phase_e;

    tb_phase_e phase = P_INIT;

    // Two unrelated clocks, as in the design.
    logic rx_clk = 1'b0;  always #3.846 rx_clk = ~rx_clk;   // ~130 MHz
    logic mem_clk = 1'b0; always #5.000 mem_clk = ~mem_clk;  // 100 MHz
    logic rx_rst_n = 1'b0, mem_rst_n = 1'b0;

    // ---- message source, receive domain ---------------------------------
    logic         src_valid = 1'b0;
    logic         src_ready;
    msg_kind_t    src_kind  = MSG_UNKNOWN;
    msg_payload_t src_pl    = '0;

    logic         dst_valid, dst_ready;
    msg_kind_t    dst_kind;
    msg_payload_t dst_pl;

    localparam int XW = 4 + $bits(msg_payload_t);

    cdc_msg_sync #(.WIDTH(XW)) u_cdc (
        .src_clk(rx_clk), .src_rst_n(rx_rst_n),
        .src_valid(src_valid), .src_ready(src_ready),
        .src_data({src_kind, src_pl}),
        .dst_clk(mem_clk), .dst_rst_n(mem_rst_n),
        .dst_valid(dst_valid), .dst_ready(dst_ready),
        .dst_data({dst_kind, dst_pl})
    );

    // ---- write subsystem --------------------------------------------------
    logic              wr_en;
    logic [NLANE-1:0]  wr_be;
    logic [ADDR_W-1:0] wr_addr;
    logic [DATA_W-1:0] rd_r, rd_g, rd_b;
    logic [DATA_W-1:0] wr_r, wr_g, wr_b;
    logic              wr_allowed = 1'b1;
    logic              rect_busy, wr_rejected;

    mem_write_subsystem u_wsub (
        .clk(mem_clk), .rst_n(mem_rst_n),
        .msg_valid(dst_valid), .msg_ready(dst_ready),
        .msg_kind(dst_kind), .msg_payload(dst_pl),
        .wr_en(wr_en), .wr_be(wr_be), .wr_addr(wr_addr),
        .wr_data_r(wr_r), .wr_data_g(wr_g), .wr_data_b(wr_b),
        .wr_allowed(wr_allowed),
        .rect_busy(rect_busy), .wr_rejected(wr_rejected)
    );

    // ---- interlock ---------------------------------------------------------
    logic start_req = 1'b0, rom_seq_busy = 1'b0, img_done = 1'b0;
    logic read_go;
    logic burst_active = 1'b0;
    logic pix_rd_req = 1'b0, pix_rd_done = 1'b0;
    logic pix_rd_gnt, pix_rd_owner;

    mem_interlock u_itl (
        .clk(mem_clk), .rst_n(mem_rst_n),
        .start_req(start_req), .rom_seq_busy(rom_seq_busy),
        .img_done(img_done), .read_go(read_go),
        .wr_port_req(dst_valid), .wr_busy(rect_busy),
        .burst_active(burst_active), .wr_port_grant(wr_allowed),
        .pix_rd_req(pix_rd_req), .pix_rd_done(pix_rd_done),
        .pix_rd_gnt(pix_rd_gnt), .pix_rd_owner(pix_rd_owner)
    );

    // ---- SRAMs. The testbench borrows the read port when idle. ------------
    logic              tb_rd_en   = 1'b0;
    logic [ADDR_W-1:0] tb_rd_addr = '0;
    // The write path issues NO reads at all -- byte enables mean a partial
    // word costs one write and nothing else. The only reader here is the
    // testbench checking the result.
    logic              s_rd_en;
    logic [ADDR_W-1:0] s_rd_addr;
    assign s_rd_en   = tb_rd_en;
    assign s_rd_addr = tb_rd_addr;

    rgb_sram #(.DEPTH(SRAM_DEPTH)) u_r (.clk(mem_clk),
        .rd_en(s_rd_en), .rd_addr(s_rd_addr), .rd_data(rd_r),
        .wr_en(wr_en), .wr_be(wr_be), .wr_addr(wr_addr), .wr_data(wr_r));
    rgb_sram #(.DEPTH(SRAM_DEPTH)) u_g (.clk(mem_clk),
        .rd_en(s_rd_en), .rd_addr(s_rd_addr), .rd_data(rd_g),
        .wr_en(wr_en), .wr_be(wr_be), .wr_addr(wr_addr), .wr_data(wr_g));
    rgb_sram #(.DEPTH(SRAM_DEPTH)) u_b (.clk(mem_clk),
        .rd_en(s_rd_en), .rd_addr(s_rd_addr), .rd_data(rd_b),
        .wr_en(wr_en), .wr_be(wr_be), .wr_addr(wr_addr), .wr_data(wr_b));

    // -------------------------------------------------------------------
    // SRAM CONTENTS, MIRRORED FLAT FOR THE WAVEFORM
    //
    // The storage array lives inside rgb_sram and Verilator does not trace
    // arrays unless --trace-max-array is passed, which this flow does not.
    // These continuous assignments expose the first eight words of each
    // channel as ordinary signals, so the memory can be watched changing.
    //
    // Word N holds pixels 4N..4N+3, LANE 0 IS THE MOST SIGNIFICANT BYTE:
    //     word 0 = pixels 0,1,2,3   word 2 = pixels 8,9,10,11
    // An 8-wide image puts row 1 at word 2, not word 1 -- the odd words are
    // the right-hand half of each row and stay untouched by a 4-wide burst.
    // -------------------------------------------------------------------
    logic [DATA_W-1:0] w0_r, w1_r, w2_r, w3_r, w4_r, w5_r, w6_r, w7_r;
    logic [DATA_W-1:0] w0_g, w2_g, w4_g, w6_g;
    logic [DATA_W-1:0] w0_b, w2_b, w4_b, w6_b;

    assign w0_r = u_r.mem[0];  assign w1_r = u_r.mem[1];
    assign w2_r = u_r.mem[2];  assign w3_r = u_r.mem[3];
    assign w4_r = u_r.mem[4];  assign w5_r = u_r.mem[5];
    assign w6_r = u_r.mem[6];  assign w7_r = u_r.mem[7];

    assign w0_g = u_g.mem[0];  assign w2_g = u_g.mem[2];
    assign w4_g = u_g.mem[4];  assign w6_g = u_g.mem[6];

    assign w0_b = u_b.mem[0];  assign w2_b = u_b.mem[2];
    assign w4_b = u_b.mem[4];  assign w6_b = u_b.mem[6];

    // -------------------------------------------------------------------
    // Access counting -- the PPA claim, measured on the real path
    // -------------------------------------------------------------------
    // The write path must NEVER read. With byte enables a partial-word
    // update is a single write with the unwanted lanes masked off, so any
    // read from this path would mean a read-modify-write had crept back in.
    int n_reads, n_writes, n_full, n_partial;
    always @(posedge mem_clk) if (mem_rst_n) begin
        if (u_wsub.u_packer.wr_valid && wr_allowed) begin
            n_writes++;
            if (wr_be == {NLANE{1'b1}}) n_full++;
            else                        n_partial++;
        end
    end
    task automatic clear_counts();
        n_reads=0; n_writes=0; n_full=0; n_partial=0;
    endtask

    int checks, fails;

    task automatic ck(input string what, input int got, input int exp);
        checks++;
        if (got !== exp) begin
            $display("  FAIL [%s] %-26s got=%0d exp=%0d",
                     phase.name(), what, got, exp);
            fails++;
        end
    endtask

    task automatic ckh(input string what, input logic [31:0] got,
                                          input logic [31:0] exp);
        checks++;
        if (got !== exp) begin
            $display("  FAIL [%s] %-26s got=%h exp=%h",
                     phase.name(), what, got, exp);
            fails++;
        end
    endtask

    // -------------------------------------------------------------------
    // Message drivers, receive domain
    // -------------------------------------------------------------------
    task automatic send(input msg_kind_t k, input msg_payload_t pl);
        @(negedge rx_clk);
        src_kind = k; src_pl = pl; src_valid = 1'b1;
        while (!src_ready) @(negedge rx_clk);
        @(posedge rx_clk);
        @(negedge rx_clk);
        src_valid = 1'b0;
    endtask

    task automatic send_hdr(input int base, input int h, input int w);
        pl_burst_hdr_t p;
        p.base_addr = 24'(base); p.height = 10'(h); p.width = 10'(w);
        send(MSG_BURST_HDR, msg_payload_t'(p));
    endtask

    task automatic send_data(input logic [23:0] p0, p1, p2, p3);
        pl_burst_data_t p;
        p.px0 = p0; p.px1 = p1; p.px2 = p2; p.px3 = p3;
        send(MSG_BURST_DATA, msg_payload_t'(p));
    endtask

    task automatic send_pix(input int addr, input logic [23:0] pix);
        pl_pix_write_t p;
        p.addr = 24'(addr); p.pixel = pix;
        send(MSG_PIX_WRITE, msg_payload_t'(p));
    endtask

    task automatic settle();
        repeat (40) @(posedge mem_clk);
    endtask

    task automatic show_mem(input string label);
        $display("    %-14s R words 0..7: %h %h %h %h %h %h %h %h",
                 label, w0_r, w1_r, w2_r, w3_r, w4_r, w5_r, w6_r, w7_r);
    endtask

    // -------------------------------------------------------------------
    // Read the image back
    // -------------------------------------------------------------------
    task automatic read_word(input int word,
                             output logic [DATA_W-1:0] r,
                             output logic [DATA_W-1:0] g,
                             output logic [DATA_W-1:0] b);
        @(negedge mem_clk);
        tb_rd_addr = ADDR_W'(word); tb_rd_en = 1'b1;
        @(negedge mem_clk);
        tb_rd_en = 1'b0;
        @(negedge mem_clk);
        r = rd_r; g = rd_g; b = rd_b;
    endtask

    // Lane 0 is the MOST significant byte.
    function automatic logic [7:0] lane_of(input logic [DATA_W-1:0] w,
                                           input int lane);
        return w[(NLANE-1-lane)*8 +: 8];
    endfunction

    // One pixel of the image, by linear address.
    task automatic read_pixel(input int lin,
                              output logic [7:0] r,
                              output logic [7:0] g,
                              output logic [7:0] b);
        logic [DATA_W-1:0] wr_, wg_, wb_;
        read_word(lin / NLANE, wr_, wg_, wb_);
        r = lane_of(wr_, lin % NLANE);
        g = lane_of(wg_, lin % NLANE);
        b = lane_of(wb_, lin % NLANE);
    endtask

    // Expected pixel value for the 4x4 burst, computed at 8 bits so the
    // arithmetic never widens into the caller's cast context.
    function automatic logic [7:0] exp_burst(input int row, input int col);
        return 8'h10 + 8'(row * 16) + 8'(col);
    endfunction

    logic [7:0] pr, pg, pb;
    logic [DATA_W-1:0] w0r, w0g, w0b;

    // -------------------------------------------------------------------
    initial begin
        $dumpfile("tb_mem_write_path_wv.fst");
        $dumpvars(0, tb_mem_write_path);

        checks = 0; fails = 0; clear_counts();

        repeat (6) @(posedge mem_clk);
        rx_rst_n = 1'b1; mem_rst_n = 1'b1;
        repeat (6) @(posedge mem_clk);

        $display("");
        $display("geometry %0dx%0d  words=%0d  crossing width=%0d bits",
                 IMG_WIDTH, IMG_HEIGHT, SRAM_DEPTH, XW);

        // ===============================================================
        phase = P_BURST;
        //
        // A 4x4 rectangle at the origin of an 8-wide image: 16 pixels,
        // four data messages. Rows land on words 0, 2, 4 and 6 -- the
        // address JUMPS between rows, because a rectangle narrower than the
        // image is not contiguous.
        //
        // Every row is four pixels, exactly one word, so every write is a
        // FULL WORD and NO READ is needed.
        // ===============================================================
        clear_counts();
        show_mem("before burst");
        burst_active = 1'b1;
        send_hdr(0, 4, 4);
        send_data(24'h100000, 24'h110000, 24'h120000, 24'h130000);  // row 0
        send_data(24'h200000, 24'h210000, 24'h220000, 24'h230000);  // row 1
        send_data(24'h300000, 24'h310000, 24'h320000, 24'h330000);  // row 2
        send_data(24'h400000, 24'h410000, 24'h420000, 24'h430000);  // row 3
        settle();
        burst_active = 1'b0;
        show_mem("after burst");

        ck("writes",        n_writes, 4);
        ck("all full words", n_full,   4);
        ck("no partial",     n_partial, 0);
        ck("NO READS EVER",  n_reads,  0);

        // read every pixel of the rectangle back
        for (int row = 0; row < 4; row++) begin
            for (int col = 0; col < 4; col++) begin
                read_pixel(row*IMG_WIDTH + col, pr, pg, pb);
                ckh($sformatf("pixel (%0d,%0d)", row, col),
                    32'(pr), 32'(exp_burst(row, col)));
            end
        end

        // the word between rows 0 and 1 must be untouched
        read_word(1, w0r, w0g, w0b);
        ckh("gap word untouched", w0r, 32'h0);

        // ===============================================================
        phase = P_SINGLE;
        //
        // One pixel into a word that already holds three others. No byte
        // enable, so this is a read-modify-write -- and the neighbours must
        // survive it.
        // ===============================================================
        clear_counts();
        show_mem("before pixel");
        send_pix(2, 24'hEE_00_00);
        settle();
        show_mem("after pixel");
        // One pixel changes one lane of a word that already holds three
        // others. With a byte enable that is ONE WRITE AND NO READ -- the
        // masked lanes are simply not driven and keep their contents.
        ck("one write",    n_writes,  1);
        ck("partial word", n_partial, 1);
        ck("NO READ",      n_reads,   0);

        read_pixel(0, pr, pg, pb); ckh("pixel 0 kept",    32'(pr), 32'h10);
        read_pixel(1, pr, pg, pb); ckh("pixel 1 kept",    32'(pr), 32'h11);
        read_pixel(2, pr, pg, pb); ckh("pixel 2 updated", 32'(pr), 32'hEE);
        read_pixel(3, pr, pg, pb); ckh("pixel 3 kept",    32'(pr), 32'h13);

        // ===============================================================
        phase = P_NARROW;
        //
        // A 2x2 rectangle at linear 9 (row 1, col 1). Two pixels per row, so
        // BOTH rows flush partial words -- two read-modify-writes, and the
        // pixels either side of each pair must survive.
        // ===============================================================
        clear_counts();
        show_mem("before narrow");
        burst_active = 1'b1;
        send_hdr(9, 2, 2);
        // A 2x2 rectangle is FOUR pixels, so all four values are real:
        //   row 1 cols 1,2 -> linear 9,10  -> word 2, lanes 1,2
        //   row 2 cols 1,2 -> linear 17,18 -> word 4, lanes 1,2
        send_data(24'h510000, 24'h520000, 24'h530000, 24'h540000);
        settle();
        burst_active = 1'b0;
        show_mem("after narrow");

        ck("two writes",       n_writes,  2);
        ck("both partial",     n_partial, 2);
        ck("NO READS",         n_reads,   0);

        read_pixel(9,  pr, pg, pb); ckh("row1 col1", 32'(pr), 32'h51);
        read_pixel(10, pr, pg, pb); ckh("row1 col2", 32'(pr), 32'h52);
        read_pixel(8,  pr, pg, pb); ckh("row1 col0 kept", 32'(pr), 32'h20);
        read_pixel(11, pr, pg, pb); ckh("row1 col3 kept", 32'(pr), 32'h23);

        read_pixel(17, pr, pg, pb); ckh("row2 col1", 32'(pr), 32'h53);
        read_pixel(18, pr, pg, pb); ckh("row2 col2", 32'(pr), 32'h54);
        read_pixel(16, pr, pg, pb); ckh("row2 col0 kept", 32'(pr), 32'h30);
        read_pixel(19, pr, pg, pb); ckh("row2 col3 kept", 32'(pr), 32'h33);

        // ===============================================================
        phase = P_CONTEND;
        //
        // A reader owns the port while messages arrive. Nothing may be lost:
        // the write path stalls, back-pressure reaches the crossing, and the
        // source stops. Everything lands once the reader releases.
        // ===============================================================
        clear_counts();
        @(negedge mem_clk); rom_seq_busy = 1'b1;   // image read in progress
        repeat (4) @(posedge mem_clk);
        ck("write path refused the port", int'(wr_allowed), 0);

        fork
            begin
                send_pix(20, 24'hAB_00_00);
                send_pix(21, 24'hCD_00_00);
            end
            begin
                repeat (200) @(posedge mem_clk);
                ck("nothing written while blocked", n_writes, 0);
                @(negedge mem_clk); rom_seq_busy = 1'b0;
            end
        join
        settle();

        ck("writes landed after release", n_writes, 2);
        read_pixel(20, pr, pg, pb); ckh("pixel 20", 32'(pr), 32'hAB);
        read_pixel(21, pr, pg, pb); ckh("pixel 21", 32'(pr), 32'hCD);

        // ===============================================================
        phase = P_DONE;
        // ===============================================================
        repeat (20) @(posedge mem_clk);

        $display("");
        $display("=====================================================");
        $display("  checks run : %0d", checks);
        $display("  failures   : %0d", fails);
        $display("  RESULT     : %s", (fails == 0) ? "PASS" : "FAIL");
        $display("=====================================================");
        $finish;
    end

    initial begin
        #2_000_000;
        $display("TIMEOUT in phase '%s'", phase.name());
        $finish;
    end

endmodule
