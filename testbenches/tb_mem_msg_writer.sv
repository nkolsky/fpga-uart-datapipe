// tb_mem_msg_writer.sv
// ====================
// mem_msg_writer + pixel_word_packer together: messages in, masked word
// writes out. This is the whole memory-side write path short of the SRAMs.
//
// Build: TOP = tb_mem_msg_writer
//        SRCS = msg_format_pkg.sv memory_pkg.sv pixel_word_packer.sv
//               mem_msg_writer.sv tb_mem_msg_writer.sv
// -DSIMULATION gives 8x8 geometry.

`timescale 1ns/1ps

module tb_mem_msg_writer;

    import msg_format_pkg::*;
    import memory_pkg::*;

    localparam int ADDR_W = SRAM_ADDR_WIDTH;
    localparam int DATA_W = SRAM_DATA_WIDTH;
    localparam int NLANE  = PIXELS_PER_WORD;
    localparam int PAYW   = 96;

    typedef enum logic [2:0] {
        P_INIT    = 3'd0,
        P_SINGLE  = 3'd1,
        P_ALIGNED = 3'd2,
        P_CARRY   = 3'd3,
        P_RECT    = 3'd4,
        P_IGNORE  = 3'd5,
        P_DONE    = 3'd6
    } tb_phase_e;

    tb_phase_e phase = P_INIT;

    logic clk = 1'b0, rst_n = 1'b0;
    always #5 clk = ~clk;

    // ---- crossing side -------------------------------------------------
    logic            msg_valid = 1'b0;
    logic            msg_ready;
    msg_kind_t       msg_kind  = MSG_UNKNOWN;
    logic [PAYW-1:0] msg_payload = '0;

    // ---- writer -> packer ----------------------------------------------
    logic        pack_start;
    logic [23:0] pack_base_addr;
    logic [9:0]  pack_height, pack_width;
    logic        pixel_valid, pixel_ready;
    logic [7:0]  pixel_r, pixel_g, pixel_b;
    logic        pack_busy, pack_done;

    mem_msg_writer u_wr (
        .clk(clk), .rst_n(rst_n),
        .msg_valid(msg_valid), .msg_ready(msg_ready),
        .msg_kind(msg_kind), .msg_payload(msg_payload),
        .pack_start(pack_start), .pack_base_addr(pack_base_addr),
        .pack_height(pack_height), .pack_width(pack_width),
        .pixel_valid(pixel_valid), .pixel_ready(pixel_ready),
        .pixel_r(pixel_r), .pixel_g(pixel_g), .pixel_b(pixel_b),
        .pack_busy(pack_busy)
    );

    // ---- packer -> SRAM write port --------------------------------------
    logic              wr_valid;
    logic              wr_ready = 1'b1;
    logic [ADDR_W-1:0] wr_addr;
    logic [NLANE-1:0]  wr_be;
    logic [DATA_W-1:0] wr_data_r, wr_data_g, wr_data_b;

    pixel_word_packer u_pack (
        .clk(clk), .rst_n(rst_n),
        .start(pack_start), .base_addr(pack_base_addr),
        .height(pack_height), .width(pack_width),
        .pixel_valid(pixel_valid), .pixel_ready(pixel_ready),
        .pixel_r(pixel_r), .pixel_g(pixel_g), .pixel_b(pixel_b),
        .wr_valid(wr_valid), .wr_ready(wr_ready),
        .wr_addr(wr_addr), .wr_be(wr_be),
        .wr_data_r(wr_data_r), .wr_data_g(wr_data_g), .wr_data_b(wr_data_b),
        .busy(pack_busy), .done(pack_done)
    );

    // -------------------------------------------------------------------
    // Observation
    // -------------------------------------------------------------------
    int                n_wr;
    logic [ADDR_W-1:0] wa_q [$];
    logic [NLANE-1:0]  be_q [$];
    logic [DATA_W-1:0] dr_q [$];

    always @(posedge clk) if (rst_n && wr_valid && wr_ready) begin
        n_wr++;
        wa_q.push_back(wr_addr);
        be_q.push_back(wr_be);
        dr_q.push_back(wr_data_r);
    end

    task automatic clear_wr();
        n_wr = 0; wa_q.delete(); be_q.delete(); dr_q.delete();
    endtask

    int checks, fails;

    task automatic ck(input string what, input int got, input int exp);
        checks++;
        if (got !== exp) begin
            $display("  FAIL [%s] %-22s got=%0d exp=%0d",
                     phase.name(), what, got, exp);
            fails++;
        end
    endtask

    task automatic ckb(input string what, input logic [31:0] got,
                                          input logic [31:0] exp);
        checks++;
        if (got !== exp) begin
            $display("  FAIL [%s] %-22s got=%b exp=%b",
                     phase.name(), what, got, exp);
            fails++;
        end
    endtask

    task automatic ckh(input string what, input logic [31:0] got,
                                          input logic [31:0] exp);
        checks++;
        if (got !== exp) begin
            $display("  FAIL [%s] %-22s got=%h exp=%h",
                     phase.name(), what, got, exp);
            fails++;
        end
    endtask

    // -------------------------------------------------------------------
    // Message drivers. Field layouts match mem_msg_writer's header.
    // -------------------------------------------------------------------
    task automatic send_msg(input msg_kind_t k, input logic [PAYW-1:0] pl);
        @(negedge clk);
        msg_kind = k; msg_payload = pl; msg_valid = 1'b1;
        while (!msg_ready) @(negedge clk);
        @(posedge clk);
        @(negedge clk);
        msg_valid = 1'b0;
    endtask

    task automatic send_pix_write(input int addr, input logic [23:0] pix);
        logic [PAYW-1:0] pl;
        pl = '0;
        pl[47:24] = 24'(addr);
        pl[23:0]  = pix;
        send_msg(MSG_PIX_WRITE, pl);
    endtask

    task automatic send_burst_hdr(input int base, input int h, input int w);
        logic [PAYW-1:0] pl;
        pl = '0;
        pl[23:0]  = 24'(base);
        pl[33:24] = 10'(h);
        pl[43:34] = 10'(w);
        send_msg(MSG_BURST_HDR, pl);
    endtask

    task automatic send_burst_data(input logic [23:0] p0, p1, p2, p3);
        send_msg(MSG_BURST_DATA, {p0, p1, p2, p3});
    endtask

    task automatic settle();
        repeat (8) @(negedge clk);
    endtask

    task automatic show();
        $display("    %0d write(s):", n_wr);
        for (int i = 0; i < n_wr; i++)
            $display("        word=%0d be=%b data_r=%h", wa_q[i], be_q[i], dr_q[i]);
    endtask

    // -------------------------------------------------------------------
    initial begin
        $dumpfile("tb_mem_msg_writer_wv.fst");
        $dumpvars(0, tb_mem_msg_writer);

        checks = 0; fails = 0;
        clear_wr();

        repeat (4) @(negedge clk);
        rst_n = 1'b1;
        repeat (2) @(negedge clk);

        $display("");
        $display("geometry %0dx%0d  words=%0d", IMG_WIDTH, IMG_HEIGHT, SRAM_DEPTH);

        // ===============================================================
        phase = P_SINGLE;
        // One message, one pixel, one write with a one-hot enable.
        // ===============================================================
        clear_wr();
        send_pix_write(6, 24'h11_22_33);
        settle();
        $display("  single pixel write, linear 6:");
        show();
        ck ("writes", n_wr, 1);
        ck ("word",   int'(wa_q[0]), 1);
        ckb("be",     32'(be_q[0]), 32'b0010);
        ckh("data_r", 32'(dr_q[0]), 32'h0000_1100);   // 0x11 in lane 2

        // ===============================================================
        phase = P_ALIGNED;
        // Header opens a 1x4 rectangle at linear 4; one data message fills
        // it. ONE write, not four -- the whole point of the rework.
        // ===============================================================
        clear_wr();
        send_burst_hdr(4, 1, 4);
        send_burst_data(24'hA0_00_00, 24'hA1_00_00, 24'hA2_00_00, 24'hA3_00_00);
        settle();
        $display("  burst 1x4 at linear 4, one data message:");
        show();
        ck ("writes", n_wr, 1);
        ck ("word",   int'(wa_q[0]), 1);
        ckb("be",     32'(be_q[0]), 32'b1111);
        ckh("data_r", 32'(dr_q[0]), 32'hA0A1A2A3);

        // ===============================================================
        phase = P_CARRY;
        // 1x6 from linear 2 across TWO data messages. Pixels 4 and 5 are
        // held in the accumulator when the first message ends, and the
        // second message's padding must be refused.
        // ===============================================================
        clear_wr();
        send_burst_hdr(2, 1, 6);
        send_burst_data(24'h50_00_00, 24'h51_00_00, 24'h52_00_00, 24'h53_00_00);
        send_burst_data(24'h54_00_00, 24'h55_00_00, 24'hEE_00_00, 24'hEE_00_00);
        settle();
        $display("  burst 1x6 at linear 2, two data messages:");
        show();
        ck ("writes",   n_wr, 2);
        ck ("word 1st", int'(wa_q[0]), 0);
        ckb("be 1st",   32'(be_q[0]), 32'b0011);
        ckh("data 1st", 32'(dr_q[0]), 32'h00005051);
        ck ("word 2nd", int'(wa_q[1]), 1);
        ckb("be 2nd",   32'(be_q[1]), 32'b1111);
        ckh("data 2nd", 32'(dr_q[1]), 32'h52535455);

        // ===============================================================
        phase = P_RECT;
        // 2x4 at the origin of an 8-wide image: rows land on words 0 and 2,
        // NOT 0 and 1. The address jumps between rows.
        // ===============================================================
        clear_wr();
        send_burst_hdr(0, 2, 4);
        send_burst_data(24'hE0_00_00, 24'hE1_00_00, 24'hE2_00_00, 24'hE3_00_00);
        send_burst_data(24'hE4_00_00, 24'hE5_00_00, 24'hE6_00_00, 24'hE7_00_00);
        settle();
        $display("  burst 2x4 at origin, two data messages:");
        show();
        ck ("writes",    n_wr, 2);
        ck ("word row0", int'(wa_q[0]), 0);
        ckb("be row0",   32'(be_q[0]), 32'b1111);
        ckh("data row0", 32'(dr_q[0]), 32'hE0E1E2E3);
        ck ("word row1", int'(wa_q[1]), 2);
        ckb("be row1",   32'(be_q[1]), 32'b1111);
        ckh("data row1", 32'(dr_q[1]), 32'hE4E5E6E7);

        // ===============================================================
        phase = P_IGNORE;
        // Reads and register writes are handled elsewhere. They must be
        // accepted and dropped here, not stall the crossing.
        // ===============================================================
        clear_wr();
        send_msg(MSG_REG_WRITE, 96'h0);
        send_msg(MSG_REG_READ,  96'h0);
        send_msg(MSG_PIX_READ,  96'h0);
        send_msg(MSG_BURST_READ, 96'h0);
        settle();
        ck("no writes",     n_wr, 0);
        ck("no rect open",  int'(pack_busy), 0);

        // and a normal write still works afterwards
        clear_wr();
        send_pix_write(9, 24'h44_55_66);
        settle();
        ck ("write after ignores", n_wr, 1);
        ck ("word",  int'(wa_q[0]), 2);
        ckb("be",    32'(be_q[0]), 32'b0100);
        ckh("data_r", 32'(dr_q[0]), 32'h0044_0000);   // 0x44 in lane 1

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
