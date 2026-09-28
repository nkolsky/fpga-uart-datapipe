// tb_rx_mac.sv
// Directed test for rx_mac owning framing: start gate, delimiter positions,
// malformed-delimiter abort, truncation recovery, parity soft reset.
`timescale 1ns/1ps

module tb_rx_mac;

    import msg_format_pkg::*;

    logic clk = 0, rst_n = 0;
    always #5 clk = ~clk;

    logic       byte_valid  = 0;
    logic [7:0] rx_byte     = 8'h00;
    logic       par_val_rst = 0;
    logic       stall       = 0;

    logic [FRAME_W-1:0]    frame_buf;
    logic [BYTE_CNT_W-1:0] byte_cnt;
    logic                  frame_done, frame_err, mac_busy;

    rx_mac u_mac (.clk(clk), .rst_n(rst_n), .byte_valid(byte_valid),
        .rx_byte(rx_byte), .par_val_rst(par_val_rst), .stall(stall),
        .frame_buf(frame_buf), .byte_cnt(byte_cnt),
        .frame_done(frame_done), .frame_err(frame_err), .mac_busy(mac_busy));

    int fails = 0, checks = 0;
    string phase;

    int n_done, n_err;
    logic [FRAME_W-1:0]    last_frame;
    logic [BYTE_CNT_W-1:0] last_len;

    always @(posedge clk) if (rst_n) begin
        if (frame_done) begin
            n_done++; last_frame = frame_buf; last_len = byte_cnt;
        end
        if (frame_err) n_err++;
    end

    task automatic ck(input string nm, input int a, input int e);
        checks++;
        if (a !== e) begin
            $display("  FAIL [%s] %-24s got=%0d exp=%0d", phase, nm, a, e);
            fails++;
        end
    endtask

    task automatic ckh(input string nm, input logic [127:0] a,
                       input logic [127:0] e);
        checks++;
        if (a !== e) begin
            $display("  FAIL [%s] %-24s got=%h exp=%h", phase, nm, a, e);
            fails++;
        end
    endtask

    task automatic put(input logic [7:0] b);
        @(negedge clk);
        rx_byte = b; byte_valid = 1'b1;
        @(negedge clk);
        byte_valid = 1'b0;
        repeat (4) @(negedge clk);
    endtask

    task automatic put_frame(input byte unsigned b[16], input int n);
        for (int i = 0; i < n; i++) put(b[i]);
    endtask

    function automatic logic [127:0] pack(input byte unsigned b[16],
                                          input int n);
        logic [127:0] v;
        v = '0;
        for (int i = 0; i < n; i++) v[127 - 8*i -: 8] = b[i];
        return v;
    endfunction

    task automatic clear;
        @(negedge clk); par_val_rst = 1'b1;
        @(negedge clk); par_val_rst = 1'b0;
        repeat (4) @(negedge clk);
        n_done = 0; n_err = 0;
    endtask

    byte unsigned f[16];

    initial begin
        repeat (3) @(negedge clk);
        rst_n = 1;
        repeat (3) @(negedge clk);
        n_done = 0; n_err = 0;

        // =============================================================
        phase = "garbage before any frame";
        // =============================================================
        clear();
        put(8'hAA); put(8'h00); put(8'hFF); put("R"); put("}"); put(",");
        ck("frames done", n_done, 0);
        ck("frames err",  n_err,  0);
        ck("byte_cnt still 0", int'(byte_cnt), 0);

        // =============================================================
        phase = "6-byte frame, close at byte 5";
        // =============================================================
        clear();
        f = '{"{","R",8'h00,8'h00,8'h08,"}",0,0,0,0,0,0,0,0,0,0};
        put_frame(f, 6);
        ck("done", n_done, 1);
        ck("err",  n_err,  0);
        ck("length", int'(last_len), 6);

        // =============================================================
        phase = "11-byte frame, comma at 5, close at 10";
        // =============================================================
        clear();
        f = '{"{","W",8'h01,8'h23,8'h45,",","P",8'hAA,8'hBB,8'hCC,"}",
              0,0,0,0,0};
        put_frame(f, 11);
        ck("done", n_done, 1);
        ck("length", int'(last_len), 11);
        ckh("frame contents", last_frame & 128'hFFFFFFFFFFFFFFFFFFFFFF0000000000,
                              pack(f,11) & 128'hFFFFFFFFFFFFFFFFFFFFFF0000000000);

        // =============================================================
        phase = "16-byte frame, commas at 5 and 10, close at 15";
        // =============================================================
        clear();
        f = '{"{","W",8'h00,8'h00,8'h10,",","V",8'h00,8'hDE,8'hAD,
              ",","V",8'h00,8'hBE,8'hEF,"}"};
        put_frame(f, 16);
        ck("done", n_done, 1);
        ck("length", int'(last_len), 16);
        ckh("frame contents", last_frame, pack(f,16));

        // =============================================================
        phase = "bad byte at delimiter position 5";
        // =============================================================
        clear();
        f = '{"{","R",8'h00,8'h00,8'h08,8'h5A,0,0,0,0,0,0,0,0,0,0};
        put_frame(f, 6);
        ck("done", n_done, 0);
        ck("err",  n_err,  1);
        ck("byte_cnt cleared", int'(byte_cnt), 0);

        // =============================================================
        phase = "bad byte at delimiter position 10";
        // =============================================================
        clear();
        f = '{"{","W",8'h01,8'h23,8'h45,",","P",8'hAA,8'hBB,8'hCC,8'h00,
              0,0,0,0,0};
        put_frame(f, 11);
        ck("done", n_done, 0);
        ck("err",  n_err,  1);

        // =============================================================
        phase = "comma at byte 15 is illegal";
        // =============================================================
        clear();
        f = '{"{","W",8'h00,8'h00,8'h10,",","V",8'h00,8'hDE,8'hAD,
              ",","V",8'h00,8'hBE,8'hEF,","};
        put_frame(f, 16);
        ck("done", n_done, 0);
        ck("err",  n_err,  1);

        // =============================================================
        phase = "payload 0x7D at byte 7 is NOT a delimiter";
        // =============================================================
        // A pixel byte that happens to equal '}' must be ignored -- this is
        // the case a delimiter SEARCH would corrupt.
        clear();
        f = '{"{","W",8'h01,8'h23,8'h45,",","P",8'h7D,8'h7B,8'h2C,"}",
              0,0,0,0,0};
        put_frame(f, 11);
        ck("done", n_done, 1);
        ck("length", int'(last_len), 11);

        // =============================================================
        phase = "truncated frame recovers via MAC_ERR";
        // =============================================================
        // Three bytes then the host restarts. The '{' lands mid-frame, but
        // the splice now hits a bad delimiter at byte 5 and aborts, so the
        // receiver resyncs instead of running on.
        clear();
        put("{"); put("R"); put(8'h00);
        f = '{"{","R",8'h00,8'h00,8'h0C,"}",0,0,0,0,0,0,0,0,0,0};
        put_frame(f, 6);
        ck("splice aborted", n_err, 1);
        ck("nothing announced", n_done, 0);

        // a clean frame now works
        clear();
        f = '{"{","R",8'h00,8'h00,8'h04,"}",0,0,0,0,0,0,0,0,0,0};
        put_frame(f, 6);
        ck("recovered done", n_done, 1);
        ck("recovered err",  n_err,  0);

        // =============================================================
        phase = "parity error abandons frame";
        // =============================================================
        clear();
        put("{"); put("W"); put(8'h11);
        @(negedge clk); par_val_rst = 1'b1;
        @(negedge clk); par_val_rst = 1'b0;
        repeat (4) @(negedge clk);
        ck("byte_cnt cleared", int'(byte_cnt), 0);
        n_done = 0; n_err = 0;
        f = '{"{","R",8'h00,8'h00,8'h14,"}",0,0,0,0,0,0,0,0,0,0};
        put_frame(f, 6);
        ck("post-parity done", n_done, 1);

        // =============================================================
        phase = "burst data frames with no bypass signal";
        // =============================================================
        // The MAC has no bypass input. Burst data is framed purely by its
        // delimiters, exactly like every other message.
        clear();
        f = '{"{",8'h10,8'h20,8'h30,8'h40,",",8'h50,8'h60,8'h70,8'h80,
              ",",8'h90,8'hA0,8'hB0,8'hC0,"}"};
        put_frame(f, 16);
        ck("done", n_done, 1);
        ck("length", int'(last_len), 16);
        ckh("frame contents", last_frame, pack(f,16));

        // =============================================================
        phase = "three back-to-back frames";
        // =============================================================
        clear();
        f = '{"{","R",8'h00,8'h00,8'h08,"}",0,0,0,0,0,0,0,0,0,0};
        put_frame(f, 6);
        f = '{"{","R",8'h00,8'h00,8'h0C,"}",0,0,0,0,0,0,0,0,0,0};
        put_frame(f, 6);
        f = '{"{","R",8'h00,8'h00,8'h10,"}",0,0,0,0,0,0,0,0,0,0};
        put_frame(f, 6);
        ck("three done", n_done, 3);
        ck("no errors",  n_err,  0);

        // =============================================================
        phase = "short frame after a dirty 16-byte buffer";
        // =============================================================
        clear();
        f = '{"{","W",8'h00,8'h00,8'h10,",","V",8'h00,8'hDE,8'hAD,
              ",","V",8'h00,8'hBE,8'hEF,"}"};
        put_frame(f, 16);
        n_done = 0; n_err = 0;
        f = '{"{","R",8'h00,8'h00,8'h08,"}",0,0,0,0,0,0,0,0,0,0};
        put_frame(f, 6);
        ck("done", n_done, 1);
        ck("length", int'(last_len), 6);

        // =============================================================
        phase = "downstream stall";
        // =============================================================
        // A completed frame must WAIT rather than be announced, so the
        // classifier's held message cannot be overwritten. The frame stays
        // intact in the buffer and mac_busy stays high, which is what keeps
        // UART_CTS asserted and the PC quiet.
        clear();
        stall = 1'b1;
        f = '{"{","R",8'h00,8'h00,8'h08,"}",0,0,0,0,0,0,0,0,0,0};
        put_frame(f, 6);
        repeat (40) @(negedge clk);
        ck("nothing announced while stalled", n_done, 0);
        ck("no error either",                 n_err,  0);
        ck("mac still busy",                  int'(mac_busy), 1);
        ck("frame held at full length",       int'(byte_cnt), 6);

        stall = 1'b0;
        repeat (6) @(negedge clk);
        ck("announced after release", n_done, 1);
        ck("correct length",          int'(last_len), 6);
        // Only the frame's own bytes are compared. rx_mac deliberately does
        // NOT clear the buffer, so positions beyond byte_cnt still hold the
        // previous message -- that is by design, and rx_msg_parser never
        // reads them because byte_cnt bounds what is real.
        ckh("frame intact",
            last_frame & 128'hFFFF_FFFF_FFFF_0000_0000_0000_0000_0000,
            pack(f,6)   & 128'hFFFF_FFFF_FFFF_0000_0000_0000_0000_0000);

        // a clean frame still works afterwards
        clear();
        f = '{"{","R",8'h00,8'h00,8'h0C,"}",0,0,0,0,0,0,0,0,0,0};
        put_frame(f, 6);
        ck("frame after stall", n_done, 1);

        $display("");
        $display("=====================================================");
        $display("  checks run : %0d", checks);
        $display("  failures   : %0d", fails);
        $display("  RESULT     : %s", fails == 0 ? "PASS" : "FAIL");
        $display("=====================================================");
        $finish;
    end

endmodule
