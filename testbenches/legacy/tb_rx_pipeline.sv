// tb_rx_pipeline.sv
// -----------------
// L5 integration test: rx_msg_decode + rx_mac + rx_parser +
// rx_pixel_wr_parser + rx_classifier, driven from a raw byte stream.
//
// This testbench exists for the ROUTING GATES, which are correctness
// critical rather than stylistic:
//
//   1. A Single Pixel Write must NOT raise classifier_valid. chip_top
//      computes rgf_pc_wen = rx_classifier_valid && !rx_col_q[0], so a
//      leaked pulse would drive an RGF write using the STALE row_q/col_q
//      of the previous legacy message -- potentially into IMG_CTRL,
//      starting a spurious image transfer.
//
//   2. A Single Pixel Write must NOT raise classifier_error either.
//      rx_parser asserts parse_error on every {W...} frame because its
//      framing check fails, so an ungated error output would flag every
//      pixel write as a fault.
//
//   3. A legacy message must NOT raise cmd_valid, and must latch
//      row_q/col_q/pixel_q exactly as before.
//
//   4. Spec Register Read / Register Write frames are recognised and
//      deliberately IGNORED in Stage 2A: no classifier_valid, no
//      cmd_valid, and no error either -- they are valid messages we
//      simply do not act on yet.

`timescale 1ns/1ps

module tb_rx_pipeline;

    import msg_pkg::*;
    import rx_msg_pkg::*;

    logic clk = 1'b0;
    always #5 clk = ~clk;

    logic rst_n;

    logic       byte_valid  = 1'b0;
    logic [7:0] rx_byte     = 8'h00;
    logic       par_val_rst = 1'b0;

    // ---- interconnect ------------------------------------------------
    logic [BYTE_CNT_W-1:0] expected_len;
    msg_kind_t             msg_kind_prov;
    logic                  kind_known;
    logic [127:0]          frame_buf;
    logic [BYTE_CNT_W-1:0] byte_cnt;

    logic         msg_valid;
    logic [127:0] msg_data;
    msg_kind_t    msg_kind_q;
    logic         mac_busy;

    logic        parse_valid, parse_error;
    logic [9:0]  row, col;
    logic [23:0] pixel;

    logic        pw_parse_valid, pw_parse_error;
    logic [23:0] pw_addr, pw_pixel;

    logic        classifier_valid, classifier_error;
    logic [9:0]  row_q, col_q;
    logic [23:0] pixel_q;
    logic        cmd_valid;
    logic [23:0] cmd_addr, cmd_pixel;
    // M4: bypass_active is now driven by rx_burst_ctrl, which this
    // Stage 2A testbench does not instantiate. Tied low -- this suite
    // never enters burst mode, so the behaviour under test is unchanged.
    logic        bypass_active = 1'b0;

    rx_msg_decode u_dec (
        .frame_buf(frame_buf), .byte_cnt(byte_cnt),
        .bypass_active(bypass_active),
        .expected_len(expected_len), .msg_kind_prov(msg_kind_prov),
        .kind_known(kind_known)
    );

    rx_mac u_mac (
        .clk(clk), .rst_n(rst_n),
        .byte_valid(byte_valid), .rx_byte(rx_byte), .par_val_rst(par_val_rst),
        .expected_len(expected_len), .msg_kind_prov(msg_kind_prov),
        .frame_buf(frame_buf), .byte_cnt(byte_cnt),
        .msg_valid(msg_valid), .msg_data(msg_data),
        .msg_kind_q(msg_kind_q), .mac_busy(mac_busy)
    );

    rx_parser u_parser (
        .msg_in(msg_data), .parse_valid(parse_valid), .parse_error(parse_error),
        .row(row), .col(col), .pixel(pixel)
    );

    rx_pixel_wr_parser u_pw (
        .msg_in(msg_data),
        .pw_parse_valid(pw_parse_valid), .pw_parse_error(pw_parse_error),
        .pw_addr(pw_addr), .pw_pixel(pw_pixel)
    );

    rx_classifier u_cls (
        .clk(clk), .rst_n(rst_n),
        .msg_valid(msg_valid), .msg_kind(msg_kind_q),
        .parse_valid(parse_valid), .parse_error(parse_error),
        .row(row), .col(col), .pixel(pixel),
        .pw_parse_valid(pw_parse_valid), .pw_parse_error(pw_parse_error),
        .pw_addr(pw_addr), .pw_pixel(pw_pixel),
        .classifier_valid(classifier_valid), .classifier_error(classifier_error),
        .row_q(row_q), .col_q(col_q), .pixel_q(pixel_q),
        .cmd_valid(cmd_valid), .cmd_addr(cmd_addr), .cmd_pixel(cmd_pixel),
        .burst_active(bypass_active),  // M4: input, tied low here
        // Register Read inputs tied inactive: this suite pre-dates the
        // feature and never sends MSG_REG_READ.
        .rr_valid(1'b0), .rr_addr_err(1'b0), .rr_rgf_addr(6'd0),
        .rr_cmd_valid(), .rr_cmd_addr()
    );

    // -----------------------------------------------------------------
    // Pulse counters, sampled every cycle
    // -----------------------------------------------------------------
    int n_cls_valid = 0, n_cls_error = 0, n_cmd_valid = 0;
    int errors = 0;
    string phase = "init";

    always @(posedge clk) begin
        if (rst_n) begin
            if (classifier_valid) n_cls_valid++;
            if (classifier_error) n_cls_error++;
            if (cmd_valid)        n_cmd_valid++;
        end
    end

    task automatic reset_counts();
        n_cls_valid = 0; n_cls_error = 0; n_cmd_valid = 0;
    endtask

    task automatic expect_counts(input int cv, input int ce, input int cmd);
        if (n_cls_valid != cv || n_cls_error != ce || n_cmd_valid != cmd) begin
            errors++;
            $display("[%s] FAIL pulses: cls_valid=%0d/%0d cls_error=%0d/%0d cmd_valid=%0d/%0d",
                     phase, n_cls_valid, cv, n_cls_error, ce, n_cmd_valid, cmd);
        end
        reset_counts();
    endtask

    task automatic chk(input bit cond, input string what);
        if (!cond) begin
            errors++;
            $display("[%s] FAIL %s", phase, what);
        end
    endtask

    // -----------------------------------------------------------------
    task automatic send_byte(input logic [7:0] b);
        @(negedge clk);
        rx_byte = b; byte_valid = 1'b1;
        @(negedge clk);
        byte_valid = 1'b0;
        repeat (6) @(negedge clk);
    endtask

    task automatic send_legacy(input int r, input int c, input int v);
        send_byte(CHAR_OPEN_BRACE); send_byte(CHAR_R);
        send_byte(ASCII_ZERO + 8'((r/100)%10));
        send_byte(ASCII_ZERO + 8'((r/10)%10));
        send_byte(ASCII_ZERO + 8'(r%10));
        send_byte(CHAR_COMMA); send_byte(CHAR_C);
        send_byte(ASCII_ZERO + 8'((c/100)%10));
        send_byte(ASCII_ZERO + 8'((c/10)%10));
        send_byte(ASCII_ZERO + 8'(c%10));
        send_byte(CHAR_COMMA); send_byte(CHAR_V);
        send_byte(ASCII_ZERO + 8'((v/100)%10));
        send_byte(ASCII_ZERO + 8'((v/10)%10));
        send_byte(ASCII_ZERO + 8'(v%10));
        send_byte(CHAR_CLOSE_BRACE);
        repeat (10) @(negedge clk);
    endtask

    task automatic send_pix_write(input logic [23:0] a, input logic [23:0] px);
        send_byte(CHAR_OPEN_BRACE); send_byte(CHAR_W);
        send_byte(a[23:16]); send_byte(a[15:8]); send_byte(a[7:0]);
        send_byte(CHAR_COMMA); send_byte(CHAR_P);
        send_byte(px[23:16]); send_byte(px[15:8]); send_byte(px[7:0]);
        send_byte(CHAR_CLOSE_BRACE);
        repeat (10) @(negedge clk);
    endtask

    task automatic send_reg_read(input logic [23:0] a);
        send_byte(CHAR_OPEN_BRACE); send_byte(CHAR_R);
        send_byte(a[23:16]); send_byte(a[15:8]); send_byte(a[7:0]);
        send_byte(CHAR_CLOSE_BRACE);
        repeat (10) @(negedge clk);
    endtask

    // Spec Single Pixel Read {R<R2,R1,R0>, C<C2,C1,C0>, P<R,G,B>}.
    // Byte 11 is 'P', which is what separates it from the legacy form.
    task automatic send_pix_read(input logic [9:0] r, input logic [9:0] c,
                                 input logic [23:0] px);
        send_byte(CHAR_OPEN_BRACE); send_byte(CHAR_R);
        send_byte(8'h00); send_byte({6'b0, r[9:8]}); send_byte(r[7:0]);
        send_byte(CHAR_COMMA); send_byte(CHAR_C);
        send_byte(8'h00); send_byte({6'b0, c[9:8]}); send_byte(c[7:0]);
        send_byte(CHAR_COMMA); send_byte(CHAR_P);
        send_byte(px[23:16]); send_byte(px[15:8]); send_byte(px[7:0]);
        send_byte(CHAR_CLOSE_BRACE);
        repeat (10) @(negedge clk);
    endtask

    // Spec Register Write {W<A2,A1,A0>, V<0,DH1,DH0>, V<0,DL1,DL0>}
    task automatic send_reg_write(input logic [23:0] a, input logic [31:0] d);
        send_byte(CHAR_OPEN_BRACE); send_byte(CHAR_W);
        send_byte(a[23:16]); send_byte(a[15:8]); send_byte(a[7:0]);
        send_byte(CHAR_COMMA); send_byte(CHAR_V);
        send_byte(8'h00); send_byte(d[31:24]); send_byte(d[23:16]);
        send_byte(CHAR_COMMA); send_byte(CHAR_V);
        send_byte(8'h00); send_byte(d[15:8]); send_byte(d[7:0]);
        send_byte(CHAR_CLOSE_BRACE);
        repeat (10) @(negedge clk);
    endtask

    // Spec Image Burst Read {R<A2,A1,A0>, H<H2,H1,H0>, W<W2,W1,W0>}
    task automatic send_burst_read(input logic [23:0] a, input logic [23:0] h,
                                   input logic [23:0] w);
        send_byte(CHAR_OPEN_BRACE); send_byte(CHAR_R);
        send_byte(a[23:16]); send_byte(a[15:8]); send_byte(a[7:0]);
        send_byte(CHAR_COMMA); send_byte(CHAR_H);
        send_byte(h[23:16]); send_byte(h[15:8]); send_byte(h[7:0]);
        send_byte(CHAR_COMMA); send_byte(CHAR_W);
        send_byte(w[23:16]); send_byte(w[15:8]); send_byte(w[7:0]);
        send_byte(CHAR_CLOSE_BRACE);
        repeat (10) @(negedge clk);
    endtask

    // -----------------------------------------------------------------
    initial begin
        logic [9:0]  saved_row, saved_col;
        logic [23:0] saved_pixel;

        $display("=================================================");
        $display(" L5: receive pipeline routing");
        $display("=================================================");

        rst_n = 1'b0;
        repeat (5) @(negedge clk);
        rst_n = 1'b1;
        repeat (5) @(negedge clk);
        reset_counts();

        // -------------------------------------------------------------
        // PHASE 1: legacy message drives the RGF path only.
        // -------------------------------------------------------------
        phase = "1-legacy";
        send_legacy(2, 0, 1);                 // IMG_CTRL write 1
        expect_counts(1, 0, 0);
        chk(row_q   === 10'd2, "row_q latched");
        chk(col_q   === 10'd0, "col_q latched");
        chk(pixel_q === 24'd1, "pixel_q latched");

        saved_row = row_q; saved_col = col_q; saved_pixel = pixel_q;

        // -------------------------------------------------------------
        // PHASE 2: Single Pixel Write drives the command path only, and
        // must leave the legacy registers completely undisturbed.
        // -------------------------------------------------------------
        phase = "2-pix-write";
        send_pix_write(24'h00_1234, 24'hFF_8040);
        expect_counts(0, 0, 1);
        chk(cmd_addr  === 24'h00_1234, "cmd_addr latched");
        chk(cmd_pixel === 24'hFF_8040, "cmd_pixel latched");
        chk(row_q   === saved_row,   "row_q undisturbed by pixel write");
        chk(col_q   === saved_col,   "col_q undisturbed by pixel write");
        chk(pixel_q === saved_pixel, "pixel_q undisturbed by pixel write");

        // -------------------------------------------------------------
        // PHASE 3: interleaved traffic. The legacy path must keep
        // working with pixel writes mixed in between.
        // -------------------------------------------------------------
        phase = "3-interleaved";
        send_legacy(4, 0, 1);
        send_pix_write(24'h00_0001, 24'h01_0203);
        send_legacy(4, 0, 0);
        send_pix_write(24'h00_0002, 24'h04_0506);
        expect_counts(2, 0, 2);
        chk(row_q     === 10'd4,        "row_q from last legacy msg");
        chk(pixel_q   === 24'd0,        "pixel_q from last legacy msg");
        chk(cmd_addr  === 24'h00_0002,  "cmd_addr from last pixel write");
        chk(cmd_pixel === 24'h04_0506,  "cmd_pixel from last pixel write");

        // -------------------------------------------------------------
        // PHASE 4: spec messages that are recognised and deliberately
        // ignored. None may raise classifier_error -- they are valid
        // frames with no consumer yet, not faults.
        // -------------------------------------------------------------
        phase = "4-recognised-but-ignored";

        // RE-SNAPSHOT the legacy registers. The values captured at the end
        // of phase 1 are stale by now: phase 3 deliberately drives further
        // legacy traffic, so row_q/col_q/pixel_q hold that message's fields
        // rather than phase 1's. The "undisturbed" checks below must compare
        // against the state as it actually is on entry to this phase.
        saved_row   = row_q;
        saved_col   = col_q;
        saved_pixel = pixel_q;

        // Register Read {R<A>} -- 6 bytes
        send_reg_read(24'h00_0008);
        expect_counts(0, 0, 0);

        // Single Pixel Read {R<..>,C<..>,P<..>} -- 16 bytes, byte 11 'P'.
        // Shares byte 1 and byte 6 with the legacy message and is
        // separated from it only at byte 11. It must NOT reach the RGF
        // path: a leaked classifier_valid here would drive an RGF write
        // from the previous message's stale row_q/col_q.
        send_pix_read(10'd12, 10'd34, 24'hAA_BB_CC);
        expect_counts(0, 0, 0);
        chk(row_q   === saved_row,   "row_q undisturbed by pixel read");
        chk(col_q   === saved_col,   "col_q undisturbed by pixel read");
        chk(pixel_q === saved_pixel, "pixel_q undisturbed by pixel read");

        // Register Write {W<A>,V<..>,V<..>} -- 16 bytes, byte 6 'V'
        send_reg_write(24'h00_0008, 32'h1234_5678);
        expect_counts(0, 0, 0);

        // Image Burst Read {R<A>,H<..>,W<..>} -- 16 bytes, byte 6 'H'
        send_burst_read(24'h00_0000, 24'h00_0100, 24'h00_0100);
        expect_counts(0, 0, 0);

        // -------------------------------------------------------------
        // PHASE 5: malformed pixel write reports an error and issues no
        // command. Framing is still count-based, so the frame completes
        // at 11 bytes and is then rejected by the parser.
        // -------------------------------------------------------------
        phase = "5-bad-pix-write";
        send_byte(CHAR_OPEN_BRACE); send_byte(CHAR_W);
        send_byte(8'h00); send_byte(8'h00); send_byte(8'h05);
        send_byte(CHAR_COMMA); send_byte(CHAR_P);
        send_byte(8'h11); send_byte(8'h22); send_byte(8'h33);
        send_byte(8'h00);                       // corrupt terminator
        repeat (10) @(negedge clk);
        expect_counts(0, 1, 0);

        // -------------------------------------------------------------
        // PHASE 6: binary payload containing the delimiters still routes
        // correctly all the way to cmd_addr / cmd_pixel.
        // -------------------------------------------------------------
        phase = "6-binary-payload";
        send_pix_write(24'h7B_7D_7B, 24'h7D_7B_7D);
        expect_counts(0, 0, 1);
        chk(cmd_addr  === 24'h7B_7D_7B, "cmd_addr with delimiter bytes");
        chk(cmd_pixel === 24'h7D_7B_7D, "cmd_pixel with delimiter bytes");

        // -------------------------------------------------------------
        // PHASE 7: bypass_active must be inactive throughout Stage 2A.
        // -------------------------------------------------------------
        phase = "7-bypass-tied-low";
        chk(bypass_active === 1'b0, "bypass_active inactive in Stage 2A");

        $display("-------------------------------------------------");
        $display(" errors: %0d", errors);
        $display(" RESULT: %s", (errors == 0) ? "PASS" : "FAIL");
        $display("=================================================");
        $finish;
    end

endmodule : tb_rx_pipeline
