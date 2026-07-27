// tb_rx_burst_mode.sv
// -------------------
// M4 regression for the RX-side control interface.
//
// Wires the receive path exactly as chip_top will in M5 -- rx_mac,
// rx_msg_decode, both burst parsers, rx_pixel_wr_parser, rx_parser,
// rx_classifier and rx_burst_ctrl -- and drives it with raw UART bytes.
//
// It exists to prove two properties that no unit test can reach, because
// both are about the RELATIONSHIP between modules:
//
//   1. BYPASS OWNERSHIP. bypass_active has exactly one driver,
//      rx_burst_ctrl, and it genuinely changes how rx_msg_decode classifies
//      frames. rx_classifier no longer has the port at all.
//
//   2. COMMAND MUTUAL EXCLUSION. rx_classifier's single-pixel cmd_valid and
//      rx_burst_ctrl's burst cmd_valid are never asserted in the same cycle,
//      and the classifier emits nothing whatsoever during a burst.
//
// -----------------------------------------------------------------------
// THE ADVERSARIAL CASE
// -----------------------------------------------------------------------
// Scenario 3 is the one that matters. Burst payload is RAW BINARY, so a
// data frame can legitimately carry bytes that spell a perfect Single Pixel
// Write -- '{' 'W' addr ',' 'P' rgb '}' -- or a perfect legacy RGF command.
// This testbench sends exactly such frames INSIDE a burst.
//
// With bypass_active working they are classified MSG_BURST_DATA and become
// four pixels. If bypass_active were ever mis-wired they would be
// classified MSG_PIX_WRITE or MSG_LEGACY_RGF, and ordinary image data would
// start issuing register writes -- one of which could address IMG_CTRL and
// launch an image read in the middle of a burst.
//
// That is why rx_classifier carries an explicit !burst_active gate as well
// as relying on the structural guarantee.

`timescale 1ns/1ps

module tb_rx_burst_mode;

    import msg_pkg::*;
    import rx_msg_pkg::*;
    import rx_burst_pkg::*;
    import memory_pkg::*;

    logic clk = 1'b0;
    always #3.846 clk = ~clk;            // 130 MHz receive domain
    logic rst_n;

    // ---- byte injection ------------------------------------------------
    logic       byte_valid = 1'b0;
    logic [7:0] rx_byte    = 8'h00;

    // ---- framing -------------------------------------------------------
    logic [BYTE_CNT_W-1:0] expected_len, byte_cnt;
    msg_kind_t             msg_kind_prov, msg_kind_q;
    logic                  kind_known;
    logic [127:0]          frame_buf, msg_data;
    logic                  msg_valid, mac_busy;

    // ---- parsers -------------------------------------------------------
    logic        parse_valid, parse_error;
    logic [9:0]  row, col;
    logic [23:0] pixel;

    logic        pw_parse_valid, pw_parse_error;
    logic [23:0] pw_addr, pw_pixel;

    logic                   hdr_frame_ok, hdr_dims_ok, hdr_valid;
    logic [BURST_DIM_W-1:0] b_height, b_width;

    logic                                          data_frame_ok, data_error;
    logic [BURST_PIX_PER_MSG-1:0][BURST_PIX_W-1:0] b_pixels;

    // ---- classifier ----------------------------------------------------
    logic        classifier_valid, classifier_error;
    logic [9:0]  row_q, col_q;
    logic [23:0] pixel_q;
    logic        cls_cmd_valid;
    logic [23:0] cls_cmd_addr, cls_cmd_pixel;

    // ---- burst controller ----------------------------------------------
    logic                    bypass_active, burst_active, burst_done;
    logic                    brst_cmd_valid;
    logic [BURST_ADDR_W-1:0] brst_cmd_addr;
    logic [BURST_PIX_W-1:0]  brst_cmd_pixel;
    logic                    err_hdr_invalid, err_data_invalid, err_unexpected;
    logic                    cmd_ready = 1'b1;

    rx_msg_decode u_decode (
        .frame_buf(frame_buf), .byte_cnt(byte_cnt),
        .bypass_active(bypass_active),           // <-- sole driver: rx_burst_ctrl
        .expected_len(expected_len), .msg_kind_prov(msg_kind_prov),
        .kind_known(kind_known));

    rx_mac u_mac (
        .clk(clk), .rst_n(rst_n),
        .byte_valid(byte_valid), .rx_byte(rx_byte), .par_val_rst(1'b0),
        .expected_len(expected_len), .msg_kind_prov(msg_kind_prov),
        .frame_buf(frame_buf), .byte_cnt(byte_cnt),
        .msg_valid(msg_valid), .msg_data(msg_data),
        .msg_kind_q(msg_kind_q), .mac_busy(mac_busy));

    rx_parser u_parser (
        .msg_in(msg_data), .parse_valid(parse_valid), .parse_error(parse_error),
        .row(row), .col(col), .pixel(pixel));

    rx_pixel_wr_parser u_pw (
        .msg_in(msg_data), .pw_parse_valid(pw_parse_valid),
        .pw_parse_error(pw_parse_error), .pw_addr(pw_addr), .pw_pixel(pw_pixel));

    rx_burst_hdr_parser u_hdr (
        .msg_in(msg_data), .hdr_frame_ok(hdr_frame_ok),
        .hdr_dims_ok(hdr_dims_ok), .hdr_valid(hdr_valid),
        .height(b_height), .width(b_width));

    rx_burst_data_parser u_bdata (
        .msg_in(msg_data), .data_frame_ok(data_frame_ok),
        .data_error(data_error), .pixels(b_pixels));

    rx_classifier u_cls (
        .clk(clk), .rst_n(rst_n),
        .msg_valid(msg_valid), .msg_kind(msg_kind_q),
        .parse_valid(parse_valid), .parse_error(parse_error),
        .row(row), .col(col), .pixel(pixel),
        .pw_parse_valid(pw_parse_valid), .pw_parse_error(pw_parse_error),
        .pw_addr(pw_addr), .pw_pixel(pw_pixel),
        .classifier_valid(classifier_valid), .classifier_error(classifier_error),
        .row_q(row_q), .col_q(col_q), .pixel_q(pixel_q),
        .cmd_valid(cls_cmd_valid), .cmd_addr(cls_cmd_addr),
        .cmd_pixel(cls_cmd_pixel),
        .burst_active(burst_active));            // <-- M4: input, not output

    rx_burst_ctrl u_burst (
        .clk(clk), .rst_n(rst_n),
        .msg_valid(msg_valid), .msg_kind(msg_kind_q),
        .hdr_valid(hdr_valid), .height(b_height), .width(b_width),
        .data_frame_ok(data_frame_ok), .pixels(b_pixels),
        .burst_abort(1'b0),
        .cmd_ready(cmd_ready),
        .cmd_valid(brst_cmd_valid), .cmd_addr(brst_cmd_addr),
        .cmd_pixel(brst_cmd_pixel),
        .bypass_active(bypass_active), .burst_active(burst_active),
        .burst_done(burst_done),
        .err_hdr_invalid(err_hdr_invalid),
        .err_data_invalid(err_data_invalid),
        .err_unexpected(err_unexpected));

    // -----------------------------------------------------------------
    // Bookkeeping
    // -----------------------------------------------------------------
    int    errors = 0, checks = 0;
    int    n_cls_cmd = 0, n_brst_cmd = 0, n_cls_valid = 0, n_cls_err = 0;
    string phase = "init";

    always @(posedge clk) begin
        if (rst_n) begin
            if (cls_cmd_valid)              n_cls_cmd++;
            if (brst_cmd_valid && cmd_ready) n_brst_cmd++;
            if (classifier_valid)           n_cls_valid++;
            if (classifier_error)           n_cls_err++;
        end
    end

    task automatic banner(input string name);
        phase = name; $display("--- %s", name);
    endtask

    task automatic chk(input bit cond, input string what);
        checks++;
        if (!cond) begin
            errors++;
            $display("  [%0t] FAIL (%s): %s", $time, phase, what);
        end
    endtask

    task automatic chk_i(input int got, input int exp, input string what);
        checks++;
        if (got !== exp) begin
            errors++;
            $display("  [%0t] FAIL (%s): %s -- got %0d, expected %0d",
                     $time, phase, what, got, exp);
        end
    endtask

    task automatic clear_counts();
        n_cls_cmd = 0; n_brst_cmd = 0; n_cls_valid = 0; n_cls_err = 0;
    endtask

    // -----------------------------------------------------------------
    // THE core invariant -- checked every cycle of the entire run
    // -----------------------------------------------------------------
    always @(posedge clk) begin
        if (rst_n) begin
            if (cls_cmd_valid && brst_cmd_valid) begin
                errors++;
                $display("  [%0t] FAIL (invariant/%s): BOTH command sources asserted",
                         $time, phase);
            end
            if (burst_active &&
                (cls_cmd_valid || classifier_valid || classifier_error)) begin
                errors++;
                $display("  [%0t] FAIL (invariant/%s): classifier active during a burst",
                         $time, phase);
            end
            if (bypass_active !== burst_active) begin
                errors++;
                $display("  [%0t] FAIL (invariant/%s): bypass_active != burst_active",
                         $time, phase);
            end
        end
    end

    // -----------------------------------------------------------------
    // Stimulus
    // -----------------------------------------------------------------
    task automatic step(input int n = 1); repeat (n) @(negedge clk); endtask

    task automatic send_byte(input logic [7:0] b);
        @(negedge clk); rx_byte = b; byte_valid = 1'b1;
        @(negedge clk); byte_valid = 1'b0;
        repeat (6) @(negedge clk);
    endtask

    task automatic send_burst_hdr(input int h, input int w);
        send_byte(CHAR_OPEN_BRACE); send_byte(CHAR_I);
        send_byte(8'h00); send_byte(8'h00); send_byte(8'h00);
        send_byte(CHAR_COMMA); send_byte(CHAR_H);
        send_byte(8'(h >> 16)); send_byte(8'(h >> 8)); send_byte(8'(h));
        send_byte(CHAR_COMMA); send_byte(CHAR_W);
        send_byte(8'(w >> 16)); send_byte(8'(w >> 8)); send_byte(8'(w));
        send_byte(CHAR_CLOSE_BRACE);
        step(10);
    endtask

    // Twelve arbitrary payload bytes, framed as burst data.
    task automatic send_burst_data(input logic [7:0] b [12]);
        send_byte(CHAR_OPEN_BRACE);
        send_byte(b[0]); send_byte(b[1]); send_byte(b[2]); send_byte(b[3]);
        send_byte(CHAR_COMMA);
        send_byte(b[4]); send_byte(b[5]); send_byte(b[6]); send_byte(b[7]);
        send_byte(CHAR_COMMA);
        send_byte(b[8]); send_byte(b[9]); send_byte(b[10]); send_byte(b[11]);
        send_byte(CHAR_CLOSE_BRACE);
        step(14);
    endtask

    task automatic send_pix_write(input logic [23:0] a, input logic [23:0] px);
        send_byte(CHAR_OPEN_BRACE); send_byte(CHAR_W);
        send_byte(a[23:16]); send_byte(a[15:8]); send_byte(a[7:0]);
        send_byte(CHAR_COMMA); send_byte(CHAR_P);
        send_byte(px[23:16]); send_byte(px[15:8]); send_byte(px[7:0]);
        send_byte(CHAR_CLOSE_BRACE);
        step(14);
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
        step(14);
    endtask

    // -----------------------------------------------------------------
    initial begin
        automatic logic [7:0] plain [12];
        automatic logic [7:0] spoof_pixwr [12];
        automatic logic [7:0] spoof_legacy [12];

        $display("=================================================");
        $display(" M4 REGRESSION: RX-side burst mode interface");
        $display("=================================================");

        rst_n = 1'b0; step(6); rst_n = 1'b1; step(6);
        clear_counts();

        // =============================================================
        banner("1 - bypass ownership and mode entry");
        // bypass_active must be low at rest, rise only on a valid header,
        // and be identical to burst_active since rx_burst_ctrl drives both.
        // =============================================================
        chk(!bypass_active, "bypass low at rest");
        chk(!burst_active,  "burst_active low at rest");

        send_burst_hdr(1, 4);
        chk(bypass_active,  "bypass high after a valid header");
        chk(burst_active,   "burst_active high after a valid header");

        // =============================================================
        banner("2 - normal burst data is classified as MSG_BURST_DATA");
        // =============================================================
        plain = '{8'h10,8'h11,8'h12, 8'h20,8'h21,8'h22,
                  8'h30,8'h31,8'h32, 8'h40,8'h41,8'h42};
        send_burst_data(plain);
        chk_i(n_brst_cmd, 4, "four burst commands from one frame");
        chk_i(n_cls_cmd,  0, "classifier emitted no command");
        chk(!bypass_active, "burst completed, bypass dropped");

        // =============================================================
        banner("3 - ADVERSARIAL: burst payload that spells other messages");
        // Payload bytes are raw binary. These two frames carry, byte for
        // byte, a perfect Single Pixel Write and a perfect legacy RGF
        // command -- inside a burst. bypass_active must force both to be
        // read as pixel data, and the classifier must stay silent.
        //
        // The legacy spoof is the dangerous one: without bypass it would
        // decode as a register write, and could address IMG_CTRL.
        // =============================================================
        clear_counts();
        send_burst_hdr(2, 4);

        //            R0     G0     B0     R1  |  G1     B1     R2     G2  |  B2     R3     G3     B3
        // as bytes:  1      2      3      4   |  6      7      8      9   |  11     12     13     14
        // A Single Pixel Write has 'W' at byte 1 and 'P' at byte 6.
        spoof_pixwr = '{CHAR_W, 8'h00, 8'h04, 8'h02,
                        CHAR_P, 8'hFF, 8'hA5, 8'h00,
                        8'h7D,  8'h11, 8'h22, 8'h33};
        send_burst_data(spoof_pixwr);
        chk_i(n_cls_cmd, 0,
              "spoofed pixel-write payload produced NO classifier command");
        chk_i(n_brst_cmd, 4, "it became four burst pixels instead");
        chk(bypass_active, "still in burst mode");

        // A legacy RGF command has 'R' at byte 1, 'C' at 6, 'V' at 11.
        spoof_legacy = '{CHAR_R, ASCII_ZERO+2, ASCII_ZERO, ASCII_ZERO,
                         CHAR_C, ASCII_ZERO,   ASCII_ZERO, ASCII_ZERO,
                         CHAR_V, ASCII_ZERO,   ASCII_ZERO, ASCII_ZERO+1};
        send_burst_data(spoof_legacy);
        chk_i(n_cls_valid, 0,
              "spoofed legacy payload produced NO RGF strobe");
        chk_i(n_cls_err, 0, "and no classifier error");
        chk_i(n_brst_cmd, 8, "it became four more burst pixels");
        chk(!bypass_active, "burst completed");

        // =============================================================
        banner("4 - single pixel write works normally outside a burst");
        // =============================================================
        clear_counts();
        send_pix_write(24'h00_0402, 24'hFF_A500);
        chk_i(n_cls_cmd,  1, "one classifier command outside a burst");
        chk_i(n_brst_cmd, 0, "burst controller silent");
        chk(cls_cmd_addr  === 24'h00_0402, "cmd_addr correct");
        chk(cls_cmd_pixel === 24'hFF_A500, "cmd_pixel correct");

        // =============================================================
        banner("5 - legacy RGF path preserved outside a burst");
        // =============================================================
        clear_counts();
        send_legacy(2, 0, 1);
        chk_i(n_cls_valid, 1, "one RGF strobe");
        chk_i(n_cls_cmd,   0, "no pixel command");
        chk_i(n_brst_cmd,  0, "no burst command");
        chk(row_q === 10'd2, "row_q latched");
        chk(pixel_q === 24'd1, "pixel_q latched");

        // =============================================================
        banner("6 - classifier error handling preserved");
        // A frame with valid framing but an unrecognised opcode must still
        // pulse classifier_error when no burst is active.
        // =============================================================
        clear_counts();
        send_byte(CHAR_OPEN_BRACE); send_byte(8'h5A);
        send_byte(8'h00); send_byte(8'h00); send_byte(8'h00);
        send_byte(CHAR_COMMA); send_byte(8'h5B);
        send_byte(8'h00); send_byte(8'h00); send_byte(8'h00);
        send_byte(CHAR_COMMA); send_byte(8'h5C);
        send_byte(8'h00); send_byte(8'h00); send_byte(8'h00);
        send_byte(CHAR_CLOSE_BRACE);
        step(14);
        chk_i(n_cls_err, 1, "unknown frame still pulses classifier_error");
        chk_i(n_cls_cmd, 0, "and emits no command");

        // =============================================================
        banner("7 - invalid header does not enter burst mode");
        // =============================================================
        clear_counts();
        send_burst_hdr(0, 4);                    // H = 0 is illegal
        chk(!bypass_active, "no burst started");
        chk(!burst_active,  "burst_active stayed low");

        // With bypass low, the next frame is opcode-decoded normally --
        // so a single pixel write still works right after a bad header.
        send_pix_write(24'h00_0001, 24'h11_2233);
        chk_i(n_cls_cmd, 1, "pixel write still works after a bad header");

        // =============================================================
        banner("8 - interleaved traffic across burst boundaries");
        // =============================================================
        clear_counts();
        send_legacy(4, 0, 1);
        send_pix_write(24'h00_0010, 24'hAA_BBCC);
        send_burst_hdr(1, 4);
        send_burst_data(plain);
        send_pix_write(24'h00_0011, 24'hDD_EEFF);
        send_legacy(1, 1, 0);
        chk_i(n_cls_cmd,   2, "two pixel writes, both outside the burst");
        chk_i(n_cls_valid, 2, "two RGF strobes, both outside the burst");
        chk_i(n_brst_cmd,  4, "four burst commands");
        chk(!bypass_active, "idle at the end");

        // =============================================================
        $display("-------------------------------------------------");
        $display(" checks executed : %0d", checks);
        $display(" errors          : %0d", errors);
        $display(" RESULT: %s", (errors == 0) ? "PASS" : "FAIL");
        $display("=================================================");
        if (errors != 0) $fatal(1, "tb_rx_burst_mode FAILED");
        $finish;
    end

    initial begin
        #10ms;
        $display(" RESULT: FAIL -- testbench timeout");
        $fatal(1, "tb_rx_burst_mode timed out");
    end

endmodule : tb_rx_burst_mode
