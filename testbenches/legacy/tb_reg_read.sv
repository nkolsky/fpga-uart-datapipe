// tb_reg_read.sv
// --------------
// Focused regression for Register Read reply support.
//
// Drives rx_reg_read_parser, rx_classifier and tx_reply_ctrl together with a
// behavioural tx_mac stand-in, so the request decode, the RGF command
// production, the reply framing and the TX arbitration are all exercised
// without needing the full chip.
//
// Covers, in order:
//   1  normal Register Read -- command out, reply framed correctly
//   2  reply byte order and value
//   3  address validation: out of range and unaligned are REJECTED
//   4  TX backpressure -- a reply survives an arbitrarily long stall
//   5  Register Read while image transmission is busy -- deferred, not lost
//   6  a second request while one is pending -- first preserved, overrun flagged
//   7  reset

`timescale 1ns/1ps

module tb_reg_read;

    import msg_pkg::*;
    import rx_msg_pkg::*;

    logic clk = 1'b0;
    always #3.846 clk = ~clk;           // 130 MHz receive / transmit domain
    logic rst_n;

    // ---- request path ------------------------------------------------
    logic [127:0] msg_in = '0;
    logic         rr_frame_ok, rr_addr_ok, rr_valid, rr_addr_err;
    logic [23:0]  rr_addr;
    logic [5:0]   rr_rgf_addr;

    rx_reg_read_parser u_parser (
        .msg_in(msg_in), .rr_frame_ok(rr_frame_ok), .rr_addr_ok(rr_addr_ok),
        .rr_valid(rr_valid), .rr_addr_err(rr_addr_err),
        .rr_addr(rr_addr), .rr_rgf_addr(rr_rgf_addr));

    logic      msg_valid = 1'b0;
    msg_kind_t msg_kind  = MSG_UNKNOWN;
    logic      cls_valid, cls_error, cls_cmd_valid;
    logic [9:0] row_q, col_q;
    logic [23:0] pixel_q, cls_cmd_addr, cls_cmd_pixel;
    logic      rr_cmd_valid;
    logic [5:0] rr_cmd_addr;

    rx_classifier u_cls (
        .clk(clk), .rst_n(rst_n),
        .msg_valid(msg_valid), .msg_kind(msg_kind),
        .parse_valid(1'b0), .parse_error(1'b0),
        .row(10'd0), .col(10'd0), .pixel(24'd0),
        .pw_parse_valid(1'b0), .pw_parse_error(1'b0),
        .pw_addr(24'd0), .pw_pixel(24'd0),
        .classifier_valid(cls_valid), .classifier_error(cls_error),
        .row_q(row_q), .col_q(col_q), .pixel_q(pixel_q),
        .cmd_valid(cls_cmd_valid), .cmd_addr(cls_cmd_addr),
        .cmd_pixel(cls_cmd_pixel),
        .burst_active(1'b0),
        .rr_valid(rr_valid), .rr_addr_err(rr_addr_err),
        .rr_rgf_addr(rr_rgf_addr),
        .rr_cmd_valid(rr_cmd_valid), .rr_cmd_addr(rr_cmd_addr));

    // ---- reply path ---------------------------------------------------
    logic        rd_valid = 1'b0;
    logic [31:0] rd_data  = '0;
    logic        tx_seq_busy = 1'b0, mac_busy = 1'b0;
    logic        reply_req, reply_pending, reply_sent, reply_overrun;
    logic [127:0] reply_msg;
    logic [4:0]  reply_len;

    tx_reply_ctrl u_reply (
        .clk(clk), .rst_n(rst_n),
        .rd_valid(rd_valid), .rd_data(rd_data),
        .tx_seq_busy(tx_seq_busy), .mac_busy(mac_busy),
        .reply_req(reply_req), .reply_msg(reply_msg), .reply_len(reply_len),
        .reply_pending(reply_pending), .reply_sent(reply_sent),
        .reply_overrun(reply_overrun));

    // Behavioural MAC: goes busy the cycle after it sees a request, stays
    // busy for `mac_cycles`, then idles -- the same handshake shape as the
    // real tx_mac.
    int  mac_cycles = 60;
    bit  mac_enable = 1'b1;
    logic [127:0] mac_seen_msg;
    logic [4:0]   mac_seen_len;
    int  mac_count = 0;

    initial forever begin
        @(posedge clk);
        if (rst_n && reply_req && mac_enable && !mac_busy) begin
            mac_seen_msg <= reply_msg;
            mac_seen_len <= reply_len;
            mac_count++;
            mac_busy <= 1'b1;
            repeat (mac_cycles) @(posedge clk);
            mac_busy <= 1'b0;
        end
    end

    // ---- bookkeeping ---------------------------------------------------
    int errors = 0, checks = 0;
    string phase = "init";
    int n_rr_cmd = 0, n_cls_err = 0;
    logic [5:0] last_rr_addr;

    always @(posedge clk) if (rst_n) begin
        if (rr_cmd_valid) begin n_rr_cmd++; last_rr_addr = rr_cmd_addr; end
        if (cls_error)    n_cls_err++;
    end

    task automatic banner(input string n); phase = n; $display("--- %s", n); endtask
    task automatic chk(input bit c, input string w);
        checks++;
        if (!c) begin errors++; $display("  [%0t] FAIL (%s): %s", $time, phase, w); end
    endtask
    task automatic chk_h(input logic [127:0] g, input logic [127:0] e,
                         input string w);
        checks++;
        if (g !== e) begin
            errors++;
            $display("  [%0t] FAIL (%s): %s -- got %032h expected %032h",
                     $time, phase, w, g, e);
        end
    endtask
    task automatic clr(); n_rr_cmd = 0; n_cls_err = 0; mac_count = 0; endtask
    task automatic step(input int n = 1); repeat (n) @(negedge clk); endtask

    // ---- stimulus ------------------------------------------------------
    task automatic build_req(input logic [23:0] a, input bit good_frame = 1'b1);
        msg_in = '0;
        msg_in[127:120] = good_frame ? CHAR_OPEN_BRACE  : 8'h00;
        msg_in[119:112] = CHAR_R;
        msg_in[111: 88] = a;
        msg_in[ 87: 80] = CHAR_CLOSE_BRACE;
        #1ps;
    endtask

    task automatic send_req(input logic [23:0] a, input bit good_frame = 1'b1);
        build_req(a, good_frame);
        @(negedge clk); msg_kind = MSG_REG_READ; msg_valid = 1'b1;
        @(negedge clk); msg_valid = 1'b0; msg_kind = MSG_UNKNOWN;
        step(3);
    endtask

    task automatic deliver(input logic [31:0] v);
        @(negedge clk); rd_data = v; rd_valid = 1'b1;
        @(negedge clk); rd_valid = 1'b0;
    endtask

    function automatic logic [127:0] exp_frame(input logic [31:0] v);
        return {80'd0, CHAR_CLOSE_BRACE, v[7:0], v[15:8], v[23:16], v[31:24],
                CHAR_OPEN_BRACE};
    endfunction

    // ---- continuous invariants -----------------------------------------
    // Written procedurally rather than as SVA: Verilator's support for
    // `|=> ... disable iff` is incomplete (it ICEs on the equivalent
    // property), so these are checked directly against sampled history and
    // are tool-independent. The SVA in tx_reply_ctrl.sv states the same
    // properties for simulators with full support.
    logic pend_d, driving_d, macbusy_d, sent_d;
    always @(posedge clk) begin
        pend_d    <= reply_pending;
        driving_d <= u_reply.driving;
        macbusy_d <= mac_busy;
        sent_d    <= reply_sent;
    end

    always @(posedge clk) if (rst_n) begin
        // 1. a reply is never offered while the transmit path is busy
        if (reply_req && (tx_seq_busy || mac_busy)) begin
            errors++;
            $display("  [%0t] FAIL (invariant/%s): reply offered while TX busy",
                     $time, phase);
        end
        // 2. pending may fall ONLY on a genuine MAC handoff
        if (pend_d && !reply_pending && !(driving_d && macbusy_d)) begin
            errors++;
            $display("  [%0t] FAIL (invariant/%s): pending cleared without a handoff",
                     $time, phase);
        end
        // 3. a queued reply is never silently dropped
        if (pend_d && !(driving_d && macbusy_d) && !reply_pending) begin
            errors++;
            $display("  [%0t] FAIL (invariant/%s): queued reply lost", $time, phase);
        end
    end

    initial begin
        $display("=================================================");
        $display(" REGRESSION: Register Read reply path");
        $display("=================================================");
        rst_n = 1'b0; step(5); rst_n = 1'b1; step(5); clr();

        // =============================================================
        banner("1 - normal Register Read");
        // IMG_TX_MON lives at byte address 0x04.
        send_req(24'h00_0004);
        chk(rr_frame_ok, "framing accepted");
        chk(rr_valid,    "address accepted");
        chk(!rr_addr_err, "no address error");
        chk(rr_rgf_addr === 6'h04, "rgf address = 0x04");
        chk(n_rr_cmd == 1, "exactly one RGF command produced");
        chk(last_rr_addr === 6'h04, "command carries the decoded address");
        chk(n_cls_err == 0, "no classifier error");
        chk(!cls_cmd_valid, "single-pixel command NOT produced");

        // =============================================================
        banner("2 - reply framing and byte order");
        // V3 is the MSB. 0xDEADBEEF must appear as DE AD BE EF on the wire.
        // Hold the MAC off so "queued" is a real observation rather than a
        // race against an immediately-available MAC, then release it.
        clr();
        mac_enable = 1'b0;
        deliver(32'hDEADBEEF);
        step(2);
        chk(reply_pending, "reply queued while the MAC is unavailable");
        mac_enable = 1'b1;
        wait (mac_count == 1);
        step(2);
        chk_h(mac_seen_msg, exp_frame(32'hDEADBEEF), "frame contents");
        chk(mac_seen_len === 5'd6, "length is 6 bytes");
        chk(mac_seen_msg[  7:  0] === CHAR_OPEN_BRACE,  "byte 0 = '{'");
        chk(mac_seen_msg[ 15:  8] === 8'hDE, "byte 1 = V3 = 0xDE");
        chk(mac_seen_msg[ 23: 16] === 8'hAD, "byte 2 = V2 = 0xAD");
        chk(mac_seen_msg[ 31: 24] === 8'hBE, "byte 3 = V1 = 0xBE");
        chk(mac_seen_msg[ 39: 32] === 8'hEF, "byte 4 = V0 = 0xEF");
        chk(mac_seen_msg[ 47: 40] === CHAR_CLOSE_BRACE, "byte 5 = '}'");
        wait (!mac_busy); step(4);
        chk(!reply_pending, "pending cleared after the MAC took it");

        // =============================================================
        banner("3 - address validation");
        // Out of range: anything above 0x3F cannot fit the 6-bit RGF port.
        clr();
        send_req(24'h00_0104);          // aliases to 0x04 if truncated
        chk(rr_frame_ok,  "framing still good");
        chk(!rr_valid,    "out-of-range address rejected");
        chk(rr_addr_err,  "address error raised");
        chk(n_rr_cmd == 0, "NO RGF command issued -- not truncated to 0x04");
        chk(n_cls_err == 1, "reported through classifier_error");

        clr();
        send_req(24'h00_0006);          // in range but not word aligned
        chk(!rr_valid,     "unaligned address rejected");
        chk(rr_addr_err,   "address error raised");
        chk(n_rr_cmd == 0, "no RGF command issued");

        clr();
        send_req(24'h00_0010);          // CLK_CTRL, legal
        chk(rr_valid, "0x10 accepted");
        chk(n_rr_cmd == 1, "command issued for a legal address");
        chk(last_rr_addr === 6'h10, "address 0x10");

        clr();
        send_req(24'h00_0004, 1'b0);    // corrupt opening brace
        chk(!rr_frame_ok, "bad framing rejected");
        chk(!rr_valid,    "not valid");
        chk(!rr_addr_err, "framing fault is not an address fault");
        chk(n_rr_cmd == 0, "no command issued");

        // =============================================================
        banner("4 - TX backpressure");
        // Hold the MAC off entirely, then release. The reply must survive
        // an arbitrary stall and be delivered intact -- never dropped,
        // never duplicated.
        clr();
        mac_enable = 1'b0;
        deliver(32'h0000_1234);
        step(60);
        chk(reply_pending, "reply still pending under backpressure");
        chk(mac_count == 0, "nothing sent while the MAC is unavailable");
        chk(reply_msg === exp_frame(32'h0000_1234), "held frame is stable");
        mac_enable = 1'b1;
        wait (mac_count == 1);
        step(2);
        chk_h(mac_seen_msg, exp_frame(32'h0000_1234), "delivered after release");
        wait (!mac_busy); step(4);
        chk(mac_count == 1, "delivered exactly once, no duplicate");
        chk(!reply_pending, "pending cleared");

        // =============================================================
        banner("5 - Register Read while image transmission is busy");
        // tx_seq_busy high means an image transfer is running. The reply
        // must be held until it finishes, so the host's 65536-packet stream
        // is never interrupted.
        clr();
        @(negedge clk); tx_seq_busy = 1'b1;
        deliver(32'hCAFE_0001);
        step(80);
        chk(reply_pending, "reply queued during the image transfer");
        chk(mac_count == 0, "NOT inserted into the image stream");
        chk(!reply_req, "reply not offered while tx_seq_busy");

        @(negedge clk); tx_seq_busy = 1'b0;      // transfer ends
        wait (mac_count == 1);
        step(2);
        chk_h(mac_seen_msg, exp_frame(32'hCAFE_0001),
              "sent once the image transfer finished");
        wait (!mac_busy); step(4);

        // =============================================================
        banner("6 - second request while one is pending");
        // The first answer is preserved and the collision is flagged.
        // Overwriting would discard a value the host is still waiting for.
        clr();
        mac_enable = 1'b0;
        deliver(32'h1111_1111);
        step(4);
        chk(reply_pending,   "first reply queued");
        chk(!reply_overrun,  "no overrun yet");

        deliver(32'h2222_2222);          // second, while the first waits
        step(4);
        chk(reply_overrun,   "overrun flagged");
        chk(reply_msg === exp_frame(32'h1111_1111),
            "FIRST value preserved, not overwritten");

        mac_enable = 1'b1;
        wait (mac_count == 1); step(2);
        chk_h(mac_seen_msg, exp_frame(32'h1111_1111), "first value delivered");
        wait (!mac_busy); step(6);
        chk(mac_count == 1, "the rejected second reply was not sent");
        chk(!reply_pending, "idle again");

        // =============================================================
        banner("7 - reset");
        clr();
        mac_enable = 1'b0;
        deliver(32'hFFFF_FFFF);
        step(4);
        chk(reply_pending, "reply pending before reset");

        @(negedge clk); rst_n = 1'b0;
        step(4);
        chk(!reply_pending, "reset cleared the pending reply");
        chk(!reply_req,     "reply_req low in reset");
        chk(!reply_overrun, "reset cleared the overrun flag");
        @(negedge clk); rst_n = 1'b1; mac_enable = 1'b1;
        step(4);

        clr();
        send_req(24'h00_0008);           // IMG_CTRL
        chk(n_rr_cmd == 1, "requests work again after reset");
        deliver(32'h0000_00AA);
        wait (mac_count == 1); step(2);
        chk_h(mac_seen_msg, exp_frame(32'h0000_00AA), "replies work after reset");

        // =============================================================
        $display("-------------------------------------------------");
        $display(" checks executed : %0d", checks);
        $display(" errors          : %0d", errors);
        $display(" RESULT: %s", (errors == 0) ? "PASS" : "FAIL");
        $display("=================================================");
        if (errors != 0) $fatal(1, "tb_reg_read FAILED");
        $finish;
    end

    initial begin
        #5ms;
        $display(" RESULT: FAIL -- testbench timeout");
        $fatal(1, "tb_reg_read timed out");
    end

endmodule : tb_reg_read
