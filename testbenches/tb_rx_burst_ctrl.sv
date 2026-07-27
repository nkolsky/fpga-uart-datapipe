// tb_rx_burst_ctrl.sv
// -------------------
// M3 regression for rx_burst_ctrl. Self-checking, no design changes.
//
// The controller is driven at its own interface -- msg_valid / msg_kind plus
// already-parsed header and pixel fields -- rather than through rx_mac and
// the parsers. Those are verified by M1 and M2; driving them again here
// would test the same logic twice and make failures harder to localise.
//
// -----------------------------------------------------------------------
// EXPECTED COMMAND TIMING (asserted throughout)
// -----------------------------------------------------------------------
//   cycle  state      slot  cmd_valid during
//   N      B_ACTIVE    -      0     data frame accepted, pixels latched
//   N+1    B_EMIT      0      0
//   N+2    B_EMIT      1      1     <- slot 0
//   N+3    B_EMIT      2      1     <- slot 1
//   N+4    B_EMIT      3      1     <- slot 2
//   N+5    B_ACTIVE    -      1     <- slot 3
//
// -----------------------------------------------------------------------
// THE GOLDEN MODEL
// -----------------------------------------------------------------------
// Expected addresses are generated independently from the rectangle
// geometry, not by mirroring the DUT's counters:
//
//     row  = idx / W ;  col = idx % W
//     addr = (base_row + row) * IMG_WIDTH + (base_col + col)
//
// Division and modulo are used deliberately -- the DUT uses incremental
// counters and a wrap comparison, so an error in either is visible as a
// disagreement rather than being reproduced on both sides.

`timescale 1ns/1ps

module tb_rx_burst_ctrl;

    import memory_pkg::*;
    import rx_msg_pkg::*;
    import rx_burst_pkg::*;

    localparam int IMG_W = IMG_WIDTH;    // 256
    localparam int IMG_H = IMG_HEIGHT;   // 256

    // -----------------------------------------------------------------
    // Clock / DUT
    // -----------------------------------------------------------------
    logic clk = 1'b0;
    always #3.846 clk = ~clk;            // 130 MHz receive domain

    logic      rst_n;
    logic      msg_valid = 1'b0;
    msg_kind_t msg_kind  = MSG_UNKNOWN;

    logic                   hdr_valid = 1'b0;
    logic [BURST_DIM_W-1:0] height    = '0;
    logic [BURST_DIM_W-1:0] width     = '0;

    logic                                                 data_frame_ok = 1'b0;
    logic [BURST_PIX_PER_MSG-1:0][BURST_PIX_W-1:0]        pixels        = '0;

    logic burst_abort = 1'b0;
    logic cmd_ready   = 1'b1;

    logic                    cmd_valid;
    logic [BURST_ADDR_W-1:0] cmd_addr;
    logic [BURST_PIX_W-1:0]  cmd_pixel;
    logic                    bypass_active, burst_active, burst_done;
    logic                    err_hdr_invalid, err_data_invalid, err_unexpected;

    rx_burst_ctrl dut (
        .clk(clk), .rst_n(rst_n),
        .msg_valid(msg_valid), .msg_kind(msg_kind),
        .hdr_valid(hdr_valid), .height(height), .width(width),
        .data_frame_ok(data_frame_ok), .pixels(pixels),
        .burst_abort(burst_abort),
        .cmd_ready(cmd_ready),
        .cmd_valid(cmd_valid), .cmd_addr(cmd_addr), .cmd_pixel(cmd_pixel),
        .bypass_active(bypass_active), .burst_active(burst_active),
        .burst_done(burst_done),
        .err_hdr_invalid(err_hdr_invalid),
        .err_data_invalid(err_data_invalid),
        .err_unexpected(err_unexpected)
    );

    // -----------------------------------------------------------------
    // Bookkeeping
    // -----------------------------------------------------------------
    int    errors = 0;
    int    checks = 0;
    string phase  = "init";

    typedef struct {
        logic [BURST_ADDR_W-1:0] addr;
        logic [BURST_PIX_W-1:0]  pix;
    } cmd_t;

    cmd_t got_q [$];
    int   n_cmd = 0, n_done = 0;
    int   n_err_hdr = 0, n_err_data = 0, n_err_unexp = 0;

    // A command is COUNTED ONLY WHEN ACCEPTED. Counting every cmd_valid
    // would measure what the controller presented rather than what the FIFO
    // took, and would therefore be blind to a command being held across
    // several cycles -- or, in the earlier design, to one being dropped.
    cmd_t cap;
    always @(posedge clk) begin
        if (rst_n) begin
            if (cmd_valid && cmd_ready) begin
                cap.addr = cmd_addr;  cap.pix = cmd_pixel;
                got_q.push_back(cap);
                n_cmd++;
            end
            if (burst_done)       n_done++;
            if (err_hdr_invalid)  n_err_hdr++;
            if (err_data_invalid) n_err_data++;
            if (err_unexpected)   n_err_unexp++;
        end
    end

    task automatic banner(input string name);
        phase = name;
        $display("--- %s", name);
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
        got_q.delete();
        n_cmd = 0; n_done = 0;
        n_err_hdr = 0; n_err_data = 0; n_err_unexp = 0;
    endtask

    // -----------------------------------------------------------------
    // Golden model -- independent of the DUT's counter structure
    // -----------------------------------------------------------------
    function automatic int exp_addr(input int idx, input int w,
                                    input int b_row = 0, input int b_col = 0);
        int r, c;
        r = idx / w;
        c = idx % w;
        return (b_row + r) * IMG_W + (b_col + c);
    endfunction

    // Deterministic, easily-read pixel values: 0x100000 + index.
    function automatic logic [23:0] exp_pix(input int idx);
        return BURST_PIX_W'(24'h10_0000 + idx);
    endfunction

    // -----------------------------------------------------------------
    // Stimulus
    // -----------------------------------------------------------------
    task automatic step(input int n = 1);
        repeat (n) @(negedge clk);
    endtask

    task automatic do_reset();
        rst_n         = 1'b0;
        msg_valid     = 1'b0;
        msg_kind      = MSG_UNKNOWN;
        hdr_valid     = 1'b0;
        data_frame_ok = 1'b0;
        burst_abort   = 1'b0;
        cmd_ready     = 1'b1;
        step(4);
        rst_n = 1'b1;
        step(3);
        clear_counts();
    endtask

    task automatic send_header(input int h, input int w, input bit valid = 1'b1);
        @(negedge clk);
        msg_kind  = MSG_BURST_HDR;
        height    = BURST_DIM_W'(h);
        width     = BURST_DIM_W'(w);
        hdr_valid = valid;
        msg_valid = 1'b1;
        @(negedge clk);
        msg_valid = 1'b0;
        msg_kind  = MSG_UNKNOWN;
    endtask

    // Present one data frame carrying pixels for indices base..base+3.
    task automatic send_data(input int first_idx, input bit ok = 1'b1);
        @(negedge clk);
        for (int k = 0; k < BURST_PIX_PER_MSG; k++)
            pixels[k] = exp_pix(first_idx + k);
        data_frame_ok = ok;
        msg_kind      = MSG_BURST_DATA;
        msg_valid     = 1'b1;
        @(negedge clk);
        msg_valid     = 1'b0;
        msg_kind      = MSG_UNKNOWN;
    endtask

    // Run a complete burst of h x w and check every command.
    task automatic run_burst(input int h, input int w, input string tag);
        automatic int total  = h * w;
        automatic int frames = (total + BURST_PIX_PER_MSG - 1) / BURST_PIX_PER_MSG;

        clear_counts();
        send_header(h, w);
        step(2);
        chk(bypass_active, {tag, ": bypass_active after header"});
        chk(burst_active,  {tag, ": burst_active after header"});

        for (int f = 0; f < frames; f++) begin
            send_data(f * BURST_PIX_PER_MSG);
            step(10);                       // serialise + inter-frame gap
        end
        step(10);

        chk_i(n_cmd,  total, {tag, ": command count"});
        chk_i(n_done, 1,     {tag, ": one burst_done pulse"});
        chk(!bypass_active,  {tag, ": bypass_active cleared"});
        chk(!burst_active,   {tag, ": burst_active cleared"});
        chk_i(n_err_data, 0, {tag, ": no data errors"});

        for (int i = 0; i < total && i < got_q.size(); i++) begin
            chk(got_q[i].addr === BURST_ADDR_W'(exp_addr(i, w)),
                $sformatf("%s: cmd %0d addr = %0d, expected %0d",
                          tag, i, got_q[i].addr, exp_addr(i, w)));
            chk(got_q[i].pix === exp_pix(i),
                $sformatf("%s: cmd %0d pixel = %06h, expected %06h",
                          tag, i, got_q[i].pix, exp_pix(i)));
        end
    endtask

    // -----------------------------------------------------------------
    // Continuous invariants
    // -----------------------------------------------------------------
    always @(posedge clk) begin
        if (rst_n) begin
            if (bypass_active !== burst_active) begin
                errors++;
                $display("  [%0t] FAIL (invariant/%s): bypass_active != burst_active",
                         $time, phase);
            end
            // A presented command must never be withdrawn or altered
            // before it is accepted. Checked every cycle of the whole run,
            // so a regression anywhere trips it, not just in scenario 16.
            // While a command is on the interface the mode outputs must
            // stay high, or chip_top's mux would withdraw it mid-handshake.
            if (cmd_valid && !burst_abort && (!bypass_active || !burst_active)) begin
                errors++;
                $display("  [%0t] FAIL (invariant/%s): mode dropped with a command pending",
                         $time, phase);
            end
            // burst_done must never coincide with a pending command.
            if (burst_done && cmd_valid) begin
                errors++;
                $display("  [%0t] FAIL (invariant/%s): burst_done with a command pending",
                         $time, phase);
            end
            if ($past(cmd_valid) && !$past(cmd_ready) && !$past(burst_abort)) begin
                if (!cmd_valid) begin
                    errors++;
                    $display("  [%0t] FAIL (invariant/%s): cmd_valid withdrawn without acceptance",
                             $time, phase);
                end
                if (cmd_addr !== $past(cmd_addr) || cmd_pixel !== $past(cmd_pixel)) begin
                    errors++;
                    $display("  [%0t] FAIL (invariant/%s): pending command changed while not ready",
                             $time, phase);
                end
            end
        end
    end

    // -----------------------------------------------------------------
    initial begin
        $display("=================================================");
        $display(" M3 REGRESSION: rx_burst_ctrl");
        $display(" IMG_WIDTH=%0d IMG_HEIGHT=%0d", IMG_W, IMG_H);
        $display("=================================================");

        do_reset();
        chk(!bypass_active, "idle after reset");
        chk(!burst_active,  "burst_active low after reset");
        chk(!cmd_valid,     "no command after reset");

        // =============================================================
        banner("1 - 1x1 burst (three padding slots)");
        // One real pixel, then slots 1..3 are host padding: no commands,
        // no position advance, clean completion.
        // =============================================================
        run_burst(1, 1, "1x1");
        chk_i(n_cmd, 1, "1x1: exactly one command");

        // =============================================================
        banner("2 - 2x2 burst (one full frame)");
        // Exactly four pixels: one frame, all four slots real.
        // Addresses (0,0)(0,1)(1,0)(1,1) -> 0, 1, 256, 257.
        // =============================================================
        run_burst(2, 2, "2x2");
        chk(got_q[0].addr === 24'd0,   "2x2: addr[0] = 0");
        chk(got_q[1].addr === 24'd1,   "2x2: addr[1] = 1");
        chk(got_q[2].addr === 24'd256, "2x2: addr[2] = 256 (row wrap)");
        chk(got_q[3].addr === 24'd257, "2x2: addr[3] = 257");

        // =============================================================
        banner("3 - 2x3 burst (two frames, partial second)");
        // Six pixels across two frames. Row wraps after column 2, and the
        // second frame's last two slots are padding.
        //
        //   idx 0 1 2 -> (0,0)(0,1)(0,2) -> 0, 1, 2
        //   idx 3 4 5 -> (1,0)(1,1)(1,2) -> 256, 257, 258
        // =============================================================
        run_burst(2, 3, "2x3");
        chk_i(n_cmd, 6, "2x3: exactly six commands, padding suppressed");
        chk(got_q[2].addr === 24'd2,   "2x3: addr[2] = 2 (end of row 0)");
        chk(got_q[3].addr === 24'd256, "2x3: addr[3] = 256 (row wrap)");
        chk(got_q[5].addr === 24'd258, "2x3: addr[5] = 258 (last pixel)");

        // =============================================================
        banner("4 - full-width row boundary");
        // W = IMG_WIDTH exercises the wrap at the true end of a row: the
        // 256th pixel is address 255 and the 257th is 256.
        // =============================================================
        run_burst(2, IMG_W, "2x256");
        chk_i(n_cmd, 2*IMG_W, "2x256: command count");
        chk(got_q[IMG_W-1].addr === BURST_ADDR_W'(IMG_W-1),
            "2x256: last pixel of row 0 = 255");
        chk(got_q[IMG_W].addr === BURST_ADDR_W'(IMG_W),
            "2x256: first pixel of row 1 = 256");

        // =============================================================
        banner("5 - multiple frames, no duplicates or omissions");
        // 3x5 = 15 pixels over four frames, the last with one padding slot.
        // run_burst checks every address and pixel individually, so a
        // duplicate or a dropped pixel shows up as a value mismatch as well
        // as a count mismatch.
        // =============================================================
        run_burst(3, 5, "3x5");
        chk_i(n_cmd, 15, "3x5: exactly fifteen commands");

        run_burst(4, 4, "4x4");
        chk_i(n_cmd, 16, "4x4: exactly sixteen commands, no padding");

        // =============================================================
        banner("6 - command timing relative to msg_valid");
        // The precise relationship the M5 mux depends on: first cmd_valid
        // two cycles after the data frame, then four consecutive cycles.
        // =============================================================
        clear_counts();
        send_header(1, 4);
        step(3);

        @(negedge clk);
        for (int k = 0; k < BURST_PIX_PER_MSG; k++) pixels[k] = exp_pix(k);
        data_frame_ok = 1'b1;
        msg_kind      = MSG_BURST_DATA;
        msg_valid     = 1'b1;             // cycle N
        @(negedge clk);
        msg_valid = 1'b0; msg_kind = MSG_UNKNOWN;
        // cmd_ready is high throughout, so every cmd_valid cycle is also
        // an accepted transfer and the timing is the documented contract.
        chk(!cmd_valid, "timing: no command at N+1");
        @(negedge clk);
        chk(cmd_valid && cmd_ready, "timing: transfer at N+2 (slot 0)");
        @(negedge clk);
        chk(cmd_valid && cmd_ready, "timing: transfer at N+3 (slot 1)");
        @(negedge clk);
        chk(cmd_valid && cmd_ready, "timing: transfer at N+4 (slot 2)");
        @(negedge clk);
        chk(cmd_valid && cmd_ready, "timing: transfer at N+5 (slot 3)");
        @(negedge clk);
        chk(!cmd_valid, "timing: no command at N+6");
        step(6);
        chk_i(n_cmd, 4, "timing: four commands total");

        // =============================================================
        banner("7 - bypass_active / burst_active lifetime");
        // Must assert after a valid header, survive the gaps between data
        // frames, and drop only after the final real pixel.
        // =============================================================
        clear_counts();
        chk(!bypass_active, "bypass low before header");
        send_header(2, 4);
        step(2);
        chk(bypass_active, "bypass high after header");

        send_data(0);
        step(30);                          // long inter-frame gap
        chk(bypass_active, "bypass stays high across the gap");
        chk_i(n_cmd, 4, "four commands from frame 1");

        send_data(4);
        step(12);
        chk(!bypass_active, "bypass drops after the final pixel");
        chk_i(n_cmd, 8, "eight commands total");
        chk_i(n_done, 1, "one completion pulse");

        // =============================================================
        banner("8 - malformed header does not start a burst");
        // =============================================================
        clear_counts();
        send_header(2, 2, 1'b0);           // hdr_valid low
        step(6);
        chk(!bypass_active, "no burst started");
        chk_i(n_err_hdr, 1, "err_hdr_invalid pulsed once");
        chk_i(n_cmd, 0, "no commands emitted");

        // Data arriving with no burst in progress is unexpected traffic.
        send_data(0);
        step(6);
        chk_i(n_err_unexp, 1, "err_unexpected on data while idle");
        chk_i(n_cmd, 0, "still no commands");

        // =============================================================
        banner("9 - malformed data frame");
        // No commands, an error pulse, and the burst stays armed at the
        // same position so a subsequent good frame continues correctly.
        // =============================================================
        clear_counts();
        send_header(1, 4);
        step(3);
        send_data(0, 1'b0);                // data_frame_ok low
        step(10);
        chk_i(n_cmd, 0, "malformed data emitted no commands");
        chk_i(n_err_data, 1, "err_data_invalid pulsed once");
        chk(bypass_active, "burst still armed after a bad frame");

        send_data(0, 1'b1);                // good frame, same pixels
        step(12);
        chk_i(n_cmd, 4, "position preserved: four commands");
        chk(got_q[0].addr === 24'd0, "resumed at address 0");
        chk(!bypass_active, "burst completed");

        // =============================================================
        banner("10 - burst_abort while waiting for data");
        // =============================================================
        clear_counts();
        send_header(4, 4);
        step(3);
        send_data(0);
        step(12);
        chk_i(n_cmd, 4, "four commands before abort");
        chk(bypass_active, "still active");

        @(negedge clk); burst_abort = 1'b1;
        @(negedge clk); burst_abort = 1'b0;
        step(4);
        chk(!bypass_active, "abort in B_ACTIVE returned to idle");
        chk(!burst_active,  "burst_active cleared by abort");
        chk_i(n_cmd, 4, "no further commands after abort");

        // A header is accepted again afterwards -- the FSM is not wedged.
        send_header(1, 1);
        step(2);
        chk(bypass_active, "new burst accepted after abort");
        send_data(0);
        step(12);
        chk(!bypass_active, "and completes normally");

        // =============================================================
        banner("11 - burst_abort mid-serialisation");
        // Commands already registered still complete; nothing further is
        // emitted, and the mode outputs drop in the same cycle.
        // =============================================================
        clear_counts();
        send_header(4, 4);
        step(3);

        @(negedge clk);
        for (int k = 0; k < BURST_PIX_PER_MSG; k++) pixels[k] = exp_pix(k);
        data_frame_ok = 1'b1; msg_kind = MSG_BURST_DATA; msg_valid = 1'b1;
        @(negedge clk);
        msg_valid = 1'b0; msg_kind = MSG_UNKNOWN;
        @(negedge clk);                    // N+2, slot 0 out
        @(negedge clk);                    // N+3, slot 1 out
        burst_abort = 1'b1;                // abort mid-serialisation
        @(negedge clk);
        burst_abort = 1'b0;
        step(8);
        chk(!bypass_active, "abort in B_EMIT returned to idle");
        chk(n_cmd < BURST_PIX_PER_MSG,
            $sformatf("serialisation truncated (%0d of 4 commands)", n_cmd));

        // =============================================================
        banner("12 - reset during an active burst");
        // =============================================================
        clear_counts();
        send_header(4, 4);
        step(3);
        send_data(0);
        step(4);
        chk(bypass_active, "burst active before reset");

        @(negedge clk); rst_n = 1'b0;
        step(3);
        chk(!bypass_active, "reset cleared bypass_active");
        chk(!burst_active,  "reset cleared burst_active");
        chk(!cmd_valid,     "reset cleared cmd_valid");
        @(negedge clk); rst_n = 1'b1;
        step(3);

        clear_counts();
        run_burst(2, 2, "post-reset");
        chk_i(n_cmd, 4, "operates normally after reset");

        // =============================================================
        banner("13 - cmd_ready backpressure");
        // The controller must never assume the FIFO can take four commands
        // in one cycle: with cmd_ready low it stalls in place, then emits
        // every pixel once ready returns. No pixel is lost or duplicated.
        // =============================================================
        clear_counts();
        send_header(1, 4);
        step(3);

        @(negedge clk); cmd_ready = 1'b0;  // FIFO full before the frame
        send_data(0);
        step(20);
        chk_i(n_cmd, 0, "no commands while cmd_ready low");
        chk(bypass_active, "burst still active while stalled");

        @(negedge clk); cmd_ready = 1'b1;
        step(15);
        chk_i(n_cmd, 4, "all four commands emitted after the stall");
        for (int i = 0; i < 4; i++)
            chk(got_q[i].pix === exp_pix(i),
                $sformatf("stall: pixel %0d preserved in order", i));
        chk(!bypass_active, "burst completed after the stall");

        // Intermittent readiness mid-frame.
        clear_counts();
        send_header(2, 4);
        step(3);
        send_data(0);
        @(negedge clk); @(negedge clk);
        cmd_ready = 1'b0;                  // stall mid-serialisation
        step(6);
        cmd_ready = 1'b1;
        step(15);
        send_data(4);
        step(15);
        chk_i(n_cmd, 8, "intermittent stall: all eight commands");
        for (int i = 0; i < 8 && i < got_q.size(); i++)
            chk(got_q[i].addr === BURST_ADDR_W'(exp_addr(i, 4)),
                $sformatf("intermittent stall: addr %0d correct", i));

        // =============================================================
        banner("14 - header while already in burst mode");
        // Defensive. In the real system bypass_active makes rx_msg_decode
        // classify every frame as MSG_BURST_DATA, so this cannot occur --
        // but if it does it must not restart the burst.
        // =============================================================
        clear_counts();
        send_header(2, 4);
        step(3);
        send_data(0);
        step(12);
        chk_i(n_cmd, 4, "first frame emitted");

        send_header(8, 8);                 // spurious header mid-burst
        step(6);
        chk_i(n_err_unexp, 1, "err_unexpected pulsed");
        chk(bypass_active, "burst not restarted, still active");

        send_data(4);
        step(12);
        chk_i(n_cmd, 8, "original 2x4 geometry preserved");
        chk(!bypass_active, "completed on the original dimensions");

        // =============================================================
        banner("16 - readiness drops immediately after a command is valid");
        // The near-full race that motivated the ready/valid rework.
        //
        // cmd_ready is dropped in the FIRST cycle a command is presented and
        // held low for several cycles. Under the earlier design the
        // controller had already advanced past that command, so it would
        // have been lost the moment the FIFO refused it. Here it must be
        // held bit-stable and delivered intact once readiness returns.
        // =============================================================
        clear_counts();
        send_header(1, 4);
        step(3);

        @(negedge clk);
        for (int k = 0; k < BURST_PIX_PER_MSG; k++) pixels[k] = exp_pix(k);
        data_frame_ok = 1'b1; msg_kind = MSG_BURST_DATA; msg_valid = 1'b1;
        @(negedge clk);
        msg_valid = 1'b0; msg_kind = MSG_UNKNOWN;   // N+1: slot 0 loaded

        @(negedge clk);                              // N+2: slot 0 presented
        chk(cmd_valid, "slot 0 presented at N+2");
        cmd_ready = 1'b0;                            // refuse it immediately

        begin : hold_check
            automatic logic [BURST_ADDR_W-1:0] held_addr = cmd_addr;
            automatic logic [BURST_PIX_W-1:0]  held_pix  = cmd_pixel;

            for (int c = 0; c < 8; c++) begin
                @(negedge clk);
                chk(cmd_valid, $sformatf("held: cmd_valid still high (+%0d)", c));
                chk(cmd_addr  === held_addr,
                    $sformatf("held: cmd_addr stable (+%0d)", c));
                chk(cmd_pixel === held_pix,
                    $sformatf("held: cmd_pixel stable (+%0d)", c));
            end
            chk_i(n_cmd, 0, "nothing accepted while not ready");

            @(negedge clk); cmd_ready = 1'b1;        // release
            step(12);
            chk_i(n_cmd, 4, "all four pixels delivered after the hold");
            chk(got_q[0].pix === held_pix,
                "the held command was delivered, not skipped");
        end : hold_check

        for (int i = 0; i < 4 && i < got_q.size(); i++) begin
            chk(got_q[i].addr === BURST_ADDR_W'(exp_addr(i, 4)),
                $sformatf("hold: addr %0d correct", i));
            chk(got_q[i].pix === exp_pix(i),
                $sformatf("hold: pixel %0d correct and in order", i));
        end
        chk(!bypass_active, "burst completed after the hold");

        // Single-cycle refusals on every slot boundary -- the pattern a
        // nearly-full FIFO actually produces.
        clear_counts();
        send_header(2, 4);
        step(3);
        send_data(0);
        for (int c = 0; c < 12; c++) begin
            @(negedge clk);
            cmd_ready = ~cmd_ready;                  // toggle every cycle
        end
        cmd_ready = 1'b1;
        step(15);
        send_data(4);
        step(20);
        chk_i(n_cmd, 8, "toggling readiness: all eight accepted exactly once");
        for (int i = 0; i < 8 && i < got_q.size(); i++)
            chk(got_q[i].pix === exp_pix(i),
                $sformatf("toggling: pixel %0d in order", i));

        // =============================================================
        banner("17 - readiness drops on the FINAL command");
        // The integration race that motivated exiting B_EMIT on fire rather
        // than on load.
        //
        // A 1x4 burst puts the last real pixel in slot 3, so the old design
        // transitioned to B_DONE in the same cycle it loaded that command.
        // With cmd_ready low it would then have reached B_IDLE with the
        // command still pending -- bypass_active low, chip_top's mux
        // switched away, the pixel silently withdrawn, and burst_done
        // already pulsed claiming success.
        // =============================================================
        clear_counts();
        send_header(1, 4);
        step(3);

        @(negedge clk);                              // cycle N
        for (int k = 0; k < BURST_PIX_PER_MSG; k++) pixels[k] = exp_pix(k);
        data_frame_ok = 1'b1; msg_kind = MSG_BURST_DATA; msg_valid = 1'b1;
        @(negedge clk);                              // N+1  slot0 loaded
        msg_valid = 1'b0; msg_kind = MSG_UNKNOWN;
        @(negedge clk);                              // N+2  slot0 presented
        @(negedge clk);                              // N+3  slot1
        @(negedge clk);                              // N+4  slot2
        @(negedge clk);                              // N+5  slot3 = FINAL

        chk(cmd_valid, "final command presented at N+5");
        chk_i(n_cmd, 3, "three commands accepted so far");
        cmd_ready = 1'b0;                            // refuse the final one

        begin : final_hold
            automatic logic [BURST_ADDR_W-1:0] fin_addr = cmd_addr;
            automatic logic [BURST_PIX_W-1:0]  fin_pix  = cmd_pixel;

            chk(fin_pix === exp_pix(3), "the held command is the final pixel");

            for (int c = 0; c < 8; c++) begin
                @(negedge clk);
                chk(cmd_valid,   $sformatf("final hold +%0d: cmd_valid high", c));
                chk(cmd_addr  === fin_addr,
                    $sformatf("final hold +%0d: cmd_addr stable", c));
                chk(cmd_pixel === fin_pix,
                    $sformatf("final hold +%0d: cmd_pixel stable", c));
                chk(bypass_active,
                    $sformatf("final hold +%0d: bypass_active still high", c));
                chk(burst_active,
                    $sformatf("final hold +%0d: burst_active still high", c));
                chk(!burst_done,
                    $sformatf("final hold +%0d: burst_done still low", c));
                chk_i(n_cmd, 3,
                    "final hold: still three accepted, nothing lost or duplicated");
            end

            // Release. The final command must fire exactly once, and only
            // then may burst_done pulse and the mode outputs drop.
            @(negedge clk); cmd_ready = 1'b1;
            @(negedge clk);
            chk_i(n_cmd, 4, "final command fired exactly once on release");

            step(6);
            chk_i(n_cmd,  4, "still exactly four -- no duplicate");
            chk_i(n_done, 1, "burst_done pulsed exactly once");
            chk(!bypass_active, "bypass_active dropped after the final fire");
            chk(!burst_active,  "burst_active dropped after the final fire");

            chk(got_q[3].pix  === fin_pix,
                "the delivered final pixel is the one that was held");
            chk(got_q[3].addr === fin_addr,
                "the delivered final address is the one that was held");
        end : final_hold

        // Same race with a PARTIAL final frame. 1x3 leaves the last real
        // pixel in slot 2 with slot 3 as padding, so the FSM reaches B_DONE
        // through the padding walk rather than directly from the last slot.
        // That walk must not carry it past a still-pending command either.
        //
        // PHASE ALIGNMENT -- this is easy to get wrong. send_data() returns
        // at the negedge of N+1, and for a 1x3 burst the final real command
        // is presented during N+4:
        //
        //   cycle  state     slot  action                        cmd_valid
        //   N      B_ACTIVE   -    latch pixels, slot<=0             0
        //   N+1    B_EMIT     0    load slot0, slot<=1               0
        //   N+2    B_EMIT     1    fire slot0, load slot1            1  slot0
        //   N+3    B_EMIT     2    fire slot1, load slot2, done<=1   1  slot1
        //   N+4    B_EMIT     3    fire slot2   <-- FINAL REAL       1  slot2
        //   N+5    B_EMIT     4    padding walked, exit              0
        //   N+6    B_DONE     -    burst_done                        0
        //
        // so exactly THREE negedges after send_data() reach the cycle in
        // which slot 2 is first presented, before its accepting posedge.
        // Stepping further would let it fire and complete the burst, and the
        // hold would then be testing nothing.
        clear_counts();
        send_header(1, 3);
        step(3);
        send_data(0);                                // returns at negedge N+1

        @(negedge clk);                              // N+2  slot0 presented
        @(negedge clk);                              // N+3  slot1 presented
        @(negedge clk);                              // N+4  slot2 presented

        // Confirm the phase before refusing, so a future timing change
        // fails here loudly instead of silently testing the wrong cycle.
        chk(cmd_valid,                "partial: final command presented");
        chk(cmd_pixel === exp_pix(2), "partial: it is pixel 2");
        chk_i(n_cmd, 2,               "partial: two accepted so far");
        chk(bypass_active,            "partial: bypass high at presentation");

        cmd_ready = 1'b0;                            // refuse before its posedge

        begin : partial_hold
            automatic logic [BURST_ADDR_W-1:0] fin_addr = cmd_addr;
            automatic logic [BURST_PIX_W-1:0]  fin_pix  = cmd_pixel;

            for (int c = 0; c < 6; c++) begin
                @(negedge clk);
                chk(cmd_valid,   $sformatf("partial hold +%0d: cmd_valid high", c));
                chk(cmd_addr  === fin_addr,
                    $sformatf("partial hold +%0d: cmd_addr stable", c));
                chk(cmd_pixel === fin_pix,
                    $sformatf("partial hold +%0d: cmd_pixel stable", c));
                chk(bypass_active,
                    $sformatf("partial hold +%0d: bypass_active high", c));
                chk(burst_active,
                    $sformatf("partial hold +%0d: burst_active high", c));
                chk(!burst_done,
                    $sformatf("partial hold +%0d: burst_done low", c));
                chk_i(n_cmd, 2, "partial hold: still two accepted");
            end

            @(negedge clk); cmd_ready = 1'b1;
            @(negedge clk);
            chk_i(n_cmd, 3, "partial: final command fired exactly once");

            step(8);
            chk_i(n_cmd,  3, "partial: exactly three commands, no duplicate");
            chk_i(n_done, 1, "partial: one completion pulse");
            chk(!bypass_active, "partial: mode dropped only after the fire");
            chk(!burst_active,  "partial: burst_active dropped after the fire");
            chk(got_q[2].pix === fin_pix,
                "partial: the delivered final pixel is the one that was held");
        end : partial_hold

        // =============================================================
        banner("15 - configurable origin structure");
        // base_row/base_col are zero in this version, so every address is
        // exactly row*IMG_WIDTH + col. Checked explicitly against the
        // golden model with base (0,0) so that when a non-zero origin is
        // introduced, the expected values here change in one place.
        // =============================================================
        clear_counts();
        run_burst(3, 3, "origin");
        for (int i = 0; i < 9 && i < got_q.size(); i++)
            chk(got_q[i].addr === BURST_ADDR_W'(exp_addr(i, 3, 0, 0)),
                $sformatf("origin (0,0): cmd %0d addr", i));
        chk(dut.base_row === '0, "base_row is zero in this version");
        chk(dut.base_col === '0, "base_col is zero in this version");

        // =============================================================
        $display("-------------------------------------------------");
        $display(" checks executed : %0d", checks);
        $display(" errors          : %0d", errors);
        $display(" RESULT: %s", (errors == 0) ? "PASS" : "FAIL");
        $display("=================================================");
        if (errors != 0) $fatal(1, "tb_rx_burst_ctrl FAILED");
        $finish;
    end

    // Guard: a hang must not look like a pass.
    initial begin
        #5ms;
        $display(" RESULT: FAIL -- testbench timeout");
        $fatal(1, "tb_rx_burst_ctrl timed out");
    end

endmodule : tb_rx_burst_ctrl
