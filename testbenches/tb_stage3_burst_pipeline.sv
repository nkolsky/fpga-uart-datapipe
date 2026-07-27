// tb_stage3_burst_pipeline.sv
// ---------------------------
// M5 integration test. Instantiates the REAL chip_top and drives it over the
// serial UART line, exactly as the host does.
//
// Unlike tb_stage2c_pipeline, which wires the datapath from modules, this
// exercises the integration itself: the bypass_active ownership, the
// burst_active synchroniser, the command mux, and the cmd_ready wiring.
// Those are the only things M5 actually adds, and none of them exist in a
// module-level testbench.
//
// -----------------------------------------------------------------------
// HOW RESULTS ARE CHECKED
// -----------------------------------------------------------------------
// SRAM contents are read by hierarchical reference rather than by reading
// the image back over UART. A full readback is 65536 messages at 22.5 us
// each -- about 1.5 s of simulation, which is impractical. Checking the
// memory directly proves the same thing for a write test and runs in
// microseconds.
//
// Serial injection at 8.125 Mbaud costs 1.354 us per byte, so the whole
// stimulus below is roughly 150 us.
//
// -----------------------------------------------------------------------
// FRAME FORMAT ON THE WIRE
// -----------------------------------------------------------------------
// 8E1: start bit (0), eight data bits LSB first, even parity, stop bit (1).
// rx_phy rejects a frame whose parity is wrong, so the parity bit must be
// computed -- a testbench that always sent 0 would see every frame dropped.

`timescale 1ns/1ps

module tb_stage3_burst_pipeline;

    import memory_pkg::*;
    import rx_msg_pkg::*;

    // -----------------------------------------------------------------
    // 100 MHz board clock
    // -----------------------------------------------------------------
    logic CLK100MHZ = 1'b0;
    always #5 CLK100MHZ = ~CLK100MHZ;

    logic        CPU_RESETN;
    logic        UART_TXD_IN = 1'b1;      // idle high
    logic        UART_RTS    = 1'b0;      // 0 = clear to send
    logic        UART_RXD_OUT;
    logic        UART_CTS;
    logic [15:0] LED;

    chip_top dut (
        .CLK100MHZ    (CLK100MHZ),
        .CPU_RESETN   (CPU_RESETN),
        .UART_TXD_IN  (UART_TXD_IN),
        .UART_RTS     (UART_RTS),
        .UART_RXD_OUT (UART_RXD_OUT),
        .UART_CTS     (UART_CTS),
        .LED          (LED)
    );

    // One bit time at 8.125 Mbaud.
    localparam real BIT_NS = 1_000_000_000.0 / 8_125_000.0;   // 123.077 ns

    // -----------------------------------------------------------------
    // Bookkeeping
    // -----------------------------------------------------------------
    int    errors = 0, checks = 0;
    string phase  = "init";

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

    task automatic chk_h(input logic [31:0] got, input logic [31:0] exp,
                         input string what);
        checks++;
        if (got !== exp) begin
            errors++;
            $display("  [%0t] FAIL (%s): %s -- got %08h, expected %08h",
                     $time, phase, what, got, exp);
        end
    endtask

    // -----------------------------------------------------------------
    // THE integration invariant. chip_top carries its own assertion for
    // this; checking it here as well means the testbench reports it even
    // if assertions are disabled in the simulator.
    // -----------------------------------------------------------------
    always @(posedge dut.pll_clk_out) begin
        if (dut.sync_pll_rst_n) begin
            if (dut.rx_burst_cmd_valid && dut.rx_cmd_valid) begin
                errors++;
                $display("  [%0t] FAIL (invariant/%s): burst and single-pixel commands overlapped",
                         $time, phase);
            end
            if (dut.rx_burst_active && dut.rx_cmd_valid) begin
                errors++;
                $display("  [%0t] FAIL (invariant/%s): classifier command during a burst",
                         $time, phase);
            end
            if (dut.rx_bypass_active !== dut.rx_burst_active) begin
                errors++;
                $display("  [%0t] FAIL (invariant/%s): bypass_active != burst_active",
                         $time, phase);
            end
        end
    end

    // -----------------------------------------------------------------
    // RX-path activity monitors
    //
    // These exist to localise a failure. If no UART message reaches the
    // classifier, these counters say WHERE the stream stopped: at the PHY
    // (no bytes), at the MAC (bytes but no frame), or at the decode (frame
    // but the wrong kind).
    // -----------------------------------------------------------------
    int n_rx_bytes = 0;      // bytes accepted by rx_phy (parity passed)
    int n_rx_msgs  = 0;      // frames completed by rx_mac
    int n_parity_err = 0;

    always @(posedge dut.pll_clk_out) begin
        if (dut.sync_pll_rst_n) begin
            if (dut.rx_byte_valid)         n_rx_bytes++;
            if (dut.rx_mac_msg_valid)      n_rx_msgs++;
            if (dut.rx_parity_err_pulse)   n_parity_err++;
        end
    end

    task automatic clear_rx_counts();
        n_rx_bytes = 0; n_rx_msgs = 0; n_parity_err = 0;
    endtask

    // -----------------------------------------------------------------
    // Serial transmission, 8E1
    // -----------------------------------------------------------------
    task automatic uart_byte(input logic [7:0] b);
        UART_TXD_IN = 1'b0;                       // start
        #(BIT_NS);
        for (int i = 0; i < 8; i++) begin         // LSB first
            UART_TXD_IN = b[i];
            #(BIT_NS);
        end
        UART_TXD_IN = ^b;                         // even parity
        #(BIT_NS);
        UART_TXD_IN = 1'b1;                       // stop
        #(BIT_NS);
    endtask

    task automatic gap(input int bits = 4);
        UART_TXD_IN = 1'b1;
        #(BIT_NS * bits);
    endtask

    // ---- message builders -------------------------------------------
    task automatic send_burst_hdr(input int h, input int w);
        uart_byte(8'h7B); uart_byte(8'h49);                       // '{' 'I'
        uart_byte(8'h00); uart_byte(8'h00); uart_byte(8'h00);     // don't care
        uart_byte(8'h2C); uart_byte(8'h48);                       // ',' 'H'
        uart_byte(8'(h >> 16)); uart_byte(8'(h >> 8)); uart_byte(8'(h));
        uart_byte(8'h2C); uart_byte(8'h57);                       // ',' 'W'
        uart_byte(8'(w >> 16)); uart_byte(8'(w >> 8)); uart_byte(8'(w));
        uart_byte(8'h7D);                                         // '}'
        gap(8);
    endtask

    // Four RGB pixels, packed as {<R0,G0,B0,R1>,<G1,B1,R2,G2>,<B2,R3,G3,B3>}
    task automatic send_burst_data(input logic [23:0] p0, input logic [23:0] p1,
                                   input logic [23:0] p2, input logic [23:0] p3);
        uart_byte(8'h7B);
        uart_byte(p0[23:16]); uart_byte(p0[15:8]); uart_byte(p0[7:0]);
        uart_byte(p1[23:16]);
        uart_byte(8'h2C);
        uart_byte(p1[15:8]);  uart_byte(p1[7:0]);
        uart_byte(p2[23:16]); uart_byte(p2[15:8]);
        uart_byte(8'h2C);
        uart_byte(p2[7:0]);
        uart_byte(p3[23:16]); uart_byte(p3[15:8]); uart_byte(p3[7:0]);
        uart_byte(8'h7D);
        gap(8);
    endtask

    task automatic send_pix_write(input logic [23:0] a, input logic [23:0] px);
        uart_byte(8'h7B); uart_byte(8'h57);                       // '{' 'W'
        uart_byte(a[23:16]); uart_byte(a[15:8]); uart_byte(a[7:0]);
        uart_byte(8'h2C); uart_byte(8'h50);                       // ',' 'P'
        uart_byte(px[23:16]); uart_byte(px[15:8]); uart_byte(px[7:0]);
        uart_byte(8'h7D);
        gap(8);
    endtask

    task automatic send_legacy(input int r, input int c, input int v);
        uart_byte(8'h7B); uart_byte(8'h52);                       // '{' 'R'
        uart_byte(8'h30 + 8'((r/100)%10));
        uart_byte(8'h30 + 8'((r/10)%10));
        uart_byte(8'h30 + 8'(r%10));
        uart_byte(8'h2C); uart_byte(8'h43);                       // ',' 'C'
        uart_byte(8'h30 + 8'((c/100)%10));
        uart_byte(8'h30 + 8'((c/10)%10));
        uart_byte(8'h30 + 8'(c%10));
        uart_byte(8'h2C); uart_byte(8'h56);                       // ',' 'V'
        uart_byte(8'h30 + 8'((v/100)%10));
        uart_byte(8'h30 + 8'((v/10)%10));
        uart_byte(8'h30 + 8'(v%10));
        uart_byte(8'h7D);
        gap(8);
    endtask

    // -----------------------------------------------------------------
    // Golden model. A burst of H x W starting at pixel 0 writes
    // (row, col) -> row*IMG_WIDTH + col, so word = addr>>2, lane = addr[1:0],
    // and lane L occupies word bits [31-8L -: 8].
    // -----------------------------------------------------------------
    function automatic int pix_addr(input int idx, input int w);
        return (idx / w) * IMG_WIDTH + (idx % w);
    endfunction

    task automatic chk_pixel(input int addr, input logic [23:0] rgb,
                             input string what);
        automatic int word = addr >> 2;
        automatic int lane = addr & 3;
        automatic int hi   = 31 - 8*lane;
        checks++;
        if (dut.u_sram_red.mem[word][hi -: 8]   !== rgb[23:16] ||
            dut.u_sram_green.mem[word][hi -: 8] !== rgb[15:8]  ||
            dut.u_sram_blue.mem[word][hi -: 8]  !== rgb[7:0]) begin
            errors++;
            $display("  [%0t] FAIL (%s): %s -- pixel %0d (word %0d lane %0d) = (%0d,%0d,%0d), expected (%0d,%0d,%0d)",
                     $time, phase, what, addr, word, lane,
                     dut.u_sram_red.mem[word][hi -: 8],
                     dut.u_sram_green.mem[word][hi -: 8],
                     dut.u_sram_blue.mem[word][hi -: 8],
                     rgb[23:16], rgb[15:8], rgb[7:0]);
        end
    endtask

    // -----------------------------------------------------------------
    initial begin
        automatic logic [23:0] burst_px [16];

        $display("=================================================");
        $display(" M5 INTEGRATION: Stage 3 burst write through chip_top");
        $display("=================================================");

        // -------------------------------------------------------------
        // RESET AND PLL LOCK
        //
        // This must wait for the MMCM, not a fixed cycle count. A real
        // clk_wiz takes tens to hundreds of microseconds to lock; until
        // `locked` rises pll_clk_out is not running, sync_pll_rst_n is held
        // low by u_pll_reset_synch, and rx_phy is in reset. Any byte sent
        // in that window is discarded silently -- which presents as "no
        // message ever reaches the classifier", with burst, single-pixel
        // and legacy traffic all failing identically.
        //
        // UART_TXD_IN is initialised to 1'b1 and nothing drives it until
        // uart_byte() is first called, so the line is idle-high throughout.
        // -------------------------------------------------------------
        UART_TXD_IN = 1'b1;
        CPU_RESETN  = 1'b0;
        repeat (40) @(posedge CLK100MHZ);
        CPU_RESETN  = 1'b1;

        $display("  waiting for PLL lock ...");
        wait (dut.pll_locked === 1'b1);
        $display("  pll_locked at %0t", $time);

        wait (dut.sync_pll_rst_n === 1'b1);
        $display("  sync_pll_rst_n at %0t", $time);

        repeat (20) @(posedge dut.pll_clk_out);

        chk(dut.pll_locked,      "PLL locked before any traffic");
        chk(dut.sync_pll_rst_n,  "receive-domain reset released");
        chk(UART_TXD_IN,         "UART line idle-high through reset");
        clear_rx_counts();

        chk(!dut.rx_bypass_active, "bypass low after reset");
        chk(!dut.rx_burst_active,  "burst_active low after reset");
        chk(!dut.burst_active_100, "synchronised burst_active low after reset");
        chk(dut.sram_wr_allowed,   "writes permitted when idle");

        // =============================================================
        banner("1 - 4x4 burst, 16 pixels");
        // Distinct colour per pixel so a mis-ordered or duplicated write is
        // visible, and so no two pixels can be confused with each other.
        // =============================================================
        for (int i = 0; i < 16; i++)
            burst_px[i] = {8'(16*i + 1), 8'(16*i + 2), 8'(16*i + 3)};

        // Send the header alone first and confirm the whole receive chain
        // moved: 16 bytes accepted by the PHY, one frame completed by the
        // MAC, and the right classification. If this fails, the counters
        // localise the stall before anything downstream is suspect.
        clear_rx_counts();
        send_burst_hdr(4, 4);
        repeat (400) @(posedge CLK100MHZ);

        chk(n_parity_err == 0,
            $sformatf("header: no parity errors (%0d)", n_parity_err));
        chk(n_rx_bytes == 16,
            $sformatf("header: rx_phy accepted 16 bytes (got %0d)", n_rx_bytes));
        chk(n_rx_msgs == 1,
            $sformatf("header: rx_mac completed one frame (got %0d)", n_rx_msgs));
        chk(dut.rx_msg_kind_q == rx_msg_pkg::MSG_BURST_HDR,
            "header: classified as MSG_BURST_HDR");
        chk(dut.rx_hdr_valid, "header: parser accepted it");

        chk(dut.rx_bypass_active, "bypass asserted after the header");
        chk(dut.rx_burst_active,  "burst_active asserted after the header");
        repeat (20) @(posedge CLK100MHZ);
        chk(dut.burst_active_100, "burst_active reached the 100 MHz domain");

        clear_rx_counts();
        for (int f = 0; f < 4; f++)
            send_burst_data(burst_px[4*f+0], burst_px[4*f+1],
                            burst_px[4*f+2], burst_px[4*f+3]);

        repeat (500) @(posedge CLK100MHZ);
        chk(n_rx_bytes == 64,
            $sformatf("four data frames = 64 bytes (got %0d)", n_rx_bytes));
        chk(n_rx_msgs == 4,
            $sformatf("four frames completed (got %0d)", n_rx_msgs));
        chk(n_parity_err == 0, "no parity errors on the data frames");
        chk(!dut.rx_bypass_active, "bypass dropped after the final pixel");
        chk(!dut.rx_burst_active,  "burst_active dropped");

        // A 4x4 rectangle occupies rows 0..3, columns 0..3 -- addresses
        // 0..3, 256..259, 512..515, 768..771. Deliberately NOT 0..15.
        for (int i = 0; i < 16; i++)
            chk_pixel(pix_addr(i, 4), burst_px[i],
                      $sformatf("burst pixel %0d", i));

        // Neighbours outside the rectangle must be untouched. Column 4 of
        // row 0 is address 4, still white in the source image.
        chk_pixel(4,   24'hFFFFFF, "pixel (0,4) outside the rectangle");
        chk_pixel(255, 24'hFFFFFF, "pixel (0,255) outside the rectangle");

        // =============================================================
        banner("2 - Single Pixel Write still works after a burst");
        // Same command and colour as the validated Stage 2C hardware test.
        // =============================================================
        send_pix_write(24'h00_0402, 24'hFF_A500);
        repeat (300) @(posedge CLK100MHZ);
        chk_pixel(1026, 24'hFF_A500, "single pixel write, pixel 1026");
        // Probe the sticky flag directly rather than LED[15]. The Stage 2C
        // investigation temporarily repurposed LED[15] to img_fifo_ovf_sticky
        // and LED[14] to tx_done_sticky, so the LED map no longer reflects
        // the functional flags. The signals themselves are unchanged.
        chk(dut.pix_wr_seen_sticky, "pix_wr_seen_sticky set by the pixel write");
        chk(!dut.rx_burst_active, "no burst was started");

        // =============================================================
        banner("3 - legacy RGF still functions outside burst mode");
        // {R004,C000,V001} selects the PLL clock via CLK_CTRL, which is
        // observable on LED[11].
        // =============================================================
        send_legacy(4, 0, 1);
        repeat (300) @(posedge CLK100MHZ);
        chk(LED[11], "CLK_CTRL written -- clk_sel high");
        send_legacy(4, 0, 0);
        repeat (300) @(posedge CLK100MHZ);
        chk(!LED[11], "CLK_CTRL written -- clk_sel low again");

        // =============================================================
        banner("4 - second burst, non-square, after other traffic");
        // 2x3 exercises the row wrap at a width that is not the image
        // width, and leaves two padding slots in the second frame.
        // =============================================================
        send_burst_hdr(2, 3);
        repeat (200) @(posedge CLK100MHZ);
        chk(dut.rx_bypass_active, "second burst armed");

        send_burst_data(24'hAA0001, 24'hAA0002, 24'hAA0003, 24'hAA0004);
        send_burst_data(24'hAA0005, 24'hAA0006, 24'hDEAD01, 24'hDEAD02);
        repeat (500) @(posedge CLK100MHZ);
        chk(!dut.rx_bypass_active, "second burst completed");

        chk_pixel(0,   24'hAA0001, "2x3 pixel (0,0)");
        chk_pixel(1,   24'hAA0002, "2x3 pixel (0,1)");
        chk_pixel(2,   24'hAA0003, "2x3 pixel (0,2)");
        chk_pixel(256, 24'hAA0004, "2x3 pixel (1,0) -- row wrap");
        chk_pixel(257, 24'hAA0005, "2x3 pixel (1,1)");
        chk_pixel(258, 24'hAA0006, "2x3 pixel (1,2) -- final real pixel");
        // PADDING SUPPRESSION. The second frame carries two dummy pixels
        // (0xDEAD01, 0xDEAD02) beyond the six real ones. They must not be
        // written anywhere.
        //
        // Column 3 is outside a 3-wide rectangle, so addresses 3 and 259 are
        // the natural place for a leaked padding write to land. They are NOT
        // white, though -- the 4x4 burst in phase 1 wrote them -- so the
        // check is that they still hold their PHASE 1 values. That is a
        // stronger test than comparing against white: it proves the padding
        // neither wrote fresh data nor disturbed existing data.
        chk_pixel(3,   burst_px[3], "padding suppressed -- (0,3) still holds the 4x4 value");
        chk_pixel(259, burst_px[7], "padding suppressed -- (1,3) still holds the 4x4 value");
        chk_pixel(260, 24'hFFFFFF,  "padding suppressed -- (1,4) still white");

        // =============================================================
        banner("5 - no command source overlap and no dropped commands");
        // =============================================================
        chk(!dut.cmd_ovf_sticky,    "command FIFO never overflowed");
        chk(!dut.sram_wr_rejected,  "no command rejected as out of range");
        chk(dut.sram_wr_seen,       "sram_wr_seen set -- SRAM writes occurred");

        // LED[13] is NOT repurposed by the Stage 2C diagnostics, so it still
        // reflects (cmd_ovf_sticky || sram_wr_rejected) and remains a valid
        // check. LED[11] likewise still carries clk_sel, which phase 3 uses.
        chk(!LED[13],               "LED[13] dark -- nothing discarded");

        // =============================================================
        $display("-------------------------------------------------");
        $display(" checks executed : %0d", checks);
        $display(" errors          : %0d", errors);
        $display(" RESULT: %s", (errors == 0) ? "PASS" : "FAIL");
        $display("=================================================");
        if (errors != 0) $fatal(1, "tb_stage3_burst_pipeline FAILED");
        $finish;
    end

    initial begin
        #5ms;
        $display(" RESULT: FAIL -- testbench timeout");
        $fatal(1, "tb_stage3_burst_pipeline timed out");
    end

endmodule : tb_stage3_burst_pipeline
