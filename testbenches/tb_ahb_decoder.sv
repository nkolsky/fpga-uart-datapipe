`timescale 1ns/1ps
// tb_ahb_decoder.sv
// =================
// The whole fabric, all real modules: ahb_master -> ahb_decoder -> three
// ahb_slave_sram, each wrapping an rgb_sram. This is the first point at
// which steps 1-4 run together.
//
// Two tests carry the weight:
//
//   T4  back-to-back bursts to DIFFERENT channels. If the response mux
//       followed the live address decode instead of the registered
//       data-phase decode, the last beat of one burst would be muxed from
//       the next burst's slave and return the wrong channel's data.
//
//   T5  an unmapped aperture. Without the default slave nothing drives
//       HREADY, the manager waits forever and the bus locks up.

module tb_ahb_decoder;

    import ahb_pkg::*;

    localparam int SRAM_ADDR_W = 14;
    localparam int DEPTH       = 16384;

    logic hclk = 1'b0;
    logic hresetn;
    always #5 hclk = ~hclk;

    // request
    logic                req_valid, req_write, req_burst;
    logic [SEL_W-1:0]    req_channel;
    logic [WORD_W-1:0]   req_word;
    logic                busy;

    logic                rd_valid;
    logic [DATA_W-1:0]   rd_data;
    logic [1:0]          rd_beat;
    logic [DATA_W-1:0]   wr_data;
    logic                wr_ack;
    logic [1:0]          wr_beat;

    // bus
    logic [ADDR_W-1:0]   haddr;
    logic                hwrite;
    logic [2:0]          hsize, hburst;
    logic [1:0]          htrans;
    logic [DATA_W-1:0]   hwdata;
    logic                hready, hresp, err_sticky;
    logic [DATA_W-1:0]   hrdata;

    logic [NUM_SLAVES-1:0] hsel, s_hreadyout, s_hresp;
    logic [DATA_W-1:0]     s_hrdata [NUM_SLAVES];
    logic                  decode_err;

    int errors = 0, checks = 0;
    task automatic chk(input string what, input logic cond);
        checks++;
        if (!cond) begin errors++; $display("  FAIL  %s   (t=%0t)", what, $time); end
        else $display("  pass  %s", what);
    endtask

    // ------------------------------------------------------------------
    ahb_master u_master (
        .hclk(hclk), .hresetn(hresetn),
        .req_valid(req_valid), .req_write(req_write), .req_burst(req_burst),
        .req_channel(req_channel), .req_word(req_word), .busy(busy),
        .rd_valid(rd_valid), .rd_data(rd_data), .rd_beat(rd_beat),
        .wr_data(wr_data), .wr_ack(wr_ack), .wr_beat(wr_beat),
        .haddr(haddr), .hwrite(hwrite), .hsize(hsize), .hburst(hburst),
        .htrans(htrans), .hwdata(hwdata),
        .hready(hready), .hrdata(hrdata), .hresp(hresp),
        .err_sticky(err_sticky)
    );

    ahb_decoder u_decoder (
        .hclk(hclk), .hresetn(hresetn),
        .haddr(haddr), .htrans(htrans),
        .m_hready(hready), .m_hrdata(hrdata), .m_hresp(hresp),
        .hsel(hsel),
        .s_hreadyout(s_hreadyout), .s_hresp(s_hresp), .s_hrdata(s_hrdata),
        .decode_err(decode_err)
    );

    // three channels, each a real slave + real SRAM
    logic                   rd_en   [NUM_SLAVES];
    logic                   wr_en   [NUM_SLAVES];
    logic [SRAM_ADDR_W-1:0] rd_addr [NUM_SLAVES];
    logic [SRAM_ADDR_W-1:0] wr_addr [NUM_SLAVES];
    logic [DATA_W-1:0]      rd_dat  [NUM_SLAVES];
    logic [DATA_W-1:0]      wr_dat  [NUM_SLAVES];
    logic [DATA_W/8-1:0]    wr_be   [NUM_SLAVES];

    for (genvar i = 0; i < NUM_SLAVES; i++) begin : g_ch
        ahb_slave_sram #(.SRAM_ADDR_W(SRAM_ADDR_W)) u_slave (
            .hclk(hclk), .hresetn(hresetn),
            .hsel(hsel[i]),
            .hready(hready),            // AGGREGATED segment ready
            .haddr(haddr), .hwrite(hwrite), .hsize(hsize),
            .hburst(hburst), .htrans(htrans), .hwdata(hwdata),
            .hreadyout(s_hreadyout[i]), .hrdata(s_hrdata[i]), .hresp(s_hresp[i]),
            .sram_rd_en(rd_en[i]), .sram_rd_addr(rd_addr[i]),
            .sram_rd_data(rd_dat[i]),
            .sram_wr_en(wr_en[i]), .sram_wr_be(wr_be[i]),
            .sram_wr_addr(wr_addr[i]), .sram_wr_data(wr_dat[i])
        );
        rgb_sram #(.DATA_WIDTH(DATA_W), .DEPTH(DEPTH), .INIT_FILE("")) u_sram (
            .clk(hclk),
            .rd_en(rd_en[i]), .rd_addr(rd_addr[i]), .rd_data(rd_dat[i]),
            .wr_en(wr_en[i]), .wr_be(wr_be[i]),
            .wr_addr(wr_addr[i]), .wr_data(wr_dat[i])
        );
    end : g_ch

    // ------------------------------------------------------------------
    int rd_pulses = 0, derr_pulses = 0;
    logic [DATA_W-1:0] got [4];
    logic [DATA_W-1:0] wsrc [4];
    assign wr_data = wsrc[wr_beat];
    int p0;

    // HSEL is a pure address decode, so it is asserted during IDLE too --
    // haddr is zero then, which decodes to R. That is legal: the slave
    // qualifies with HTRANS. What must never happen is a REAL transfer to an
    // unmapped aperture selecting a slave.
    int bad_sel_cycles = 0;

    // The mapped slaves are all zero-wait, so HREADY only ever goes low for
    // the first cycle of a two-cycle error response. Counting those cycles is
    // what distinguishes a protocol-legal error from a one-cycle one: both
    // complete and both report, but only the legal one stalls first.
    int hready_low_cycles = 0;

    always @(posedge hclk) if (hresetn) begin
        if (decode_err) derr_pulses++;
        if (rd_valid) begin got[rd_beat] <= rd_data; rd_pulses++; end
        if (((htrans == HTRANS_NONSEQ) || (htrans == HTRANS_SEQ)) &&
            (addr_sel(haddr) == 2'b11) && (hsel != '0))
            bad_sel_cycles++;
        if (!hready) hready_low_cycles++;
    end

    task automatic issue(input logic wr, input logic burst,
                         input logic [SEL_W-1:0] ch, input logic [WORD_W-1:0] w);
        @(negedge hclk);
        while (busy) @(negedge hclk);
        req_valid = 1'b1; req_write = wr; req_burst = burst;
        req_channel = ch; req_word = w;
        @(negedge hclk);
        req_valid = 1'b0;
    endtask

    task automatic settle(output bit ok);
        int guard = 0;
        ok = 1'b1;
        while (busy) begin
            @(negedge hclk);
            guard++;
            if (guard > 80) begin ok = 1'b0; return; end
        end
        @(negedge hclk);
    endtask

    bit ok;

    // Seed a channel with a recognisable pattern via AHB writes.
    task automatic seed(input logic [SEL_W-1:0] ch, input logic [DATA_W-1:0] tag);
        for (int b = 0; b < 4; b++) wsrc[b] = tag + 32'(b);
        issue(1'b1, 1'b1, ch, 14'd40);
        settle(ok);
    endtask

    initial begin
        hresetn = 1'b0;
        req_valid = 0; req_write = 0; req_burst = 0; req_channel = '0; req_word = '0;
        for (int i = 0; i < 4; i++) wsrc[i] = '0;
        repeat (4) @(negedge hclk);
        hresetn = 1'b1;
        repeat (2) @(negedge hclk);

        $display("\n=== T1: each channel decodes to its own slave ===");
        seed(2'b00, 32'hAA00_0000);
        seed(2'b01, 32'hBB00_0000);
        seed(2'b10, 32'hCC00_0000);
        p0 = rd_pulses;
        issue(1'b0, 1'b1, 2'b00, 14'd40); settle(ok);
        chk("R burst completed", ok);
        chk("R data",            got[0] === 32'hAA00_0000 && got[3] === 32'hAA00_0003);
        issue(1'b0, 1'b1, 2'b01, 14'd40); settle(ok);
        chk("G data",            got[0] === 32'hBB00_0000 && got[3] === 32'hBB00_0003);
        issue(1'b0, 1'b1, 2'b10, 14'd40); settle(ok);
        chk("B data",            got[0] === 32'hCC00_0000 && got[3] === 32'hCC00_0003);
        chk("12 beats in total", rd_pulses == p0 + 12);

        $display("\n=== T2: HSEL is one-hot ===");
        @(negedge hclk);
        chk("at most one slave selected", $onehot0(hsel));

        $display("\n=== T3: writes landed in the right channel only ===");
        // R was seeded with AA, so G and B must not contain it.
        issue(1'b0, 1'b0, 2'b01, 14'd40); settle(ok);
        chk("G word 40 is G's value, not R's", got[0] === 32'hBB00_0000);

        $display("\n=== T4: THE MUX TEST -- back-to-back bursts, different channels ===");
        // R then B with no idle between. If the response mux followed the
        // live decode, R's last beat would be muxed from B.
        p0 = rd_pulses;
        issue(1'b0, 1'b1, 2'b00, 14'd40);
        settle(ok);
        chk("R burst, last beat is R's", got[3] === 32'hAA00_0003);
        issue(1'b0, 1'b1, 2'b10, 14'd40);
        settle(ok);
        chk("B burst, first beat is B's", got[0] === 32'hCC00_0000);
        chk("B burst, last beat is B's",  got[3] === 32'hCC00_0003);
        chk("8 beats, none lost",         rd_pulses == p0 + 8);

        $display("\n=== T5: DEADLOCK GUARD -- unmapped aperture ===");
        p0 = rd_pulses; derr_pulses = 0; bad_sel_cycles = 0; hready_low_cycles = 0;
        issue(1'b0, 1'b0, 2'b11, 14'd0);        // channel 3: nothing there
        settle(ok);
        chk("transfer COMPLETED rather than hanging", ok);
        chk("manager returned to idle", busy === 1'b0);
        chk("HRESP=ERROR captured",     err_sticky === 1'b1);
        chk("decode_err pulsed once",   derr_pulses == 1);
        chk("no slave selected during the unmapped transfer", bad_sel_cycles == 0);
        chk("error stalled for one cycle first (2-cycle response)",
            hready_low_cycles == 1);

        $display("\n=== T6: the bus recovers after an unmapped access ===");
        p0 = rd_pulses;
        issue(1'b0, 1'b1, 2'b01, 14'd40);
        settle(ok);
        chk("next mapped burst still works", ok);
        chk("G data still correct",          got[0] === 32'hBB00_0000);
        chk("four beats",                    rd_pulses == p0 + 4);

        $display("\n=== T7: unmapped BURST -- every beat answered ===");
        p0 = rd_pulses; derr_pulses = 0; hready_low_cycles = 0;
        issue(1'b0, 1'b1, 2'b11, 14'd0);
        settle(ok);
        chk("unmapped burst completed", ok);
        chk("four error responses",     derr_pulses == 4);
        chk("each beat stalled one cycle -- 4 two-cycle responses",
            hready_low_cycles == 4);
        chk("manager idle",             busy === 1'b0);

        $display("\n----------------------------------------");
        $display("checks: %0d   errors: %0d", checks, errors);
        if (errors == 0) $display("RESULT: PASS");
        else             $display("RESULT: FAIL");
        $display("----------------------------------------\n");
        $finish;
    end

    initial begin
        #400000;
        $display("TIMEOUT -- likely a hung transfer");
        $finish;
    end

endmodule : tb_ahb_decoder
