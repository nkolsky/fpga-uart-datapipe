`timescale 1ns/1ps
// tb_ahb_master.sv
// ================
// Unit test for ahb_master. A behavioural slave models the memory with
// configurable wait states and a forceable HRESP=ERROR -- ahb_slave_sram is
// zero-wait and cannot fail, so it cannot exercise either path.
//
// The protocol sequencing (NONSEQ first, SEQ after, address stepping one
// word, address held while HREADY is low) is checked by the assertions
// inside ahb_master, compiled in with --assert. This file checks the
// observable behaviour: cycle counts, data ordering, and wait-state handling.

module tb_ahb_master;

    import ahb_pkg::*;

    logic hclk = 1'b0;
    logic hresetn;
    always #5 hclk = ~hclk;

    // request
    logic                req_valid, req_write, req_burst;
    logic [SEL_W-1:0]    req_channel;
    logic [WORD_W-1:0]   req_word;
    logic                busy;

    // data
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
    logic [DATA_W-1:0]   hwdata, hrdata;
    logic                hready, hresp;
    logic                err_sticky;

    int errors = 0, checks = 0;
    task automatic chk(input string what, input logic cond);
        checks++;
        if (!cond) begin errors++; $display("  FAIL  %s   (t=%0t)", what, $time); end
        else $display("  pass  %s", what);
    endtask

    // ------------------------------------------------------------------
    // Behavioural slave: a small memory, N wait states per beat, optional
    // error response.
    // ------------------------------------------------------------------
    localparam int MEMW = 64;
    logic [DATA_W-1:0] mem [MEMW];

    int   wait_states = 0;
    logic force_err   = 1'b0;
    int   wcnt = 0;

    logic active;
    assign active = (htrans == HTRANS_NONSEQ) || (htrans == HTRANS_SEQ);

    always_ff @(posedge hclk or negedge hresetn) begin
        if (!hresetn)                 wcnt <= 0;
        else if (!active)             wcnt <= 0;
        else if (wcnt == wait_states) wcnt <= 0;
        else                          wcnt <= wcnt + 1;
    end

    assign hready = !active || (wcnt == wait_states);

    // address phase captured into the data phase
    logic              d_valid, d_write;
    logic [WORD_W-1:0] d_word;

    always_ff @(posedge hclk or negedge hresetn) begin
        if (!hresetn) begin
            d_valid <= 1'b0; d_write <= 1'b0; d_word <= '0;
        end
        else if (hready) begin
            d_valid <= active;
            d_write <= hwrite;
            d_word  <= addr_word(haddr);
        end
    end

    always_ff @(posedge hclk) begin
        if (d_valid && d_write && hready) mem[d_word[5:0]] <= hwdata;
    end

    assign hrdata = (d_valid && !d_write) ? mem[d_word[5:0]] : 32'hXXXX_XXXX;
    assign hresp  = force_err ? HRESP_ERROR : HRESP_OKAY;

    // ------------------------------------------------------------------
    ahb_master u_dut (
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

    // ------------------------------------------------------------------
    // Observers
    // ------------------------------------------------------------------
    int rd_pulses = 0, wr_pulses = 0, busy_cycles = 0, nonseq_cnt = 0, seq_cnt = 0;
    logic [DATA_W-1:0] got [4];
    int p0;

    always @(posedge hclk) if (hresetn) begin
        if (busy) busy_cycles++;
        if (htrans == HTRANS_NONSEQ) nonseq_cnt++;
        if (htrans == HTRANS_SEQ)    seq_cnt++;
        if (rd_valid) begin
            got[rd_beat] <= rd_data;
            rd_pulses++;
        end
        if (wr_ack) wr_pulses++;
    end

    // Write source: beat index -> data. Presented combinationally so the word
    // for the beat in its data phase is always on wr_data.
    logic [DATA_W-1:0] wsrc [4];
    assign wr_data = wsrc[wr_beat];

    task automatic issue(input logic wr, input logic burst,
                         input logic [SEL_W-1:0] ch, input logic [WORD_W-1:0] w);
        @(negedge hclk);
        while (busy) @(negedge hclk);
        req_valid = 1'b1; req_write = wr; req_burst = burst;
        req_channel = ch; req_word = w;
        @(negedge hclk);
        req_valid = 1'b0;
    endtask

    task automatic settle();
        int guard = 0;
        while (busy) begin
            @(negedge hclk);
            guard++;
            if (guard > 100) begin
                $display("  FAIL  manager never returned to idle"); errors++; return;
            end
        end
        @(negedge hclk);
    endtask

    initial begin
        hresetn = 1'b0;
        req_valid = 0; req_write = 0; req_burst = 0; req_channel = '0; req_word = '0;
        for (int i = 0; i < 4; i++) wsrc[i] = '0;
        for (int i = 0; i < MEMW; i++) mem[i] = 32'h2000_0000 + 32'(i);
        repeat (4) @(negedge hclk);
        hresetn = 1'b1;
        repeat (2) @(negedge hclk);

        $display("\n=== T1: SINGLE read ===");
        p0 = rd_pulses; nonseq_cnt = 0; seq_cnt = 0;
        issue(1'b0, 1'b0, 2'b00, 14'd5);
        settle();
        chk("one data beat returned", rd_pulses == p0 + 1);
        chk("correct word",           got[0] === 32'h2000_0005);
        chk("one NONSEQ, no SEQ",     nonseq_cnt == 1 && seq_cnt == 0);

        $display("\n=== T2: INCR4 read -- four beats, right order ===");
        p0 = rd_pulses; nonseq_cnt = 0; seq_cnt = 0;
        issue(1'b0, 1'b1, 2'b00, 14'd8);
        settle();
        chk("four beats returned", rd_pulses == p0 + 4);
        chk("beat 0", got[0] === 32'h2000_0008);
        chk("beat 1", got[1] === 32'h2000_0009);
        chk("beat 2", got[2] === 32'h2000_000A);
        chk("beat 3", got[3] === 32'h2000_000B);
        chk("one NONSEQ then three SEQ", nonseq_cnt == 1 && seq_cnt == 3);

        $display("\n=== T3: INCR4 costs five cycles, not four ===");
        // 4 address beats + 1 trailing data phase, plus the request cycle.
        busy_cycles = 0;
        issue(1'b0, 1'b1, 2'b00, 14'd8);
        settle();
        chk("busy for 6 cycles (1 request + 4 address + 1 drain)", busy_cycles == 6);

        $display("\n=== T4: INCR4 write ===");
        for (int i = 0; i < 4; i++) wsrc[i] = 32'hBEEF_0000 + 32'(i);
        p0 = wr_pulses;
        issue(1'b1, 1'b1, 2'b00, 14'd20);
        settle();
        chk("four write acks", wr_pulses == p0 + 4);
        chk("word 20 stored", mem[20] === 32'hBEEF_0000);
        chk("word 21 stored", mem[21] === 32'hBEEF_0001);
        chk("word 22 stored", mem[22] === 32'hBEEF_0002);
        chk("word 23 stored", mem[23] === 32'hBEEF_0003);

        $display("\n=== T5: wait states stretch the burst, data still correct ===");
        wait_states = 2;
        p0 = rd_pulses; busy_cycles = 0;
        issue(1'b0, 1'b1, 2'b00, 14'd8);
        settle();
        chk("still four beats",  rd_pulses == p0 + 4);
        chk("beat 0 correct",    got[0] === 32'h2000_0008);
        chk("beat 3 correct",    got[3] === 32'h2000_000B);
        chk("burst took longer", busy_cycles > 6);
        wait_states = 0;

        $display("\n=== T6: channel select reaches HADDR ===");
        issue(1'b0, 1'b0, 2'b10, 14'd3);      // channel B
        @(posedge hclk);
        chk("HADDR carries channel 2", haddr[SEL_LSB +: SEL_W] === 2'b10);
        settle();

        $display("\n=== T7: HRESP=ERROR is captured ===");
        chk("err_sticky clear so far", err_sticky === 1'b0);
        force_err = 1'b1;
        issue(1'b0, 1'b0, 2'b00, 14'd1);
        settle();
        chk("err_sticky set on ERROR", err_sticky === 1'b1);
        force_err = 1'b0;

        $display("\n=== T8: back-to-back bursts, none lost ===");
        p0 = rd_pulses;
        for (int i = 0; i < 5; i++) begin
            issue(1'b0, 1'b1, 2'b00, WORD_W'(8 + i*4));
            settle();
        end
        chk("5 bursts gave 20 beats", rd_pulses == p0 + 20);
        chk("manager idle",           busy === 1'b0);
        chk("bus idle",               htrans === HTRANS_IDLE);

        $display("\n----------------------------------------");
        $display("checks: %0d   errors: %0d", checks, errors);
        if (errors == 0) $display("RESULT: PASS");
        else             $display("RESULT: FAIL");
        $display("----------------------------------------\n");
        $finish;
    end

    initial begin
        #300000;
        $display("TIMEOUT");
        $finish;
    end

endmodule : tb_ahb_master
