`timescale 1ns/1ps
// tb_apb_master.sv
// ================
// Unit test for apb_master. A behavioural slave model plays the far side,
// with configurable wait states and a forceable PSLVERR -- neither of which
// apb_slave_rgf can produce at WAIT_STATES=0, which is why the model exists
// rather than reusing the real slave.
//
// The protocol properties (SETUP exactly one cycle, PENABLE only inside a
// transfer, address stable across ACCESS, no request while busy) are checked
// by the assertions inside apb_master itself, compiled in with --assert.
// This testbench checks observable behaviour and cycle counts.
//
// SAMPLING NOTE
// rsp_valid, rsp_is_read and rsp_error are all registered and clear the cycle
// after completion. Waiting for the response and THEN reading them is a race
// that reports false failures -- it did, on the first run of this file. Every
// response field is therefore latched by an observer at the moment rsp_valid
// is high, and the checks read the latched copies.

module tb_apb_master;

    import apb_pkg::*;

    logic clk = 1'b0;
    logic rst_n;
    always #5 clk = ~clk;                       // 100 MHz

    logic                req_valid, req_write;
    logic [ADDR_W-1:0]   req_addr;
    logic [DATA_W-1:0]   req_wdata;
    logic                busy;

    logic                rsp_valid, rsp_is_read, rsp_error;
    logic [DATA_W-1:0]   rsp_rdata;

    logic                m_active, m_penable, m_pwrite;
    logic [ADDR_W-1:0]   m_paddr;
    logic [DATA_W-1:0]   m_pwdata;
    logic [STRB_W-1:0]   m_pstrb;
    logic                m_pready, m_pslverr;
    logic [DATA_W-1:0]   m_prdata;

    int errors = 0, checks = 0;

    task automatic chk(input string what, input logic cond);
        checks++;
        if (!cond) begin errors++; $display("  FAIL  %s   (t=%0t)", what, $time); end
        else $display("  pass  %s", what);
    endtask

    // ------------------------------------------------------------------
    // Behavioural slave
    // ------------------------------------------------------------------
    int   wait_states = 0;
    logic force_err   = 1'b0;
    int   wcnt = 0;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)                        wcnt <= 0;
        else if (!(m_active && m_penable)) wcnt <= 0;
        else if (wcnt != wait_states)      wcnt <= wcnt + 1;
    end

    assign m_pready  = (m_active && m_penable) && (wcnt == wait_states);
    assign m_pslverr = m_pready && force_err;
    assign m_prdata  = {16'hA5A5, m_paddr};

    // ------------------------------------------------------------------
    // DUT
    // ------------------------------------------------------------------
    apb_master u_dut (
        .clk(clk), .rst_n(rst_n),
        .req_valid(req_valid), .req_write(req_write),
        .req_addr(req_addr), .req_wdata(req_wdata), .busy(busy),
        .rsp_valid(rsp_valid), .rsp_is_read(rsp_is_read),
        .rsp_rdata(rsp_rdata), .rsp_error(rsp_error),
        .m_active(m_active), .m_penable(m_penable), .m_pwrite(m_pwrite),
        .m_paddr(m_paddr), .m_pwdata(m_pwdata), .m_pstrb(m_pstrb),
        .m_pready(m_pready), .m_prdata(m_prdata), .m_pslverr(m_pslverr)
    );

    // ------------------------------------------------------------------
    // Observers. Response fields latched AT the pulse.
    // ------------------------------------------------------------------
    int  setup_cycles = 0, access_cycles = 0, busy_cycles = 0, rsp_pulses = 0;
    logic              obs_is_read, obs_error;
    logic [DATA_W-1:0] obs_rdata;
    int  p0;

    always @(posedge clk) if (rst_n) begin
        if (m_active && !m_penable) setup_cycles++;
        if (m_active &&  m_penable) access_cycles++;
        if (busy)                   busy_cycles++;
        if (rsp_valid) begin
            rsp_pulses++;
            obs_is_read <= rsp_is_read;
            obs_error   <= rsp_error;
            obs_rdata   <= rsp_rdata;
        end
    end

    task automatic issue(input logic wr, input logic [ADDR_W-1:0] a,
                         input logic [DATA_W-1:0] d);
        @(negedge clk);
        while (busy) @(negedge clk);
        req_valid = 1'b1; req_write = wr; req_addr = a; req_wdata = d;
        @(negedge clk);
        req_valid = 1'b0; req_write = 1'b0; req_addr = '0; req_wdata = '0;
    endtask

    task automatic await(input int prev);
        int guard = 0;
        while (rsp_pulses == prev) begin
            @(negedge clk);
            guard++;
            if (guard > 200) begin
                $display("  FAIL  response never arrived (t=%0t)", $time);
                errors++;
                return;
            end
        end
        @(negedge clk);              // let the observer's latches settle
    endtask

    initial begin
        rst_n = 0; req_valid = 0; req_write = 0; req_addr = '0; req_wdata = '0;
        repeat (4) @(negedge clk);
        rst_n = 1;
        repeat (2) @(negedge clk);

        $display("\n=== T1: zero-wait write is a two-cycle transfer ===");
        setup_cycles = 0; access_cycles = 0; p0 = rsp_pulses;
        issue(1'b1, 16'h0010, 32'hDEAD_BEEF);
        await(p0);
        chk("SETUP occupied exactly 1 cycle",  setup_cycles  == 1);
        chk("ACCESS occupied exactly 1 cycle", access_cycles == 1);
        chk("exactly one response pulse",      rsp_pulses    == p0 + 1);
        chk("rsp_is_read low for a write",     obs_is_read === 1'b0);
        chk("rsp_error low",                   obs_error   === 1'b0);

        $display("\n=== T2: read returns PRDATA and flags itself ===");
        p0 = rsp_pulses;
        issue(1'b0, 16'h0014, 32'h0);
        await(p0);
        chk("rsp_is_read high for a read", obs_is_read === 1'b1);
        chk("rsp_rdata captured PRDATA",   obs_rdata === {16'hA5A5, 16'h0014});
        chk("exactly one response pulse",  rsp_pulses == p0 + 1);

        $display("\n=== T3: busy spans acceptance to completion ===");
        // request cycle + SETUP + ACCESS = 3 cycles of busy at zero wait
        busy_cycles = 0; p0 = rsp_pulses;
        issue(1'b1, 16'h0008, 32'h1);
        await(p0);
        chk("busy asserted for 3 cycles", busy_cycles == 3);

        $display("\n=== T4: busy is high during the request cycle itself ===");
        // Without the req_valid term in busy, a strobe in the very next cycle
        // would be accepted while the FSM was still in SETUP.
        @(negedge clk);
        while (busy) @(negedge clk);
        p0 = rsp_pulses;
        req_valid = 1'b1; req_write = 1'b1; req_addr = 16'h0000; req_wdata = 32'h2;
        @(posedge clk);
        chk("busy high in the same cycle as req_valid", busy === 1'b1);
        @(negedge clk);
        req_valid = 1'b0;
        await(p0);

        $display("\n=== T5: PSTRB reflects direction ===");
        p0 = rsp_pulses;
        issue(1'b1, 16'h0010, 32'hFFFF_0000);
        @(posedge clk);
        chk("PSTRB all lanes on a write", m_pstrb === {STRB_W{1'b1}});
        await(p0);
        p0 = rsp_pulses;
        issue(1'b0, 16'h0010, 32'h0);
        @(posedge clk);
        chk("PSTRB zero on a read", m_pstrb === {STRB_W{1'b0}});
        await(p0);

        $display("\n=== T6: wait states stretch ACCESS, not SETUP ===");
        wait_states = 3;
        setup_cycles = 0; access_cycles = 0; p0 = rsp_pulses;
        issue(1'b0, 16'h0004, 32'h0);
        await(p0);
        chk("SETUP still exactly 1 cycle",         setup_cycles  == 1);
        chk("ACCESS stretched to 4 cycles",        access_cycles == 4);
        chk("read data still correct after waits", obs_rdata === {16'hA5A5, 16'h0004});
        chk("still exactly one response pulse",    rsp_pulses    == p0 + 1);
        wait_states = 0;

        $display("\n=== T7: PSLVERR is captured ===");
        force_err = 1'b1; p0 = rsp_pulses;
        issue(1'b0, 16'h0100, 32'h0);
        await(p0);
        chk("rsp_error set on PSLVERR", obs_error === 1'b1);
        force_err = 1'b0; p0 = rsp_pulses;
        issue(1'b0, 16'h0000, 32'h0);
        await(p0);
        chk("rsp_error clears again", obs_error === 1'b0);

        $display("\n=== T8: back-to-back requests all complete ===");
        p0 = rsp_pulses;
        for (int i = 0; i < 8; i++) issue(i[0], 16'(i * 4), 32'(i));
        repeat (12) @(negedge clk);
        chk("8 requests produced 8 responses", rsp_pulses == p0 + 8);
        chk("master returned to idle",         busy === 1'b0);
        chk("bus deasserted",                  m_active === 1'b0);

        $display("\n----------------------------------------");
        $display("checks: %0d   errors: %0d", checks, errors);
        if (errors == 0) $display("RESULT: PASS");
        else             $display("RESULT: FAIL");
        $display("----------------------------------------\n");
        $finish;
    end

    initial begin
        #200000;
        $display("TIMEOUT");
        $finish;
    end

endmodule : tb_apb_master
