`timescale 1ns/1ps
// tb_apb_bar.sv
// =============
// Tests apb_bar in the real chain: apb_master -> apb_bar -> apb_slave_rgf ->
// rgf. No behavioural models. This is the first point at which every APB
// piece runs together, so it doubles as the integration test for steps 1-4.
//
// The test that matters is T4. Without apb_bar's default responder, an access
// to an unmapped aperture leaves nobody driving PREADY, apb_master sits in
// M_ACCESS forever and busy never falls -- which in the real design would
// stall the router, the crossing and UART_CTS permanently. T4 asserts that
// the transfer COMPLETES and the master returns to idle.

module tb_apb_bar;

    import apb_pkg::*;
    import rgf_pkg::*;

    logic clk = 1'b0;
    logic rst_n;
    always #5 clk = ~clk;                       // 100 MHz

    // request side
    logic                req_valid, req_write;
    logic [ADDR_W-1:0]   req_addr;
    logic [DATA_W-1:0]   req_wdata;
    logic                busy;

    // response side
    logic                rsp_valid, rsp_is_read, rsp_error;
    logic [DATA_W-1:0]   rsp_rdata;

    // master <-> bar
    logic                m_active, m_penable, m_pwrite;
    logic [ADDR_W-1:0]   m_paddr;
    logic [DATA_W-1:0]   m_pwdata;
    logic [STRB_W-1:0]   m_pstrb;
    logic                m_pready, m_pslverr;
    logic [DATA_W-1:0]   m_prdata;

    // bar <-> slaves
    logic [NUM_SLAVES-1:0] psel;
    logic [NUM_SLAVES-1:0] s_pready, s_pslverr;
    logic [DATA_W-1:0]     s_prdata [NUM_SLAVES];
    logic                  decode_err;

    // slave <-> rgf
    logic [OFFSET_W-1:0] pc_addr;
    logic                pc_wen, pc_ren;
    logic [DATA_W-1:0]   pc_wdata, pc_rdata;

    logic                start_img_read, clk_sel;

    int errors = 0, checks = 0;

    task automatic chk(input string what, input logic cond);
        checks++;
        if (!cond) begin errors++; $display("  FAIL  %s   (t=%0t)", what, $time); end
        else $display("  pass  %s", what);
    endtask

    // ------------------------------------------------------------------
    // DUTs -- all real
    // ------------------------------------------------------------------
    apb_master u_master (
        .clk(clk), .rst_n(rst_n),
        .req_valid(req_valid), .req_write(req_write),
        .req_addr(req_addr), .req_wdata(req_wdata), .busy(busy),
        .rsp_valid(rsp_valid), .rsp_is_read(rsp_is_read),
        .rsp_rdata(rsp_rdata), .rsp_error(rsp_error),
        .m_active(m_active), .m_penable(m_penable), .m_pwrite(m_pwrite),
        .m_paddr(m_paddr), .m_pwdata(m_pwdata), .m_pstrb(m_pstrb),
        .m_pready(m_pready), .m_prdata(m_prdata), .m_pslverr(m_pslverr)
    );

    apb_bar u_bar (
        .m_active(m_active), .m_penable(m_penable), .m_paddr(m_paddr),
        .m_pready(m_pready), .m_prdata(m_prdata), .m_pslverr(m_pslverr),
        .psel(psel),
        .s_pready(s_pready), .s_pslverr(s_pslverr), .s_prdata(s_prdata),
        .decode_err(decode_err)
    );

    apb_slave_rgf #(.WAIT_STATES(0)) u_slave (
        .clk(clk), .rst_n(rst_n),
        .psel(psel[0]), .penable(m_penable), .pwrite(m_pwrite),
        .paddr(m_paddr), .pwdata(m_pwdata),
        .pready(s_pready[0]), .prdata(s_prdata[0]), .pslverr(s_pslverr[0]),
        .pc_addr(pc_addr), .pc_wen(pc_wen), .pc_ren(pc_ren),
        .pc_wdata(pc_wdata), .pc_rdata(pc_rdata)
    );

    rgf u_rgf (
        .clk(clk), .rst_n(rst_n),
        .pc_wen(pc_wen), .pc_ren(pc_ren), .pc_addr(pc_addr),
        .pc_wdata(pc_wdata), .pc_rdata(pc_rdata),
        .status_wen(1'b0), .status_addr(8'h0), .status_wdata(32'h0),
        .img_height_in(10'd256), .img_width_in(10'd256), .img_ready_in(1'b1),
        .start_img_read_out(start_img_read), .clk_sel_out(clk_sel),
        .parity_fault_incr(1'b0),
        .fifo_full(1'b0), .fifo_empty(1'b1),
        .fifo_almost_full(1'b0), .fifo_almost_empty(1'b1)
    );

    // ------------------------------------------------------------------
    // Observers -- response fields latched AT the pulse (see tb_apb_master)
    // ------------------------------------------------------------------
    int  rsp_pulses = 0, derr_pulses = 0, psel_cycles = 0;
    logic              obs_is_read, obs_error;
    logic [DATA_W-1:0] obs_rdata;
    int  p0;

    always @(posedge clk) if (rst_n) begin
        if (decode_err) derr_pulses++;
        if (psel[0])    psel_cycles++;
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

    // Bounded wait. Returns 0 if the response never came -- which is what a
    // missing default responder looks like.
    function automatic bit responded(input int prev);
        return (rsp_pulses != prev);
    endfunction

    task automatic await(input int prev, output bit ok);
        int guard = 0;
        ok = 1'b1;
        while (rsp_pulses == prev) begin
            @(negedge clk);
            guard++;
            if (guard > 60) begin ok = 1'b0; return; end
        end
        @(negedge clk);
    endtask

    bit ok;

    initial begin
        rst_n = 0; req_valid = 0; req_write = 0; req_addr = '0; req_wdata = '0;
        repeat (4) @(negedge clk);
        rst_n = 1;
        repeat (2) @(negedge clk);

        $display("\n=== T1: aperture 0 decodes to the RGF ===");
        p0 = rsp_pulses; psel_cycles = 0;
        issue(1'b0, 16'h0000, 32'h0);           // IMG_STATUS
        await(p0, ok);
        chk("transfer completed",            ok);
        chk("psel[0] asserted for 2 cycles", psel_cycles == 2);
        chk("IMG_STATUS height = 256",       obs_rdata[9:0]   == 10'd256);
        chk("IMG_STATUS width  = 256",       obs_rdata[19:10] == 10'd256);
        chk("no PSLVERR on a mapped access", obs_error === 1'b0);

        $display("\n=== T2: write and read back through the whole chain ===");
        p0 = rsp_pulses;
        issue(1'b1, 16'h0010, 32'h0000_0001);   // CLK_CTRL = 1
        await(p0, ok);
        chk("write completed", ok);
        chk("clk_sel_out followed the write", clk_sel === 1'b1);
        p0 = rsp_pulses;
        issue(1'b0, 16'h0010, 32'h0);
        await(p0, ok);
        chk("read completed",        ok);
        chk("CLK_CTRL reads back 1", obs_rdata[0] == 1'b1);
        chk("rsp_is_read set",       obs_is_read === 1'b1);

        $display("\n=== T3: offset is taken from PADDR[7:0] only ===");
        // Aperture 0 with a high offset: still the RGF, unimplemented offset.
        p0 = rsp_pulses;
        issue(1'b0, 16'h0018, 32'h0);
        await(p0, ok);
        chk("in-aperture unimplemented offset completes", ok);
        chk("reads zero",                obs_rdata == 32'h0);
        chk("no PSLVERR (offset is the slave's business)", obs_error === 1'b0);

        $display("\n=== T4: DEADLOCK GUARD -- unmapped aperture ===");
        p0 = rsp_pulses; derr_pulses = 0; psel_cycles = 0;
        issue(1'b0, 16'h0100, 32'h0);           // aperture 1: nothing there
        await(p0, ok);
        chk("transfer COMPLETED rather than hanging", ok);
        chk("master returned to idle",   busy === 1'b0);
        chk("bus deasserted",            m_active === 1'b0);
        chk("PSLVERR reported",          obs_error === 1'b1);
        chk("no slave was selected",     psel_cycles == 0);
        chk("decode_err pulsed exactly once", derr_pulses == 1);

        $display("\n=== T5: the bus recovers after an unmapped access ===");
        p0 = rsp_pulses;
        issue(1'b0, 16'h0000, 32'h0);
        await(p0, ok);
        chk("next mapped access still works", ok);
        chk("IMG_STATUS still correct",  obs_rdata[9:0] == 10'd256);
        chk("rsp_error cleared again",   obs_error === 1'b0);

        $display("\n=== T6: unmapped WRITE is reported, not silently dropped ===");
        p0 = rsp_pulses; derr_pulses = 0;
        issue(1'b1, 16'hFF00, 32'hDEAD_BEEF);
        await(p0, ok);
        chk("unmapped write completed",   ok);
        chk("PSLVERR on the write",       obs_error === 1'b1);
        chk("decode_err pulsed",          derr_pulses == 1);

        $display("\n=== T7: mixed traffic, all accounted for ===");
        p0 = rsp_pulses; derr_pulses = 0;
        for (int i = 0; i < 6; i++) begin
            issue(i[0], (i % 2 == 0) ? 16'h0010 : 16'h0200, 32'(i));
        end
        repeat (15) @(negedge clk);
        chk("6 requests produced 6 responses", rsp_pulses == p0 + 6);
        chk("3 of them were decode errors",    derr_pulses == 3);
        chk("master idle at the end",          busy === 1'b0);

        $display("\n----------------------------------------");
        $display("checks: %0d   errors: %0d", checks, errors);
        if (errors == 0) $display("RESULT: PASS");
        else             $display("RESULT: FAIL");
        $display("----------------------------------------\n");
        $finish;
    end

    initial begin
        #200000;
        $display("TIMEOUT -- likely a hung transfer");
        $finish;
    end

endmodule : tb_apb_bar
