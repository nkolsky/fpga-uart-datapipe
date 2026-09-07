`timescale 1ns/1ps
// tb_apb_integration.sv
// =====================
// Step 5 integration test: mem_msg_router -> apb_master -> apb_bar ->
// apb_slave_rgf -> rgf, all real modules. Drives register MESSAGES at the
// router exactly as cdc_msg_sync does.
//
// Two things here that no earlier testbench could reach:
//
//   T3  msg_ready must DROP while the bus is busy. Before step 5 the router
//       treated the RGF as always ready. If dest_ready is still tied high, a
//       message arriving during a transfer is accepted and silently lost.
//
//   T4  rgf_cmd_addr no longer parks at IDLE_ADDR. IMG_TX_MON must survive
//       idle cycles anyway, because pc_ren now qualifies the read-to-clear.

module tb_apb_integration;

    import apb_pkg::*;
    import rgf_pkg::*;
    import msg_format_pkg::*;

    logic clk = 1'b0;
    logic rst_n;
    always #5 clk = ~clk;                       // 100 MHz

    // router message input
    logic          msg_valid;
    logic          msg_ready;
    msg_kind_t     msg_kind;
    msg_payload_t  msg_payload;

    // router outputs we do not exercise here
    logic          wr_msg_valid;
    msg_kind_t     wr_msg_kind;
    msg_payload_t  wr_msg_payload;
    logic          pix_req_valid, brd_req_valid;
    logic [9:0]    pix_req_row, pix_req_col;
    logic [9:0]    brd_req_base_row, brd_req_base_col, brd_req_height, brd_req_width;

    // router -> master
    logic          rgf_cmd_valid, rgf_cmd_is_write;
    logic [7:0]    rgf_cmd_addr;
    logic [31:0]   rgf_cmd_wdata;
    logic          apb_busy;

    // master response
    logic                rsp_valid, rsp_is_read, rsp_error;
    logic [DATA_W-1:0]   rsp_rdata;

    // bus
    logic                m_active, m_penable, m_pwrite;
    logic [ADDR_W-1:0]   m_paddr;
    logic [DATA_W-1:0]   m_pwdata;
    logic [STRB_W-1:0]   m_pstrb;
    logic                m_pready, m_pslverr;
    logic [DATA_W-1:0]   m_prdata;

    logic [NUM_SLAVES-1:0] psel, s_pready, s_pslverr;
    logic [DATA_W-1:0]     s_prdata [NUM_SLAVES];
    logic                  decode_err;

    // slave -> rgf
    logic [OFFSET_W-1:0] pc_addr;
    logic                pc_wen, pc_ren;
    logic [DATA_W-1:0]   pc_wdata, pc_rdata;

    // rgf status port
    logic                status_wen;
    logic [7:0]          status_addr;
    logic [DATA_W-1:0]   status_wdata;
    logic                start_img_read, clk_sel;

    int errors = 0, checks = 0;
    task automatic chk(input string what, input logic cond);
        checks++;
        if (!cond) begin errors++; $display("  FAIL  %s   (t=%0t)", what, $time); end
        else $display("  pass  %s", what);
    endtask

    // ------------------------------------------------------------------
    // DUTs
    // ------------------------------------------------------------------
    mem_msg_router u_router (
        .clk(clk), .rst_n(rst_n),
        .msg_valid(msg_valid), .msg_ready(msg_ready),
        .msg_kind(msg_kind), .msg_payload(msg_payload),
        .wr_msg_valid(wr_msg_valid), .wr_msg_ready(1'b1),
        .wr_msg_kind(wr_msg_kind), .wr_msg_payload(wr_msg_payload),
        .pix_req_valid(pix_req_valid), .pix_req_row(pix_req_row),
        .pix_req_col(pix_req_col), .pix_busy(1'b0),
        .brd_req_valid(brd_req_valid), .brd_req_base_row(brd_req_base_row),
        .brd_req_base_col(brd_req_base_col), .brd_req_height(brd_req_height),
        .brd_req_width(brd_req_width), .brd_busy(1'b0),
        .rgf_cmd_valid(rgf_cmd_valid), .rgf_cmd_is_write(rgf_cmd_is_write),
        .rgf_cmd_addr(rgf_cmd_addr), .rgf_cmd_wdata(rgf_cmd_wdata),
        .apb_busy(apb_busy)
    );

    apb_master u_master (
        .clk(clk), .rst_n(rst_n),
        .req_valid(rgf_cmd_valid), .req_write(rgf_cmd_is_write),
        .req_addr(addr_from_msg(rgf_cmd_addr)), .req_wdata(rgf_cmd_wdata),
        .busy(apb_busy),
        .rsp_valid(rsp_valid), .rsp_is_read(rsp_is_read),
        .rsp_rdata(rsp_rdata), .rsp_error(rsp_error),
        .m_active(m_active), .m_penable(m_penable), .m_pwrite(m_pwrite),
        .m_paddr(m_paddr), .m_pwdata(m_pwdata), .m_pstrb(m_pstrb),
        .m_pready(m_pready), .m_prdata(m_prdata), .m_pslverr(m_pslverr)
    );

    apb_bar u_bar (
        .m_active(m_active), .m_penable(m_penable), .m_paddr(m_paddr),
        .m_pready(m_pready), .m_prdata(m_prdata), .m_pslverr(m_pslverr),
        .psel(psel), .s_pready(s_pready), .s_pslverr(s_pslverr),
        .s_prdata(s_prdata), .decode_err(decode_err)
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
        .status_wen(status_wen), .status_addr(status_addr),
        .status_wdata(status_wdata),
        .img_height_in(10'd256), .img_width_in(10'd256), .img_ready_in(1'b1),
        .start_img_read_out(start_img_read), .clk_sel_out(clk_sel),
        .parity_fault_incr(1'b0),
        .fifo_full(1'b0), .fifo_empty(1'b1),
        .fifo_almost_full(1'b0), .fifo_almost_empty(1'b1)
    );

    // ------------------------------------------------------------------
    int rsp_pulses = 0, not_ready_cycles = 0;
    logic              obs_is_read, obs_error;
    logic [DATA_W-1:0] obs_rdata;
    int p0;

    always @(posedge clk) if (rst_n) begin
        if (!msg_ready) not_ready_cycles++;
        if (rsp_valid) begin
            rsp_pulses++;
            obs_is_read <= rsp_is_read;
            obs_error   <= rsp_error;
            obs_rdata   <= rsp_rdata;
        end
    end

    // Present a register message the way cdc_msg_sync does: hold it until
    // msg_ready is seen high.
    task automatic send_reg(input logic wr, input logic [7:0] a,
                            input logic [31:0] d);
        pl_reg_write_t rw;
        pl_reg_read_t  rr;
        @(negedge clk);
        if (wr) begin
            rw.addr = a; rw.data = d;
            msg_kind    = MSG_REG_WRITE;
            msg_payload = msg_payload_t'(rw);
        end
        else begin
            rr.addr = a;
            msg_kind    = MSG_REG_READ;
            msg_payload = msg_payload_t'(rr);
        end
        msg_valid = 1'b1;
        forever begin
            @(posedge clk);
            if (msg_ready) break;
        end
        @(negedge clk);
        msg_valid = 1'b0;
    endtask

    task automatic await(input int prev, output bit ok);
        int guard = 0;
        ok = 1'b1;
        while (rsp_pulses == prev) begin
            @(negedge clk);
            guard++;
            if (guard > 80) begin ok = 1'b0; return; end
        end
        @(negedge clk);
    endtask

    bit ok;

    initial begin
        rst_n = 0; msg_valid = 0; msg_kind = MSG_UNKNOWN; msg_payload = '0;
        status_wen = 0; status_addr = '0; status_wdata = '0;
        repeat (4) @(negedge clk);
        rst_n = 1;
        repeat (2) @(negedge clk);

        $display("\n=== T1: register read message reaches the RGF and replies ===");
        p0 = rsp_pulses;
        send_reg(1'b0, IMG_STATUS_ADDR, 32'h0);
        await(p0, ok);
        chk("reply arrived",            ok);
        chk("flagged as a read",        obs_is_read === 1'b1);
        chk("IMG_STATUS height = 256",  obs_rdata[9:0]   == 10'd256);
        chk("IMG_STATUS width  = 256",  obs_rdata[19:10] == 10'd256);
        chk("no error",                 obs_error === 1'b0);

        $display("\n=== T2: register write message lands ===");
        p0 = rsp_pulses;
        send_reg(1'b1, CLK_CTRL_ADDR, 32'h0000_0001);
        await(p0, ok);
        chk("write completed",             ok);
        chk("clk_sel_out followed",        clk_sel === 1'b1);
        p0 = rsp_pulses;
        send_reg(1'b0, CLK_CTRL_ADDR, 32'h0);
        await(p0, ok);
        chk("reads back 1", obs_rdata[0] == 1'b1);

        $display("\n=== T3: BACK-PRESSURE -- msg_ready drops while the bus works ===");
        not_ready_cycles = 0;
        p0 = rsp_pulses;
        send_reg(1'b1, CLK_CTRL_ADDR, 32'h0000_0000);
        await(p0, ok);
        chk("msg_ready dropped during the transfer", not_ready_cycles > 0);
        chk("transfer still completed",              ok);

        $display("\n=== T4: no parking, and IMG_TX_MON still survives idle ===");
        // Set complete via the status port, then sit idle with rgf_cmd_addr
        // left wherever the last message put it. Before step 2 this was only
        // safe because the router parked the address; that parking is gone.
        @(negedge clk);
        status_wen   = 1'b1;
        status_addr  = IMG_TX_MON_ADDR;
        status_wdata = {10'b0, 1'b0, 1'b1, 10'd9, 10'd5};
        @(negedge clk);
        status_wen = 1'b0;
        repeat (25) @(negedge clk);
        chk("rgf_cmd_addr is NOT parked at IDLE_ADDR",
            rgf_cmd_addr !== rgf_pkg::IDLE_ADDR);
        p0 = rsp_pulses;
        send_reg(1'b0, IMG_TX_MON_ADDR, 32'h0);
        await(p0, ok);
        chk("img_send_complete survived 25 idle cycles", obs_rdata[20] == 1'b1);
        chk("row_cnt preserved", obs_rdata[9:0]   == 10'd5);
        chk("col_cnt preserved", obs_rdata[19:10] == 10'd9);

        $display("\n=== T5: read-to-clear still works, interlock re-arms ===");
        p0 = rsp_pulses;
        send_reg(1'b0, IMG_TX_MON_ADDR, 32'h0);
        await(p0, ok);
        chk("complete cleared by the previous read", obs_rdata[20] == 1'b0);
        p0 = rsp_pulses;
        send_reg(1'b1, IMG_CTRL_ADDR, 32'h0000_0001);
        await(p0, ok);
        repeat (2) @(negedge clk);
        chk("IMG_CTRL start accepted after the clear", ok);

        $display("\n=== T6: burst of register messages, none lost ===");
        p0 = rsp_pulses;
        for (int i = 0; i < 6; i++) send_reg(1'b0, IMG_STATUS_ADDR, 32'h0);
        repeat (20) @(negedge clk);
        chk("6 messages produced 6 replies", rsp_pulses == p0 + 6);

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

endmodule : tb_apb_integration
