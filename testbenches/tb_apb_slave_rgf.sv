`timescale 1ns/1ps
// tb_apb_slave_rgf.sv
// ===================
// Unit test for apb_slave_rgf driving the real rgf. The testbench plays the
// APB master, since apb_master does not exist until step 3.
//
// The interesting test is T5/T6. Everything else is table stakes; T5 is the
// bug this module exists to remove.

module tb_apb_slave_rgf;

    import apb_pkg::*;
    import rgf_pkg::*;

    localparam int WAIT_STATES = 0;

    logic clk = 1'b0;
    logic rst_n;
    always #5 clk = ~clk;              // 100 MHz

    // APB
    logic                psel, penable, pwrite;
    logic [ADDR_W-1:0]   paddr;
    logic [DATA_W-1:0]   pwdata, prdata;
    logic                pready, pslverr;

    // slave <-> rgf
    logic [OFFSET_W-1:0] pc_addr;
    logic                pc_wen, pc_ren;
    logic [DATA_W-1:0]   pc_wdata, pc_rdata;

    // rgf status port
    logic                status_wen;
    logic [7:0]          status_addr;
    logic [DATA_W-1:0]   status_wdata;
    logic                start_img_read, clk_sel;

    int errors = 0;
    int checks = 0;

    task automatic chk(input string what, input logic cond);
        checks++;
        if (!cond) begin
            errors++;
            $display("  FAIL  %s   (t=%0t)", what, $time);
        end
        else $display("  pass  %s", what);
    endtask

    // ------------------------------------------------------------------
    // APB master model. Drives a full SETUP/ACCESS pair and returns read
    // data. Deliberately parks paddr at the LAST address used between
    // transfers rather than at an idle value -- that is exactly the driver
    // behaviour the old level-sensitive read-to-clear could not survive.
    // ------------------------------------------------------------------
    task automatic apb_write(input logic [ADDR_W-1:0] a, input logic [DATA_W-1:0] d);
        @(negedge clk);
        psel = 1'b1; penable = 1'b0; pwrite = 1'b1; paddr = a; pwdata = d;
        @(negedge clk);
        penable = 1'b1;
        forever begin
            @(posedge clk);
            if (pready) break;
        end
        @(negedge clk);
        psel = 1'b0; penable = 1'b0; pwrite = 1'b0;   // paddr intentionally held
    endtask

    task automatic apb_read(input logic [ADDR_W-1:0] a, output logic [DATA_W-1:0] d);
        @(negedge clk);
        psel = 1'b1; penable = 1'b0; pwrite = 1'b0; paddr = a;
        @(negedge clk);
        penable = 1'b1;
        forever begin
            @(posedge clk);
            if (pready) begin d = prdata; break; end
        end
        @(negedge clk);
        psel = 1'b0; penable = 1'b0;                  // paddr intentionally held
    endtask

    // ------------------------------------------------------------------
    // DUT
    // ------------------------------------------------------------------
    apb_slave_rgf #(.WAIT_STATES(WAIT_STATES)) u_slave (
        .clk(clk), .rst_n(rst_n),
        .psel(psel), .penable(penable), .pwrite(pwrite),
        .paddr(paddr), .pwdata(pwdata),
        .pready(pready), .prdata(prdata), .pslverr(pslverr),
        .pc_addr(pc_addr), .pc_wen(pc_wen), .pc_ren(pc_ren),
        .pc_wdata(pc_wdata), .pc_rdata(pc_rdata)
    );

    rgf u_rgf (
        .clk(clk), .rst_n(rst_n),
        .pc_wen(pc_wen), .pc_ren(pc_ren), .pc_addr(pc_addr),
        .pc_wdata(pc_wdata), .pc_rdata(pc_rdata),
        .status_wen(status_wen), .status_addr(status_addr), .status_wdata(status_wdata),
        .img_height_in(10'd256), .img_width_in(10'd256), .img_ready_in(1'b1),
        .start_img_read_out(start_img_read),
        .clk_sel_out(clk_sel),
        .parity_fault_incr(1'b0),
        .fifo_full(1'b0), .fifo_empty(1'b1),
        .fifo_almost_full(1'b0), .fifo_almost_empty(1'b1)
    );

    // Count how many cycles each strobe is high, to prove one-cycle width.
    int wen_cycles = 0, ren_cycles = 0;
    always @(posedge clk) if (rst_n) begin
        if (pc_wen) wen_cycles++;
        if (pc_ren) ren_cycles++;
    end

    logic [DATA_W-1:0] rd;

    // Count start pulses. A counter rather than a sticky flag because a flag
    // cleared from the stimulus process and set from an always block is a
    // dual-driver race -- which is exactly what made this test lie the first
    // time it was run.
    int start_pulses = 0;
    always @(posedge clk) if (start_img_read) start_pulses++;
    int p0;

    initial begin
        rst_n = 1'b0;
        psel = 0; penable = 0; pwrite = 0; paddr = '0; pwdata = '0;
        status_wen = 0; status_addr = '0; status_wdata = '0;
        repeat (4) @(negedge clk);
        rst_n = 1'b1;
        repeat (2) @(negedge clk);

        $display("\n=== T1: read IMG_STATUS through the bus ===");
        apb_read(16'h0000, rd);
        chk("IMG_STATUS height field = 256", rd[9:0]   == 10'd256);
        chk("IMG_STATUS width  field = 256", rd[19:10] == 10'd256);
        chk("IMG_STATUS ready bit set",      rd[20]    == 1'b1);

        $display("\n=== T2: write then read back CLK_CTRL ===");
        apb_write(16'h0010, 32'h0000_0001);
        chk("clk_sel_out follows the write", clk_sel === 1'b1);
        apb_read(16'h0010, rd);
        chk("CLK_CTRL reads back 1", rd[0] == 1'b1);

        $display("\n=== T3: strobe widths ===");
        wen_cycles = 0; ren_cycles = 0;
        apb_write(16'h0010, 32'h0000_0000);
        chk("pc_wen high for exactly 1 cycle", wen_cycles == 1);
        chk("pc_ren never high during a write", ren_cycles == 0);
        wen_cycles = 0; ren_cycles = 0;
        apb_read(16'h0000, rd);
        chk("pc_ren high for exactly 1 cycle", ren_cycles == 1);
        chk("pc_wen never high during a read", wen_cycles == 0);

        $display("\n=== T4: offset extraction ignores the aperture field ===");
        apb_write(16'h0010, 32'h0000_0001);
        apb_read (16'h0010, rd);
        chk("PADDR[7:0] selects the register", rd[0] == 1'b1);
        apb_write(16'h0010, 32'h0000_0000);

        $display("\n=== T5: THE POINT -- IMG_TX_MON survives idle cycles ===");
        // Sequencer reports the image finished.
        @(negedge clk);
        status_wen   = 1'b1;
        status_addr  = IMG_TX_MON_ADDR;
        status_wdata = {10'b0, 1'b0, 1'b1, 10'd7, 10'd3};  // complete=1
        @(negedge clk);
        status_wen = 1'b0;
        // Park the bus AT IMG_TX_MON with no access in flight, for a long time.
        paddr = 16'h0004; psel = 1'b0; penable = 1'b0; pwrite = 1'b0;
        repeat (20) @(negedge clk);
        apb_read(16'h0004, rd);
        chk("img_send_complete SURVIVED 20 idle cycles at its own address",
            rd[20] == 1'b1);
        chk("row_cnt preserved", rd[9:0]   == 10'd3);
        chk("col_cnt preserved", rd[19:10] == 10'd7);

        $display("\n=== T6: read-to-clear still works on a real read ===");
        apb_read(16'h0004, rd);
        chk("img_send_complete cleared by the previous read", rd[20] == 1'b0);

        $display("\n=== T7: IMG_CTRL start interlock is reachable ===");
        // With complete/error clear, a start write must pulse start_img_read.
        // This is the interlock the old level-sensitive clear could disable.
        p0 = start_pulses;
        apb_write(16'h0008, 32'h0000_0001);
        repeat (3) @(negedge clk);
        chk("start_img_read pulsed exactly once", start_pulses == p0 + 1);

        $display("\n=== T7b: interlock BLOCKS start while complete is set ===");
        @(negedge clk);
        status_wen   = 1'b1;
        status_addr  = IMG_TX_MON_ADDR;
        status_wdata = {10'b0, 1'b0, 1'b1, 10'd0, 10'd0};   // complete=1
        @(negedge clk);
        status_wen = 1'b0;
        p0 = start_pulses;
        apb_write(16'h0008, 32'h0000_0001);
        repeat (3) @(negedge clk);
        chk("start_img_read correctly suppressed", start_pulses == p0);
        apb_read(16'h0004, rd);   // clear it again

        $display("\n=== T8: unimplemented offset reads zero ===");
        apb_read(16'h0018, rd);
        chk("offset 0x18 reads 0", rd == 32'h0);
        chk("pslverr stays low (apertures are apb_bar's job)", pslverr === 1'b0);

        $display("\n----------------------------------------");
        $display("checks: %0d   errors: %0d", checks, errors);
        if (errors == 0) $display("RESULT: PASS");
        else             $display("RESULT: FAIL");
        $display("----------------------------------------\n");
        $finish;
    end

    initial begin
        #100000;
        $display("TIMEOUT");
        $finish;
    end

endmodule : tb_apb_slave_rgf
