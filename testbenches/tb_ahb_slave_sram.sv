`timescale 1ns/1ps
// tb_ahb_slave_sram.sv
// ====================
// Unit test for ahb_slave_sram driving a real rgb_sram. The testbench plays
// the AHB manager, since ahb_master does not exist until step 3.
//
// The tests that matter are T3 and T4: an INCR4 read burst returning the
// right four words with no gaps, and a write whose data arrives a cycle
// after its address -- the phase offset that makes an AHB slave different
// from a plain memory port.

module tb_ahb_slave_sram;

    import ahb_pkg::*;

    localparam int SRAM_ADDR_W = 14;
    localparam int DEPTH       = 16384;

    logic hclk = 1'b0;
    logic hresetn;
    always #5 hclk = ~hclk;                      // 100 MHz

    // AHB
    logic                  hsel, hready, hwrite;
    logic [ADDR_W-1:0]     haddr;
    logic [2:0]            hsize, hburst;
    logic [1:0]            htrans;
    logic [DATA_W-1:0]     hwdata, hrdata;
    logic                  hreadyout, hresp;

    // slave <-> sram
    logic                  rd_en, wr_en;
    logic [SRAM_ADDR_W-1:0] rd_addr, wr_addr;
    logic [DATA_W-1:0]     rd_data, wr_data;
    logic [DATA_W/8-1:0]   wr_be;

    int errors = 0, checks = 0;
    task automatic chk(input string what, input logic cond);
        checks++;
        if (!cond) begin errors++; $display("  FAIL  %s   (t=%0t)", what, $time); end
        else $display("  pass  %s", what);
    endtask

    // ------------------------------------------------------------------
    ahb_slave_sram #(.SRAM_ADDR_W(SRAM_ADDR_W)) u_dut (
        .hclk(hclk), .hresetn(hresetn),
        .hsel(hsel), .hready(hready), .haddr(haddr), .hwrite(hwrite),
        .hsize(hsize), .hburst(hburst), .htrans(htrans), .hwdata(hwdata),
        .hreadyout(hreadyout), .hrdata(hrdata), .hresp(hresp),
        .sram_rd_en(rd_en), .sram_rd_addr(rd_addr), .sram_rd_data(rd_data),
        .sram_wr_en(wr_en), .sram_wr_be(wr_be),
        .sram_wr_addr(wr_addr), .sram_wr_data(wr_data)
    );

    rgb_sram #(.DATA_WIDTH(DATA_W), .DEPTH(DEPTH), .INIT_FILE("")) u_sram (
        .clk(hclk),
        .rd_en(rd_en), .rd_addr(rd_addr), .rd_data(rd_data),
        .wr_en(wr_en), .wr_be(wr_be), .wr_addr(wr_addr), .wr_data(wr_data)
    );

    // The manager drives hready from the segment; with one slave it is the
    // slave's own hreadyout, which is what a single-slave segment looks like.
    assign hready = hreadyout;

    // ------------------------------------------------------------------
    // Manager model
    // ------------------------------------------------------------------
    task automatic idle();
        @(negedge hclk);
        hsel = 1'b0; htrans = HTRANS_IDLE; hwrite = 1'b0;
        haddr = '0; hburst = HBURST_SINGLE;
    endtask

    // Single word write. Address in cycle N, data in cycle N+1.
    task automatic ahb_write(input logic [WORD_W-1:0] word_idx,
                             input logic [DATA_W-1:0] data);
        @(negedge hclk);
        hsel = 1'b1; htrans = HTRANS_NONSEQ; hwrite = 1'b1;
        hsize = HSIZE_WORD; hburst = HBURST_SINGLE;
        haddr = addr_of(2'b00, word_idx);
        @(negedge hclk);
        hsel = 1'b0; htrans = HTRANS_IDLE; hwrite = 1'b0;
        hwdata = data;                       // data phase
        @(negedge hclk);
        hwdata = '0;
    endtask

    task automatic ahb_read(input logic [WORD_W-1:0] word_idx,
                            output logic [DATA_W-1:0] data);
        @(negedge hclk);
        hsel = 1'b1; htrans = HTRANS_NONSEQ; hwrite = 1'b0;
        hsize = HSIZE_WORD; hburst = HBURST_SINGLE;
        haddr = addr_of(2'b00, word_idx);
        @(negedge hclk);
        hsel = 1'b0; htrans = HTRANS_IDLE;
        @(posedge hclk);
        data = hrdata;                       // data phase
    endtask

    // INCR4 read: NONSEQ then three SEQ, addresses stepping one word.
    task automatic ahb_read_incr4(input logic [WORD_W-1:0] base,
                                  output logic [DATA_W-1:0] d [4]);
        @(negedge hclk);
        hsel = 1'b1; hwrite = 1'b0; hsize = HSIZE_WORD; hburst = HBURST_INCR4;
        htrans = HTRANS_NONSEQ; haddr = addr_of(2'b00, base);
        for (int i = 1; i < 4; i++) begin
            @(negedge hclk);
            htrans = HTRANS_SEQ;
            haddr  = addr_of(2'b00, WORD_W'(base + i));
            @(posedge hclk);
            d[i-1] = hrdata;                 // data for beat i-1
        end
        @(negedge hclk);
        hsel = 1'b0; htrans = HTRANS_IDLE;
        @(posedge hclk);
        d[3] = hrdata;                       // last beat's data
    endtask

    logic [DATA_W-1:0] rd, burst [4];
    int rd_en_cycles;

    always @(posedge hclk) if (hresetn && rd_en) rd_en_cycles++;

    initial begin
        hresetn = 1'b0;
        hsel = 0; htrans = HTRANS_IDLE; hwrite = 0; haddr = '0;
        hsize = HSIZE_WORD; hburst = HBURST_SINGLE; hwdata = '0;
        repeat (4) @(negedge hclk);
        hresetn = 1'b1;
        repeat (2) @(negedge hclk);

        $display("\n=== T1: idle drives nothing at the SRAM ===");
        idle();
        repeat (3) @(posedge hclk);
        chk("rd_en low when idle", rd_en === 1'b0);
        chk("wr_en low when idle", wr_en === 1'b0);
        chk("hreadyout high when idle", hreadyout === 1'b1);
        chk("hresp OKAY", hresp === HRESP_OKAY);

        $display("\n=== T2: single word write, then read it back ===");
        ahb_write(14'd7, 32'hDEAD_BEEF);
        idle();
        ahb_read(14'd7, rd);
        chk("read returns what was written", rd === 32'hDEAD_BEEF);
        chk("all four byte lanes written", wr_be === 4'b1111);

        $display("\n=== T3: write data arrives one cycle after its address ===");
        // If the slave issued the write from the address phase it would
        // capture whatever was on hwdata a cycle early -- here, zero.
        ahb_write(14'd9, 32'hCAFE_F00D);
        idle();
        ahb_read(14'd9, rd);
        chk("correct data stored, not the previous cycle's", rd === 32'hCAFE_F00D);

        $display("\n=== T4: INCR4 read burst ===");
        for (int i = 0; i < 4; i++) begin
            ahb_write(WORD_W'(100 + i), 32'h1000_0000 + 32'(i));
            idle();
        end
        rd_en_cycles = 0;
        ahb_read_incr4(14'd100, burst);
        chk("beat 0 correct", burst[0] === 32'h1000_0000);
        chk("beat 1 correct", burst[1] === 32'h1000_0001);
        chk("beat 2 correct", burst[2] === 32'h1000_0002);
        chk("beat 3 correct", burst[3] === 32'h1000_0003);
        chk("four SRAM reads, no gaps or repeats", rd_en_cycles == 4);

        $display("\n=== T5: BUSY inside a burst moves no data ===");
        // BUSY means the manager is stalling but the burst is not over.
        @(negedge hclk);
        hsel = 1'b1; hwrite = 1'b0; hsize = HSIZE_WORD; hburst = HBURST_INCR4;
        htrans = HTRANS_NONSEQ; haddr = addr_of(2'b00, 14'd100);
        @(negedge hclk);
        rd_en_cycles = 0;
        htrans = HTRANS_BUSY;
        repeat (3) @(posedge hclk);
        chk("BUSY beats issue no SRAM read", rd_en_cycles == 0);
        chk("hreadyout stays high through BUSY", hreadyout === 1'b1);
        idle();

        $display("\n=== T6: deselected slave ignores the bus ===");
        @(negedge hclk);
        hsel = 1'b0; htrans = HTRANS_NONSEQ; hwrite = 1'b1;
        haddr = addr_of(2'b00, 14'd7); hwdata = 32'hFFFF_FFFF;
        repeat (2) @(posedge hclk);
        chk("wr_en low while deselected", wr_en === 1'b0);
        idle();
        ahb_read(14'd7, rd);
        chk("memory untouched by the deselected transfer", rd === 32'hDEAD_BEEF);

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

endmodule : tb_ahb_slave_sram
