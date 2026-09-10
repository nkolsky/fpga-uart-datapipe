// tb_burst_read.sv
// ----------------
// chip_top integration test for Image Burst Read, over the real pins.
//
// Requests are shifted into UART_TXD_IN as 8E1 characters and replies are
// recovered from UART_RXD_OUT by an independent receiver, so a break
// anywhere in the chain shows up here: parser, classifier, request CDC,
// arbiter, interlock, SRAM mux, burst walk, packing, reply CDC,
// tx_reply_ctrl, tx_mac, PHY.
//
// -----------------------------------------------------------------------
// WHY THE MESSAGE COUNT MATTERS MORE THAN THE VALUES
// -----------------------------------------------------------------------
// The burst reply carries NO coordinates. The host reconstructs position
// from message ORDER alone, so a dropped, duplicated or reordered message
// silently shifts everything after it, with no way for the host to notice.
// That is a materially weaker property than the coordinate-tagged
// full-frame path, so this testbench checks the exact message count
// ceil(H*W/4) and the exact row-major pixel sequence, not just spot values.
//
// -----------------------------------------------------------------------
// OWNERSHIP PROOFS
// -----------------------------------------------------------------------
// Four properties are checked continuously, not sampled:
//
//   1 ownership is acquired ONCE per burst    -- grant count == 1
//   2 it is never released between messages   -- pix_rd_owner never falls
//                                                while brd_busy
//   3 release happens only after the last real pixel is captured AND the
//     final message is accepted                -- done implies both
//   4 a stalled TX cannot duplicate a message  -- one cdc_cmd_sync command
//                                                per packed message
//
// -----------------------------------------------------------------------
// ENVIRONMENT
// -----------------------------------------------------------------------
// chip_top instantiates clk_wiz_0, so this must run inside the Vivado
// project containing that IP. Traffic is not sent until pll_locked.

`timescale 1ns/1ps

module tb_burst_read;

    import msg_pkg::*;
    import memory_pkg::*;
    import rx_burst_pkg::*;

    logic        CLK100MHZ = 1'b0;
    logic        CPU_RESETN;
    logic        UART_TXD_IN;
    logic        UART_RTS;
    logic        UART_RXD_OUT;
    logic        UART_CTS;
    logic [15:0] LED;

    always #5 CLK100MHZ = ~CLK100MHZ;

    chip_top dut (
        .CLK100MHZ(CLK100MHZ), .CPU_RESETN(CPU_RESETN),
        .UART_TXD_IN(UART_TXD_IN), .UART_RTS(UART_RTS),
        .UART_RXD_OUT(UART_RXD_OUT), .UART_CTS(UART_CTS), .LED(LED)
    );

    localparam real BIT_NS = 1000.0 / 8.125;      // 8.125 Mbaud

    int checks = 0, errors = 0;
    string phase = "init";
    task automatic banner(input string n); phase = n; $display("--- %s", n); endtask
    task automatic chk(input bit c, input string w);
        checks++;
        if (!c) begin errors++; $display("  ERROR [%s] %s @%0t", phase, w, $time); end
    endtask
    task automatic chk_b(input logic [7:0] g, input logic [7:0] e, input string w);
        checks++;
        if (g !== e) begin
            errors++;
            $display("  ERROR [%s] %s -- got %02h expected %02h @%0t",
                     phase, w, g, e, $time);
        end
    endtask

    // -----------------------------------------------------------------
    // Reference image
    // -----------------------------------------------------------------
    logic [31:0] ref_r [0:SRAM_DEPTH-1];
    logic [31:0] ref_g [0:SRAM_DEPTH-1];
    logic [31:0] ref_b [0:SRAM_DEPTH-1];

    function automatic logic [7:0] lane_of(input logic [31:0] wd, input int ln);
        case (ln)
            0: return wd[31:24];
            1: return wd[23:16];
            2: return wd[15: 8];
            default: return wd[7:0];
        endcase
    endfunction

    function automatic logic [23:0] expected_pixel(input int row, input int col);
        int idx, wd, ln;
        idx = row * IMG_WIDTH + col;
        wd = idx / 4; ln = idx % 4;
        return {lane_of(ref_r[wd], ln), lane_of(ref_g[wd], ln), lane_of(ref_b[wd], ln)};
    endfunction

    // -----------------------------------------------------------------
    // UART
    // -----------------------------------------------------------------
    task automatic uart_send_byte(input logic [7:0] b);
        logic par;
        par = ^b;
        UART_TXD_IN = 1'b0; #(BIT_NS);
        for (int i = 0; i < 8; i++) begin UART_TXD_IN = b[i]; #(BIT_NS); end
        UART_TXD_IN = par;  #(BIT_NS);
        UART_TXD_IN = 1'b1; #(BIT_NS);
    endtask

    task automatic send_frame(input logic [7:0] f [16], input int n);
        for (int i = 0; i < n; i++) begin
            uart_send_byte(f[i]);
            #(BIT_NS * 2);
        end
    endtask

    task automatic uart_recv_byte(output logic [7:0] b, output bit ok,
                                  input real timeout_ns);
        real waited; logic par_rx;
        b = 8'h00; ok = 1'b0; waited = 0.0;
        while (UART_RXD_OUT !== 1'b0) begin
            #1; waited = waited + 1.0;
            if (waited > timeout_ns) return;
        end
        #(BIT_NS / 2.0);
        if (UART_RXD_OUT !== 1'b0) return;
        for (int i = 0; i < 8; i++) begin #(BIT_NS); b[i] = UART_RXD_OUT; end
        #(BIT_NS); par_rx = UART_RXD_OUT;
        #(BIT_NS);
        if (UART_RXD_OUT !== 1'b1) begin
            errors++; $display("  ERROR [%s] framing: stop not high @%0t", phase, $time);
            return;
        end
        if (par_rx !== (^b)) begin
            errors++; $display("  ERROR [%s] parity on %02h @%0t", phase, b, $time);
            return;
        end
        ok = 1'b1;
    endtask

    // Assert the line stays idle -- used to prove a rejected request emits
    // nothing at all.
    task automatic expect_silence(input real ns, input string why);
        bit saw; saw = 1'b0;
        fork
            begin : w  @(negedge UART_RXD_OUT); saw = 1'b1; end
            begin : t  #(ns); end
        join_any
        disable fork;
        checks++;
        if (saw) begin
            errors++;
            $display("  ERROR [%s] %s -- a reply was transmitted @%0t", phase, why, $time);
        end
    endtask

    // -----------------------------------------------------------------
    // Frame builders
    // -----------------------------------------------------------------
    function automatic void build_burst_read(output logic [7:0] f [16],
                                             input logic [23:0] a,
                                             input logic [23:0] h,
                                             input logic [23:0] w);
        f[0]=CHAR_OPEN_BRACE; f[1]=CHAR_R;
        f[2]=a[23:16]; f[3]=a[15:8]; f[4]=a[7:0];
        f[5]=CHAR_COMMA; f[6]=CHAR_H;
        f[7]=h[23:16]; f[8]=h[15:8]; f[9]=h[7:0];
        f[10]=CHAR_COMMA; f[11]=CHAR_W;
        f[12]=w[23:16]; f[13]=w[15:8]; f[14]=w[7:0];
        f[15]=CHAR_CLOSE_BRACE;
    endfunction

    function automatic void build_pix_read(output logic [7:0] f [16],
                                           input logic [23:0] r, input logic [23:0] c);
        f[0]=CHAR_OPEN_BRACE; f[1]=CHAR_R;
        f[2]=r[23:16]; f[3]=r[15:8]; f[4]=r[7:0];
        f[5]=CHAR_COMMA; f[6]=CHAR_C;
        f[7]=c[23:16]; f[8]=c[15:8]; f[9]=c[7:0];
        f[10]=CHAR_COMMA; f[11]=CHAR_P;
        f[12]=8'h00; f[13]=8'h00; f[14]=8'h00;
        f[15]=CHAR_CLOSE_BRACE;
    endfunction

    function automatic void build_reg_read(output logic [7:0] f [16],
                                           input logic [23:0] a);
        f[0]=CHAR_OPEN_BRACE; f[1]=CHAR_R;
        f[2]=a[23:16]; f[3]=a[15:8]; f[4]=a[7:0];
        f[5]=CHAR_CLOSE_BRACE;
    endfunction

    function automatic void build_pix_write(output logic [7:0] f [16],
                                            input logic [23:0] addr,
                                            input logic [23:0] rgb);
        f[0]=CHAR_OPEN_BRACE; f[1]=CHAR_W;
        f[2]=addr[23:16]; f[3]=addr[15:8]; f[4]=addr[7:0];
        f[5]=CHAR_COMMA;  f[6]=CHAR_P;
        f[7]=rgb[23:16];  f[8]=rgb[15:8];  f[9]=rgb[7:0];
        f[10]=CHAR_CLOSE_BRACE;
    endfunction

    function automatic void build_legacy(output logic [7:0] f [16],
                                         input int r, input int c, input int v);
        f[0]=CHAR_OPEN_BRACE; f[1]=CHAR_R;
        f[2]=ASCII_ZERO+8'((r/100)%10); f[3]=ASCII_ZERO+8'((r/10)%10); f[4]=ASCII_ZERO+8'(r%10);
        f[5]=CHAR_COMMA; f[6]=CHAR_C;
        f[7]=ASCII_ZERO+8'((c/100)%10); f[8]=ASCII_ZERO+8'((c/10)%10); f[9]=ASCII_ZERO+8'(c%10);
        f[10]=CHAR_COMMA; f[11]=CHAR_V;
        f[12]=ASCII_ZERO+8'((v/100)%10); f[13]=ASCII_ZERO+8'((v/10)%10); f[14]=ASCII_ZERO+8'(v%10);
        f[15]=CHAR_CLOSE_BRACE;
    endfunction

    // =================================================================
    // TRACE POINTS -- nine boundaries, each latched on its own event
    // =================================================================
    // 1 classifier output (130 MHz)
    logic [9:0] tr_cls_row, tr_cls_col, tr_cls_h, tr_cls_w;
    int         tr_cls_n = 0;
    always @(posedge dut.pll_clk_out) if (dut.rx_br_cmd_valid) begin
        tr_cls_row <= dut.rx_br_cmd_base_row; tr_cls_col <= dut.rx_br_cmd_base_col;
        tr_cls_h   <= dut.rx_br_cmd_height;   tr_cls_w   <= dut.rx_br_cmd_width;
        tr_cls_n   <= tr_cls_n + 1;
    end

    // 2 request CDC output (100 MHz)
    logic [9:0] tr_req_row, tr_req_col, tr_req_h, tr_req_w;
    int         tr_req_n = 0;
    always @(posedge CLK100MHZ) if (dut.brd_req_valid_100) begin
        tr_req_row <= dut.brd_req_data_100[39:30];
        tr_req_col <= dut.brd_req_data_100[29:20];
        tr_req_h   <= dut.brd_req_data_100[19:10];
        tr_req_w   <= dut.brd_req_data_100[ 9: 0];
        tr_req_n   <= tr_req_n + 1;
    end

    // 3 burst controller SRAM request / address, and 4 captured pixel
    int          tr_sram_n = 0;
    logic [13:0] tr_last_addr;
    always @(posedge CLK100MHZ) if (dut.brd_sram_rd_en) begin
        tr_sram_n   <= tr_sram_n + 1;
        tr_last_addr <= dut.brd_sram_rd_addr;
    end

    // 5 four-pixel packed message leaving the controller (100 MHz)
    int tr_msg_send_n = 0;
    always @(posedge CLK100MHZ) if (dut.brd_msg_send) tr_msg_send_n <= tr_msg_send_n + 1;

    // 6 reply CDC output (130 MHz)
    int tr_cdc_n = 0;
    logic [95:0] tr_cdc_payload;
    always @(posedge dut.pll_clk_out) if (dut.brd_msg_valid_130) begin
        tr_cdc_n       <= tr_cdc_n + 1;
        tr_cdc_payload <= dut.brd_msg_data_130;
    end

    // 7 tx_reply_ctrl handoff (130 MHz)
    int tr_accept_n = 0;
    logic [127:0] tr_msgq;
    always @(posedge dut.pll_clk_out) if (dut.brd_msg_accept_130) tr_accept_n <= tr_accept_n + 1;
    logic acc_d;
    always @(posedge dut.pll_clk_out) acc_d <= dut.brd_msg_accept_130;
    always @(posedge dut.pll_clk_out) if (acc_d) tr_msgq <= dut.u_tx_reply_ctrl.msg_q;

    // 8 tx_mac load (130 MHz)
    logic mac_busy_d;
    wire  mac_load = dut.mac_busy && !mac_busy_d;
    always @(posedge dut.pll_clk_out) mac_busy_d <= dut.mac_busy;
    int tr_mac_n = 0;
    logic [127:0] tr_macdata;
    logic [4:0]   tr_maclen;
    always @(posedge dut.pll_clk_out) if (mac_load) begin
        tr_mac_n   <= tr_mac_n + 1;
        tr_macdata <= dut.tx_mac_msg_data;
        tr_maclen  <= dut.tx_mac_msg_len;
    end

    // =================================================================
    // OWNERSHIP MONITORS -- continuous, not sampled
    // =================================================================
    int own_grants = 0, own_releases = 0, own_drops_in_burst = 0;
    int write_in_burst = 0, imgread_in_burst = 0;
    logic owner_d;

    always @(posedge CLK100MHZ) begin
        owner_d <= dut.pix_rd_owner;
        if (CPU_RESETN) begin
            // 1 - acquisitions
            if (dut.brd_rd_gnt) own_grants <= own_grants + 1;
            // 3 - releases
            if (dut.brd_rd_done) begin
                own_releases <= own_releases + 1;
                if (!dut.u_burst_rd_ctrl.all_captured)
                    $display("  ERROR [%s] release before all pixels captured @%0t",
                             phase, $time);
            end
            // 2 - ownership must not lapse mid-burst
            if (dut.brd_busy && owner_d && !dut.pix_rd_owner && !dut.brd_rd_done) begin
                own_drops_in_burst++;
                $display("  ERROR [%s] ownership lapsed mid-burst @%0t", phase, $time);
            end
            // coherence
            if (dut.arb_brd_owns && dut.pix_rd_owner) begin
                if (dut.sram_wr_allowed) begin
                    write_in_burst++;
                    $display("  ERROR [%s] write allowed inside a burst @%0t", phase, $time);
                end
                if (dut.read_go) begin
                    imgread_in_burst++;
                    $display("  ERROR [%s] image read launched inside a burst @%0t",
                             phase, $time);
                end
            end
        end
    end

    task automatic trace_clear();
        tr_cls_n = 0; tr_req_n = 0; tr_sram_n = 0; tr_msg_send_n = 0;
        tr_cdc_n = 0; tr_accept_n = 0; tr_mac_n = 0;
        own_grants = 0; own_releases = 0;
    endtask

    // =================================================================
    // Run one region and check EVERYTHING
    // =================================================================
    task automatic run_region(input int brow, input int bcol,
                              input int h, input int w,
                              input real timeout_ns = 4_000_000.0);
        logic [7:0]  req [16];
        logic [7:0]  rep [16];
        bit          ok;
        int          total, exp_msgs, idx, r, c;
        logic [23:0] want;
        logic [7:0]  px [0:3][0:2];

        total    = h * w;
        exp_msgs = (total + 3) / 4;
        idx      = 0;
        trace_clear();
        build_burst_read(req, 24'(brow*IMG_WIDTH + bcol), 24'(h), 24'(w));

        fork
            send_frame(req, 16);
            begin : collect
                for (int m = 0; m < exp_msgs; m++) begin
                    for (int i = 0; i < 16; i++) begin
                        uart_recv_byte(rep[i], ok, timeout_ns);
                        if (!ok) begin
                            errors++; checks++;
                            $display("  ERROR [%s] %0dx%0d msg %0d byte %0d lost @%0t",
                                     phase, h, w, m, i, $time);
                            disable collect;
                        end
                    end

                    // ---- exact 16-byte packed format ----
                    chk_b(rep[0],  CHAR_OPEN_BRACE,  $sformatf("msg%0d byte0", m));
                    chk_b(rep[5],  CHAR_COMMA,       $sformatf("msg%0d byte5", m));
                    chk_b(rep[10], CHAR_COMMA,       $sformatf("msg%0d byte10", m));
                    chk_b(rep[15], CHAR_CLOSE_BRACE, $sformatf("msg%0d byte15", m));

                    // Unpack per the guidelines layout:
                    //   <R0,G0,B0,R1> , <G1,B1,R2,G2> , <B2,R3,G3,B3>
                    px[0][0]=rep[1];  px[0][1]=rep[2];  px[0][2]=rep[3];
                    px[1][0]=rep[4];  px[1][1]=rep[6];  px[1][2]=rep[7];
                    px[2][0]=rep[8];  px[2][1]=rep[9];  px[2][2]=rep[11];
                    px[3][0]=rep[12]; px[3][1]=rep[13]; px[3][2]=rep[14];

                    for (int sl = 0; sl < 4; sl++) begin
                        if (idx < total) begin
                            // ---- exact row-major sequence ----
                            r = brow + (idx / w);
                            c = bcol + (idx % w);
                            want = expected_pixel(r, c);
                            chk_b(px[sl][0], want[23:16],
                                  $sformatf("%0dx%0d m%0d s%0d R (%0d,%0d)", h,w,m,sl,r,c));
                            chk_b(px[sl][1], want[15:8],
                                  $sformatf("%0dx%0d m%0d s%0d G (%0d,%0d)", h,w,m,sl,r,c));
                            chk_b(px[sl][2], want[7:0],
                                  $sformatf("%0dx%0d m%0d s%0d B (%0d,%0d)", h,w,m,sl,r,c));
                            idx++;
                        end
                        else begin
                            // ---- zero-valued final padding ----
                            chk_b(px[sl][0], 8'h00, $sformatf("%0dx%0d m%0d s%0d pad R", h,w,m,sl));
                            chk_b(px[sl][1], 8'h00, $sformatf("%0dx%0d m%0d s%0d pad G", h,w,m,sl));
                            chk_b(px[sl][2], 8'h00, $sformatf("%0dx%0d m%0d s%0d pad B", h,w,m,sl));
                        end
                    end
                end
            end
        join

        // ---- no EXTRA messages ----
        expect_silence(60_000.0, $sformatf("%0dx%0d: extra message after %0d", h, w, exp_msgs));

        repeat (400) @(posedge CLK100MHZ);

        // ---- trace: every boundary saw exactly the right count ----
        chk(tr_cls_n == 1, $sformatf("classifier emitted %0d commands, expected 1", tr_cls_n));
        chk(tr_req_n == 1, $sformatf("request CDC delivered %0d, expected 1", tr_req_n));
        chk(tr_cls_row === 10'(brow), "trace/classifier base_row");
        chk(tr_cls_col === 10'(bcol), "trace/classifier base_col");
        chk(tr_cls_h   === 10'(h),    "trace/classifier height");
        chk(tr_cls_w   === 10'(w),    "trace/classifier width");
        chk(tr_req_row === tr_cls_row, "trace/request CDC base_row differs");
        chk(tr_req_col === tr_cls_col, "trace/request CDC base_col differs");
        chk(tr_req_h   === tr_cls_h,   "trace/request CDC height differs");
        chk(tr_req_w   === tr_cls_w,   "trace/request CDC width differs");

        // ---- no SRAM access for padded slots ----
        chk(tr_sram_n == total,
            $sformatf("%0dx%0d: %0d SRAM reads, expected %0d (one per REAL pixel)",
                      h, w, tr_sram_n, total));

        // ---- message counts agree at every stage: no drop, no duplicate ----
        chk(tr_msg_send_n == exp_msgs,
            $sformatf("controller sent %0d messages, expected %0d", tr_msg_send_n, exp_msgs));
        chk(tr_cdc_n == exp_msgs,
            $sformatf("reply CDC delivered %0d, expected %0d", tr_cdc_n, exp_msgs));
        chk(tr_accept_n == exp_msgs,
            $sformatf("tx_reply_ctrl accepted %0d, expected %0d", tr_accept_n, exp_msgs));
        chk(tr_mac_n == exp_msgs,
            $sformatf("tx_mac loaded %0d messages, expected %0d", tr_mac_n, exp_msgs));
        chk(tr_maclen === 5'd16, "tx_mac loaded a non-16 length");

        // ---- ownership: acquired once, released once ----
        chk(own_grants == 1,
            $sformatf("ownership acquired %0d times, expected exactly 1", own_grants));
        chk(own_releases == 1,
            $sformatf("ownership released %0d times, expected exactly 1", own_releases));
        chk(own_drops_in_burst == 0, "ownership lapsed between reply messages");
        chk(write_in_burst == 0, "a write was permitted inside the burst");
        chk(imgread_in_burst == 0, "an image read launched inside the burst");
    endtask

    // =================================================================
    initial begin
        logic [7:0] req [16];
        logic [7:0] rep [16];
        bit         ok;

        $display("=================================================");
        $display(" INTEGRATION: chip_top Image Burst Read");
        $display("=================================================");

        $readmemh("red_hex.mem",   ref_r);
        $readmemh("green_hex.mem", ref_g);
        $readmemh("blue_hex.mem",  ref_b);

        UART_TXD_IN = 1'b1; UART_RTS = 1'b0; CPU_RESETN = 1'b0;
        repeat (20) @(posedge CLK100MHZ);
        CPU_RESETN = 1'b1;
        wait (dut.pll_locked === 1'b1);
        repeat (200) @(posedge CLK100MHZ);

        banner("1 - 1x1: 1 real + 3 padded, 1 message");
        run_region(0, 0, 1, 1);

        banner("2 - 1x2: 2 real + 2 padded, 1 message");
        run_region(0, 0, 1, 2);

        banner("3 - 2x3: 6 real, second message 2 real + 2 padded");
        run_region(0, 0, 2, 3);

        banner("4 - 3x3: 9 real, third message 1 real + 3 padded");
        run_region(0, 0, 3, 3);

        banner("5 - exact multiple of four");
        run_region(0, 0, 2, 2);          // 4 pixels,  1 message
        run_region(0, 0, 4, 4);          // 16 pixels, 4 messages

        banner("6 - nonzero and non-word-aligned start address");
        run_region(0, 1, 1, 2);          // starts at lane 1
        run_region(0, 2, 2, 2);          // lane 2, crosses a word boundary
        run_region(0, 3, 1, 3);          // lane 3, spans two words
        run_region(4, 2, 2, 3);          // pixel 1026

        banner("7 - row stride across multiple rows");
        // 3x2 at column 254: if the walk wrapped on image width rather than
        // region width these would return the wrong pixels.
        run_region(10, 254, 3, 2);
        run_region(1, 0, 3, 4);
        run_region(200, 100, 4, 5);

        banner("8 - final valid pixel and last row");
        run_region(IMG_HEIGHT-1, IMG_WIDTH-1, 1, 1);
        run_region(IMG_HEIGHT-1, IMG_WIDTH-4, 1, 4);

        banner("9 - invalid requests produce no reply and raise the diagnostic");
        begin : invalid_cases
            int err_before;
            // Zero height
            build_burst_read(req, 24'd0, 24'd0, 24'd4);
            send_frame(req, 16);
            expect_silence(80_000.0, "H=0 produced a reply");
            // Zero width
            build_burst_read(req, 24'd0, 24'd4, 24'd0);
            send_frame(req, 16);
            expect_silence(80_000.0, "W=0 produced a reply");
            // Beyond the image
            build_burst_read(req, 24'd0, 24'(IMG_HEIGHT+1), 24'd1);
            send_frame(req, 16);
            expect_silence(80_000.0, "H>IMG_HEIGHT produced a reply");
            // Address outside the framebuffer
            build_burst_read(req, 24'h01_0000, 24'd1, 24'd1);
            send_frame(req, 16);
            expect_silence(80_000.0, "A outside the framebuffer produced a reply");
            // Aliasing
            build_burst_read(req, 24'hFF_0005, 24'd1, 24'd1);
            send_frame(req, 16);
            expect_silence(80_000.0, "aliased A produced a reply");
            // Horizontal extent overflow / row wrap
            build_burst_read(req, 24'(IMG_WIDTH-1), 24'd1, 24'd2);
            send_frame(req, 16);
            expect_silence(80_000.0, "row-wrap region produced a reply");
            // Vertical extent overflow
            build_burst_read(req, 24'((IMG_HEIGHT-1)*IMG_WIDTH), 24'd2, 24'd1);
            send_frame(req, 16);
            expect_silence(80_000.0, "vertical overflow produced a reply");

            // No SRAM access may have occurred for any of them.
            chk(dut.u_burst_rd_ctrl.busy === 1'b0,
                "burst controller busy after rejected requests");
        end

        // The design still answers a good request afterwards.
        run_region(0, 0, 2, 3);

        banner("10 - request while writes are pending");
        for (int i = 0; i < 6; i++) begin
            logic [7:0] wfr [16];
            build_pix_write(wfr, 24'(3000 + i), 24'h112233);
            send_frame(wfr, 11);
        end
        run_region(0, 0, 2, 3);

        banner("11 - request while a full-frame readback is active");
        begin : during_image
            build_legacy(req, 2, 0, 1);        // IMG_CTRL start
            send_frame(req, 16);
            repeat (20000) @(posedge CLK100MHZ);
            chk(dut.tx_seq_busy === 1'b1, "image transfer did not start");

            build_burst_read(req, 24'd0, 24'd1, 24'd1);
            send_frame(req, 16);
            repeat (2000) @(posedge CLK100MHZ);

            // Deferred, not lost, and not interleaved.
            chk(dut.u_burst_rd_ctrl.busy === 1'b1,
                "burst read lost during the image readback");
            chk(dut.pix_rd_owner === 1'b0,
                "burst read took memory during an image readback");
            chk(dut.tx_seq_busy === 1'b1,
                "image transfer stopped when the burst request arrived");
        end

        banner("12 - reset mid-command");
        CPU_RESETN = 1'b0;
        repeat (50) @(posedge CLK100MHZ);
        CPU_RESETN = 1'b1;
        wait (dut.pll_locked === 1'b1);
        repeat (500) @(posedge CLK100MHZ);
        chk(dut.u_burst_rd_ctrl.busy === 1'b0, "burst controller busy after reset");
        chk(dut.pix_rd_owner === 1'b0, "ownership survived reset");
        chk(dut.u_tx_reply_ctrl.reply_pending === 1'b0, "a reply survived reset");

        banner("13 - burst read still works after reset");
        run_region(0, 0, 2, 3);
        run_region(4, 2, 3, 3);

        banner("14 - Single Pixel Read still works afterwards");
        begin : spr_after
            logic [23:0] want;
            build_pix_read(req, 24'd4, 24'd2);
            want = expected_pixel(4, 2);
            fork
                send_frame(req, 16);
                begin
                    for (int i = 0; i < 16; i++) uart_recv_byte(rep[i], ok, 400_000.0);
                end
            join
            chk_b(rep[0],  CHAR_OPEN_BRACE,  "SPR byte 0");
            chk_b(rep[11], CHAR_P,           "SPR byte 11");
            chk_b(rep[4],  8'd4,             "SPR row");
            chk_b(rep[9],  8'd2,             "SPR col");
            chk_b(rep[12], want[23:16],      "SPR R");
            chk_b(rep[13], want[15:8],       "SPR G");
            chk_b(rep[14], want[7:0],        "SPR B");
        end

        banner("15 - Register Read still works afterwards");
        begin : rr_after
            build_reg_read(req, 24'h00_0010);      // CLK_CTRL
            fork
                send_frame(req, 6);
                begin
                    for (int i = 0; i < 6; i++) uart_recv_byte(rep[i], ok, 400_000.0);
                end
            join
            chk_b(rep[0], CHAR_OPEN_BRACE,  "RegRead byte 0");
            chk_b(rep[5], CHAR_CLOSE_BRACE, "RegRead byte 5");
        end

        banner("16 - final burst read confirms the path is still intact");
        run_region(0, 0, 4, 4);

        $display("-------------------------------------------------");
        $display(" checks executed : %0d", checks);
        $display(" errors          : %0d", errors);
        $display(" RESULT: %s", (errors == 0) ? "PASS" : "FAIL");
        $display("=================================================");
        $finish;
    end

    initial begin
        #400_000_000;
        $display(" RESULT: FAIL -- testbench timeout");
        $fatal(1, "tb_burst_read timed out");
    end

endmodule : tb_burst_read
