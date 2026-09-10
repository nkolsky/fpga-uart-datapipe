// tb_pixel_read.sv
// ----------------
// chip_top serial integration test for Single Pixel Read.
//
// Everything here goes through the real pins. A request is shifted into
// UART_TXD_IN one 8E1 character at a time and the reply is recovered by
// sampling UART_RXD_OUT with an independent receiver. Nothing reaches into
// the design to inject a message or read a result, so a break anywhere in
// the chain -- PHY, MAC, decode, classify, CDC, arbitration, SRAM mux,
// compose, reply queue, MAC, PHY -- shows up here.
//
// -----------------------------------------------------------------------
// EXPECTED DATA
// -----------------------------------------------------------------------
// The expected RGB values are not hardcoded. The testbench loads the same
// three .mem files the SRAMs initialise from into its own reference arrays
// and computes what each pixel must be, using the documented packing:
//
//   word = (row * IMG_WIDTH + col) / 4     lane = index % 4
//   lane 0 is the MOST significant byte of the 32-bit word
//
// So a lane-orientation error cannot be masked by an expectation that was
// derived with the same mistake.
//
// -----------------------------------------------------------------------
// REQUIREMENTS ON THE SIMULATION ENVIRONMENT
// -----------------------------------------------------------------------
// chip_top instantiates clk_wiz_0, a real Xilinx clocking IP. This
// testbench must therefore be run inside the Vivado project that contains
// that IP; it will not elaborate standalone. The testbench waits for
// pll_locked before sending anything, because traffic sent before lock is
// simply lost -- a failure mode this project has already been bitten by.

`timescale 1ns/1ps

module tb_pixel_read;

    import msg_pkg::*;
    import memory_pkg::*;

    // -----------------------------------------------------------------
    // Board-level signals
    // -----------------------------------------------------------------
    logic        CLK100MHZ = 1'b0;
    logic        CPU_RESETN;
    logic        UART_TXD_IN;
    logic        UART_RTS;
    logic        UART_RXD_OUT;
    logic        UART_CTS;
    logic [15:0] LED;

    always #5 CLK100MHZ = ~CLK100MHZ;          // 100 MHz

    chip_top dut (
        .CLK100MHZ    (CLK100MHZ),
        .CPU_RESETN   (CPU_RESETN),
        .UART_TXD_IN  (UART_TXD_IN),
        .UART_RTS     (UART_RTS),
        .UART_RXD_OUT (UART_RXD_OUT),
        .UART_CTS     (UART_CTS),
        .LED          (LED)
    );

    // -----------------------------------------------------------------
    // UART timing
    //
    // 8.125 Mbaud, 8 data bits, EVEN parity, 1 stop bit. The bit period is
    // deliberately a real number: 1/8.125 MHz is 123.0769... ns and
    // rounding it to an integer accumulates roughly a bit of skew across a
    // 16-byte frame.
    // -----------------------------------------------------------------
    localparam real BIT_NS = 1000.0 / 8.125;   // ~123.0769 ns

    // -----------------------------------------------------------------
    // Bookkeeping
    // -----------------------------------------------------------------
    int    checks = 0;
    int    errors = 0;
    string phase  = "init";

    task automatic banner(input string n);
        phase = n;
        $display("--- %s", n);
    endtask

    task automatic chk(input bit c, input string w);
        checks++;
        if (!c) begin
            errors++;
            $display("  ERROR [%s] %s  @%0t", phase, w, $time);
        end
    endtask

    task automatic chk_b(input logic [7:0] got, input logic [7:0] exp,
                         input string w);
        checks++;
        if (got !== exp) begin
            errors++;
            $display("  ERROR [%s] %s -- got %02h, expected %02h  @%0t",
                     phase, w, got, exp, $time);
        end
    endtask

    // -----------------------------------------------------------------
    // Reference image
    // -----------------------------------------------------------------
    logic [31:0] ref_r [0:SRAM_DEPTH-1];
    logic [31:0] ref_g [0:SRAM_DEPTH-1];
    logic [31:0] ref_b [0:SRAM_DEPTH-1];

    // Lane 0 is the most significant byte.
    function automatic logic [7:0] lane_of(input logic [31:0] word,
                                           input int          lane);
        case (lane)
            0: return word[31:24];
            1: return word[23:16];
            2: return word[15: 8];
            default: return word[7:0];
        endcase
    endfunction

    function automatic logic [23:0] expected_pixel(input int row, input int col);
        int idx, word, lane;
        idx  = row * IMG_WIDTH + col;
        word = idx / 4;
        lane = idx % 4;
        return {lane_of(ref_r[word], lane),
                lane_of(ref_g[word], lane),
                lane_of(ref_b[word], lane)};
    endfunction

    // -----------------------------------------------------------------
    // UART transmitter: one 8E1 character onto UART_TXD_IN
    // -----------------------------------------------------------------
    task automatic uart_send_byte(input logic [7:0] b);
        logic par;
        par = ^b;                       // even parity
        UART_TXD_IN = 1'b0;             // start
        #(BIT_NS);
        for (int i = 0; i < 8; i++) begin
            UART_TXD_IN = b[i];         // LSB first
            #(BIT_NS);
        end
        UART_TXD_IN = par;
        #(BIT_NS);
        UART_TXD_IN = 1'b1;             // stop
        #(BIT_NS);
    endtask

    // Inter-character gap. Real hosts do not send back-to-back and the
    // receive path is specified against a paced stream.
    task automatic uart_gap(input int bits = 2);
        #(BIT_NS * bits);
    endtask

    task automatic send_frame(input logic [7:0] f [16], input int n);
        for (int i = 0; i < n; i++) begin
            uart_send_byte(f[i]);
            uart_gap(2);
        end
    endtask

    // -----------------------------------------------------------------
    // UART receiver: recover one 8E1 character from UART_RXD_OUT
    //
    // Samples in the middle of each bit. Reports a parity or framing
    // problem rather than silently returning bad data.
    // -----------------------------------------------------------------
    task automatic uart_recv_byte(output logic [7:0] b,
                                  output bit         ok,
                                  input  real        timeout_ns);
        real waited;
        logic par_rx, par_calc;
        b   = 8'h00;
        ok  = 1'b0;
        waited = 0.0;

        // Wait for the start bit.
        while (UART_RXD_OUT !== 1'b0) begin
            #1;
            waited = waited + 1.0;
            if (waited > timeout_ns) return;    // ok stays 0
        end

        #(BIT_NS / 2.0);                        // middle of the start bit
        if (UART_RXD_OUT !== 1'b0) return;      // false start

        for (int i = 0; i < 8; i++) begin
            #(BIT_NS);
            b[i] = UART_RXD_OUT;
        end

        #(BIT_NS);
        par_rx   = UART_RXD_OUT;
        par_calc = ^b;

        #(BIT_NS);
        if (UART_RXD_OUT !== 1'b1) begin
            $display("  ERROR [%s] framing: stop bit not high @%0t", phase, $time);
            errors++;
            return;
        end
        if (par_rx !== par_calc) begin
            $display("  ERROR [%s] parity: got %b expected %b for %02h @%0t",
                     phase, par_rx, par_calc, b, $time);
            errors++;
            return;
        end
        ok = 1'b1;
    endtask

    // Collect exactly n bytes.
    task automatic uart_recv_frame(output logic [7:0] f [16],
                                   output int         got,
                                   input  int         n,
                                   input  real        timeout_ns);
        bit ok;
        logic [7:0] b;
        got = 0;
        for (int i = 0; i < n; i++) begin
            uart_recv_byte(b, ok, timeout_ns);
            if (!ok) return;
            f[i] = b;
            got++;
        end
    endtask

    // Assert that the line stays idle for a while -- used to prove that a
    // rejected request produces NO reply at all.
    task automatic expect_silence(input real ns, input string why);
        bit saw;
        saw = 1'b0;
        fork
            begin : watch
                @(negedge UART_RXD_OUT);
                saw = 1'b1;
            end
            begin : timer
                #(ns);
            end
        join_any
        disable fork;
        checks++;
        if (saw) begin
            errors++;
            $display("  ERROR [%s] %s -- a reply was transmitted @%0t",
                     phase, why, $time);
        end
    endtask

    // -----------------------------------------------------------------
    // Frame builders
    // -----------------------------------------------------------------

    // Single Pixel Read request. row24/col24 are written in FULL so a test
    // can put rubbish in the high bytes.
    function automatic void build_pix_read(output logic [7:0] f [16],
                                           input logic [23:0] row24,
                                           input logic [23:0] col24);
        f[0]  = CHAR_OPEN_BRACE;
        f[1]  = CHAR_R;
        f[2]  = row24[23:16];
        f[3]  = row24[15: 8];
        f[4]  = row24[ 7: 0];
        f[5]  = CHAR_COMMA;
        f[6]  = CHAR_C;
        f[7]  = col24[23:16];
        f[8]  = col24[15: 8];
        f[9]  = col24[ 7: 0];
        f[10] = CHAR_COMMA;
        f[11] = CHAR_P;
        f[12] = 8'h00;
        f[13] = 8'h00;
        f[14] = 8'h00;
        f[15] = CHAR_CLOSE_BRACE;
    endfunction

    // Legacy {Rnnn,Cnnn,Vnnn} RGF command, ASCII decimal digits.
    function automatic void build_legacy(output logic [7:0] f [16],
                                         input int r, input int c, input int v);
        f[0]  = CHAR_OPEN_BRACE;
        f[1]  = CHAR_R;
        f[2]  = ASCII_ZERO + 8'((r / 100) % 10);
        f[3]  = ASCII_ZERO + 8'((r /  10) % 10);
        f[4]  = ASCII_ZERO + 8'( r        % 10);
        f[5]  = CHAR_COMMA;
        f[6]  = CHAR_C;
        f[7]  = ASCII_ZERO + 8'((c / 100) % 10);
        f[8]  = ASCII_ZERO + 8'((c /  10) % 10);
        f[9]  = ASCII_ZERO + 8'( c        % 10);
        f[10] = CHAR_COMMA;
        f[11] = CHAR_V;
        f[12] = ASCII_ZERO + 8'((v / 100) % 10);
        f[13] = ASCII_ZERO + 8'((v /  10) % 10);
        f[14] = ASCII_ZERO + 8'( v        % 10);
        f[15] = CHAR_CLOSE_BRACE;
    endfunction

    // Single Pixel Write, 11 bytes: {W<A2,A1,A0>, P<R,G,B>}
    function automatic void build_pix_write(output logic [7:0] f [16],
                                            input logic [23:0] addr,
                                            input logic [23:0] rgb);
        f[0]  = CHAR_OPEN_BRACE;
        f[1]  = CHAR_W;
        f[2]  = addr[23:16];
        f[3]  = addr[15: 8];
        f[4]  = addr[ 7: 0];
        f[5]  = CHAR_COMMA;
        f[6]  = CHAR_P;
        f[7]  = rgb[23:16];
        f[8]  = rgb[15: 8];
        f[9]  = rgb[ 7: 0];
        f[10] = CHAR_CLOSE_BRACE;
    endfunction

    // -----------------------------------------------------------------
    // The core check: send a request, expect a correct 16-byte reply.
    // -----------------------------------------------------------------
    task automatic read_pixel_expect(input int row, input int col,
                                     input real timeout_ns = 200_000.0);
        logic [7:0] req [16];
        logic [7:0] rep [16];
        int          got;
        logic [23:0] exp_rgb;
        logic [9:0]  row10, col10;

        row10 = 10'(row);
        col10 = 10'(col);
        build_pix_read(req, 24'(row), 24'(col));
        exp_rgb = expected_pixel(row, col);

        fork
            send_frame(req, 16);
            uart_recv_frame(rep, got, 16, timeout_ns);
        join

        checks++;
        if (got != 16) begin
            errors++;
            $display("  ERROR [%s] (%0d,%0d): got %0d reply bytes, expected 16 @%0t",
                     phase, row, col, got, $time);
            return;
        end

        // Frame
        chk_b(rep[0],  CHAR_OPEN_BRACE,  $sformatf("(%0d,%0d) byte 0", row, col));
        chk_b(rep[1],  CHAR_R,           $sformatf("(%0d,%0d) byte 1", row, col));
        chk_b(rep[5],  CHAR_COMMA,       $sformatf("(%0d,%0d) byte 5", row, col));
        chk_b(rep[6],  CHAR_C,           $sformatf("(%0d,%0d) byte 6", row, col));
        chk_b(rep[10], CHAR_COMMA,       $sformatf("(%0d,%0d) byte 10", row, col));
        chk_b(rep[11], CHAR_P,           $sformatf("(%0d,%0d) byte 11", row, col));
        chk_b(rep[15], CHAR_CLOSE_BRACE, $sformatf("(%0d,%0d) byte 15", row, col));

        // Row echo, as msg_composer packs it
        chk_b(rep[2], 8'h00,                 $sformatf("(%0d,%0d) row byte 2", row, col));
        chk_b(rep[3], {6'b0, row10[9:8]},    $sformatf("(%0d,%0d) row byte 3", row, col));
        chk_b(rep[4], row10[7:0],            $sformatf("(%0d,%0d) row byte 4", row, col));

        // Column echo
        chk_b(rep[7], 8'h00,                 $sformatf("(%0d,%0d) col byte 7", row, col));
        chk_b(rep[8], {6'b0, col10[9:8]},    $sformatf("(%0d,%0d) col byte 8", row, col));
        chk_b(rep[9], col10[7:0],            $sformatf("(%0d,%0d) col byte 9", row, col));

        // RGB
        chk_b(rep[12], exp_rgb[23:16], $sformatf("(%0d,%0d) R", row, col));
        chk_b(rep[13], exp_rgb[15: 8], $sformatf("(%0d,%0d) G", row, col));
        chk_b(rep[14], exp_rgb[ 7: 0], $sformatf("(%0d,%0d) B", row, col));
    endtask


    // -----------------------------------------------------------------
    // END-TO-END PAYLOAD TRACE
    //
    // The all-zero-payload failure was invisible to a reply-level check:
    // the frame was well formed and 16 bytes long, so UART framing, parity
    // and length all passed while every field was zero. These probes make
    // the payload observable at EVERY boundary it crosses, so the next such
    // failure names the stage that dropped it instead of just the endpoint.
    //
    // Six sample points, in order of travel:
    //   1  classifier output, 130 MHz   (parsed row/col)
    //   2  request CDC output, 100 MHz  (atomic {row,col})
    //   3  pixel_rd_ctrl reply, 100 MHz (row/col/pixel, at the send edge)
    //   4  reply CDC output, 130 MHz    (atomic {row,col,pixel})
    //   5  msg_composer inputs, 130 MHz (at the accept edge)
    //   6  the UART bytes themselves    (checked by read_pixel_expect)
    // -----------------------------------------------------------------
    logic [9:0]  tr_cls_row, tr_cls_col;          bit tr_cls_seen;
    logic [9:0]  tr_req_row, tr_req_col;          bit tr_req_seen;
    logic [9:0]  tr_snd_row, tr_snd_col;
    logic [23:0] tr_snd_pix;                      bit tr_snd_seen;
    logic [9:0]  tr_cdc_row, tr_cdc_col;
    logic [23:0] tr_cdc_pix;                      bit tr_cdc_seen;
    logic [9:0]  tr_cmp_row, tr_cmp_col;
    logic [23:0] tr_cmp_pix;                      bit tr_cmp_seen;

    task automatic trace_clear();
        tr_cls_seen = 0; tr_req_seen = 0; tr_snd_seen = 0;
        tr_cdc_seen = 0; tr_cmp_seen = 0;
    endtask

    // 1 - rx_classifier accepted the request (130 MHz)
    always @(posedge dut.pll_clk_out) if (dut.rx_pr_cmd_valid) begin
        tr_cls_row <= dut.rx_pr_cmd_row;
        tr_cls_col <= dut.rx_pr_cmd_col;
        tr_cls_seen <= 1'b1;
    end

    // 2 - request arrived in the memory domain (100 MHz)
    always @(posedge CLK100MHZ) if (dut.pix_req_valid_100) begin
        tr_req_row <= dut.pix_req_data_100[19:10];
        tr_req_col <= dut.pix_req_data_100[ 9: 0];
        tr_req_seen <= 1'b1;
    end

    // 3 - pixel_rd_ctrl offered the reply (100 MHz, the send edge)
    always @(posedge CLK100MHZ) if (dut.pix_rpy_send) begin
        tr_snd_row <= dut.pix_rpy_row;
        tr_snd_col <= dut.pix_rpy_col;
        tr_snd_pix <= dut.pix_rpy_pixel;
        tr_snd_seen <= 1'b1;
    end

    // 4 - the reply crossed coherently (130 MHz)
    always @(posedge dut.pll_clk_out) if (dut.pix_rpy_valid_130) begin
        tr_cdc_row <= dut.pix_rpy_data_130[43:34];
        tr_cdc_col <= dut.pix_rpy_data_130[33:24];
        tr_cdc_pix <= dut.pix_rpy_data_130[23: 0];
        tr_cdc_seen <= 1'b1;
    end

    // 5 - what msg_composer actually saw when the frame was latched (130 MHz)
    always @(posedge dut.pll_clk_out) if (dut.pix_rpy_accept_130) begin
        tr_cmp_row <= dut.u_pix_reply_composer.row;
        tr_cmp_col <= dut.u_pix_reply_composer.col;
        tr_cmp_pix <= dut.u_pix_reply_composer.pixel;
        tr_cmp_seen <= 1'b1;
    end


    // -----------------------------------------------------------------
    // PACKED-BUS TRACE -- msg_composer output through to the PHY
    //
    // The stage-input trace above proved the payload reaches msg_composer
    // intact, so the remaining suspects are all packed 128-bit buses and
    // the handoff timing between them. These probes sample every one.
    //
    // Orientation, verified against the implementations rather than the
    // comments:
    //   msg_composer : output logic [15:0][7:0] msg  -> msg[0] is bits [7:0]
    //   tx_mac       : phy_data <= msg_buf[byte_idx*8 +: 8] -> byte0 = [7:0]
    // Both agree, and tx_sequencer feeds the image path through the SAME
    // msg_composer with the same packing, so orientation is NOT a variable
    // here. The checks below still compare full 128-bit values so a future
    // orientation change is caught rather than assumed away.
    // -----------------------------------------------------------------
    logic [127:0] tr_cmp_out;   bit tr_cmp_out_seen;   // 1 pix_reply_msg @accept
    logic [127:0] tr_msgq;      bit tr_msgq_seen;      // 3 msg_q after accept
    logic [127:0] tr_rrmsg;     logic [4:0] tr_rrlen;
                                bit tr_rrmsg_seen;     // 4 rr_reply_msg/len
    logic [127:0] tr_macdata;   logic [4:0] tr_maclen;
                                bit tr_mac_seen;       // 5 bus at MAC_LOAD
    logic [127:0] tr_macbuf;    logic [3:0] tr_maclast;
                                bit tr_macbuf_seen;    // 6 tx_mac internal
    logic [7:0]   tr_phy [0:15];
    int           tr_phy_n;                            // 7 each phy byte

    // MAC_LOAD detection. mac_busy is registered from next_state, so it is
    // already high in the MAC_LOAD cycle -- the cycle tx_mac captures.
    logic mac_busy_d, mac_load_d;
    wire  mac_load_cycle = dut.mac_busy && !mac_busy_d;

    always @(posedge dut.pll_clk_out) begin
        mac_busy_d <= dut.mac_busy;
        mac_load_d <= mac_load_cycle;
    end

    // 1 - the composer's 128-bit output, at the instant tx_reply_ctrl takes it
    always @(posedge dut.pll_clk_out) if (dut.pix_rpy_accept_130) begin
        tr_cmp_out <= dut.pix_reply_msg;
        tr_cmp_out_seen <= 1'b1;
    end

    // 3 - what actually landed in the pending slot, one cycle later.
    //     An explicit delay register, NOT $past: sampled-value functions are
    //     only legal in assertion/clocking contexts, and an ordinary always
    //     block is neither.
    logic pix_accept_d;
    always @(posedge dut.pll_clk_out) pix_accept_d <= dut.pix_rpy_accept_130;

    always @(posedge dut.pll_clk_out) if (pix_accept_d) begin
        tr_msgq <= dut.u_tx_reply_ctrl.msg_q;
        tr_msgq_seen <= 1'b1;
    end

    // 4 - what tx_reply_ctrl offers when it requests the MAC
    always @(posedge dut.pll_clk_out) if (dut.rr_reply_req) begin
        tr_rrmsg <= dut.rr_reply_msg;
        tr_rrlen <= dut.rr_reply_len;
        tr_rrmsg_seen <= 1'b1;
    end

    // 5 - the bus tx_mac will latch, sampled in the MAC_LOAD cycle itself
    always @(posedge dut.pll_clk_out) if (mac_load_cycle) begin
        tr_macdata <= dut.tx_mac_msg_data;
        tr_maclen  <= dut.tx_mac_msg_len;
        tr_mac_seen <= 1'b1;
    end

    // 6 - what tx_mac really holds, one cycle after the capture
    always @(posedge dut.pll_clk_out) if (mac_load_d) begin
        tr_macbuf  <= dut.u_tx_mac.msg_buf;
        tr_maclast <= dut.u_tx_mac.last_idx;
        tr_macbuf_seen <= 1'b1;
    end

    // 7 - every byte handed to the PHY, in transmission order
    always @(posedge dut.pll_clk_out) if (dut.u_tx_mac.phy_valid) begin
        if (tr_phy_n < 16) tr_phy[tr_phy_n] <= dut.u_tx_mac.phy_data;
        tr_phy_n <= tr_phy_n + 1;
    end

    task automatic packed_trace_clear();
        tr_cmp_out_seen = 0; tr_msgq_seen = 0; tr_rrmsg_seen = 0;
        tr_mac_seen = 0; tr_macbuf_seen = 0; tr_phy_n = 0;
    endtask

    // The reference frame, byte 0 in bits [7:0] -- msg_composer's orientation.
    function automatic logic [127:0] expected_frame(input int row, input int col);
        logic [7:0]  f [16];
        logic [127:0] m;
        logic [23:0] px;
        logic [9:0]  r10, c10;
        r10 = 10'(row); c10 = 10'(col);
        px  = expected_pixel(row, col);
        f[0]  = CHAR_OPEN_BRACE;
        f[1]  = CHAR_R;
        f[2]  = 8'h00;
        f[3]  = {6'b0, r10[9:8]};
        f[4]  = r10[7:0];
        f[5]  = CHAR_COMMA;
        f[6]  = CHAR_C;
        f[7]  = 8'h00;
        f[8]  = {6'b0, c10[9:8]};
        f[9]  = c10[7:0];
        f[10] = CHAR_COMMA;
        f[11] = CHAR_P;
        f[12] = px[23:16];
        f[13] = px[15: 8];
        f[14] = px[ 7: 0];
        f[15] = CHAR_CLOSE_BRACE;
        for (int i = 0; i < 16; i++) m[8*i +: 8] = f[i];   // byte 0 -> [7:0]
        return m;
    endfunction

    // Byte-reverse helper. Written as an explicit loop rather than the
    // streaming operator {<<8{...}}: the loop is universally supported and
    // this is diagnostic code, not something worth a portability risk.
    function automatic logic [127:0] byte_reverse(input logic [127:0] v);
        logic [127:0] r;
        for (int i = 0; i < 16; i++) r[8*i +: 8] = v[8*(15-i) +: 8];
        return r;
    endfunction

    // Reports the FIRST stage at which the frame stops matching, so the
    // boundary is named rather than inferred.
    bit first_bad_reported;
    task automatic stage_chk(input bit seen, input logic [127:0] got,
                             input logic [127:0] exp, input string name);
        checks++;
        if (!seen) begin
            errors++;
            $display("  ERROR [%s] %s : never sampled", phase, name);
        end
        else if (got !== exp) begin
            errors++;
            $display("  ERROR [%s] %s : got %032h", phase, name, got);
            $display("                  expected %032h", exp);
            if (!first_bad_reported) begin
                first_bad_reported = 1'b1;
                $display("  >>> FIRST DIVERGENCE AT: %s", name);
                if (got === 128'd0)
                    $display("  >>> bus is entirely zero here");
                else if (got === byte_reverse(exp))
                    $display("  >>> bus is BYTE-REVERSED here (orientation flip)");
            end
        end
    endtask

    // -----------------------------------------------------------------
    // Test sequence
    // -----------------------------------------------------------------
    initial begin
        logic [7:0] req [16];
        logic [7:0] wfr [16];

        $display("=================================================");
        $display(" INTEGRATION: chip_top Single Pixel Read");
        $display("=================================================");

        $readmemh("red_hex.mem",   ref_r);
        $readmemh("green_hex.mem", ref_g);
        $readmemh("blue_hex.mem",  ref_b);

        UART_TXD_IN = 1'b1;         // idle high
        UART_RTS    = 1'b0;         // active-low: the host is ready
        CPU_RESETN  = 1'b0;

        repeat (20) @(posedge CLK100MHZ);
        CPU_RESETN = 1'b1;

        // Wait for the PLL. Sending before lock loses the message outright.
        wait (dut.pll_locked === 1'b1);
        repeat (200) @(posedge CLK100MHZ);

        // -------------------------------------------------------------
        banner("1 - pixel 0");
        // -------------------------------------------------------------
        read_pixel_expect(0, 0);

        // -------------------------------------------------------------
        banner("2 - all four byte lanes");
        // -------------------------------------------------------------
        read_pixel_expect(0, 0);    // lane 0
        read_pixel_expect(0, 1);    // lane 1
        read_pixel_expect(0, 2);    // lane 2
        read_pixel_expect(0, 3);    // lane 3

        // -------------------------------------------------------------
        banner("3 - pixel 1026");
        // -------------------------------------------------------------
        // 1026 = row 4, col 2 -> word 256, lane 2.
        read_pixel_expect(4, 2);


        // -------------------------------------------------------------
        banner("3b - pixel 1026 traced through every boundary");
        // -------------------------------------------------------------
        // Pixel 1026 = row 4, col 2 -> word 256, lane 2. The same numbers
        // must appear, unchanged, at all six sample points. A stage that
        // zeroes or garbles the payload is named directly.
        begin : trace_1026
            logic [23:0] exp_rgb;
            exp_rgb = expected_pixel(4, 2);

            trace_clear();
            read_pixel_expect(4, 2);
            repeat (200) @(posedge CLK100MHZ);

            chk(tr_cls_seen, "trace: classifier never accepted the request");
            chk(tr_req_seen, "trace: request never reached the 100 MHz domain");
            chk(tr_snd_seen, "trace: pixel_rd_ctrl never offered a reply");
            chk(tr_cdc_seen, "trace: reply never crossed to 130 MHz");
            chk(tr_cmp_seen, "trace: msg_composer input never latched");

            // 1 - parsed row/col at 130 MHz
            chk(tr_cls_row === 10'd4, $sformatf("trace/classifier row = %0d, expected 4", tr_cls_row));
            chk(tr_cls_col === 10'd2, $sformatf("trace/classifier col = %0d, expected 2", tr_cls_col));

            // 2 - request payload at 100 MHz
            chk(tr_req_row === 10'd4, $sformatf("trace/request row = %0d, expected 4", tr_req_row));
            chk(tr_req_col === 10'd2, $sformatf("trace/request col = %0d, expected 2", tr_req_col));

            // 3 - pixel_rd_ctrl reply payload at 100 MHz
            chk(tr_snd_row === 10'd4, $sformatf("trace/ctrl row = %0d, expected 4", tr_snd_row));
            chk(tr_snd_col === 10'd2, $sformatf("trace/ctrl col = %0d, expected 2", tr_snd_col));
            chk(tr_snd_pix === exp_rgb,
                $sformatf("trace/ctrl pixel = %06h, expected %06h", tr_snd_pix, exp_rgb));

            // 4 - CDC output at 130 MHz must be bit-identical to 3
            chk(tr_cdc_row === tr_snd_row,
                $sformatf("trace/CDC row %0d != source %0d", tr_cdc_row, tr_snd_row));
            chk(tr_cdc_col === tr_snd_col,
                $sformatf("trace/CDC col %0d != source %0d", tr_cdc_col, tr_snd_col));
            chk(tr_cdc_pix === tr_snd_pix,
                $sformatf("trace/CDC pixel %06h != source %06h", tr_cdc_pix, tr_snd_pix));

            // 5 - msg_composer must see exactly what crossed
            chk(tr_cmp_row === tr_cdc_row,
                $sformatf("trace/composer row %0d != CDC %0d", tr_cmp_row, tr_cdc_row));
            chk(tr_cmp_col === tr_cdc_col,
                $sformatf("trace/composer col %0d != CDC %0d", tr_cmp_col, tr_cdc_col));
            chk(tr_cmp_pix === tr_cdc_pix,
                $sformatf("trace/composer pixel %06h != CDC %06h", tr_cmp_pix, tr_cdc_pix));

            // 6 - and the absolute values, so an all-zero payload cannot pass
            //     by being consistently zero at every stage.
            chk(tr_cmp_row === 10'd4,  "trace/composer row is not 4");
            chk(tr_cmp_col === 10'd2,  "trace/composer col is not 2");
            chk(tr_cmp_pix === exp_rgb, "trace/composer pixel does not match memory");
            chk(tr_cmp_pix !== 24'd0 || exp_rgb === 24'd0,
                "trace: payload is zero where memory is not");
        end


        // -------------------------------------------------------------
        banner("3c - pixel 1026 traced across every packed bus");
        // -------------------------------------------------------------
        // Row 4, col 2. The identical 128-bit frame must appear at the
        // composer output, in tx_reply_ctrl's slot, on the MAC bus at the
        // capture cycle, inside tx_mac, and byte-for-byte at the PHY.
        begin : packed_1026
            logic [127:0] exp_msg;
            exp_msg = expected_frame(4, 2);
            first_bad_reported = 1'b0;

            packed_trace_clear();
            read_pixel_expect(4, 2);
            repeat (400) @(posedge CLK100MHZ);

            // 1 / 2 - msg_composer output == what tx_reply_ctrl sees
            stage_chk(tr_cmp_out_seen, tr_cmp_out, exp_msg,
                      "1-2 pix_reply_msg / u_tx_reply_ctrl.pix_msg");

            // 3 - latched into the pending slot
            stage_chk(tr_msgq_seen, tr_msgq, exp_msg,
                      "3 u_tx_reply_ctrl.msg_q after pix_accept");

            // 4 - offered to the MAC mux
            stage_chk(tr_rrmsg_seen, tr_rrmsg, exp_msg,
                      "4 rr_reply_msg at rr_reply_req");
            checks++;
            if (tr_rrlen !== 5'd16) begin
                errors++;
                $display("  ERROR [%s] 4 rr_reply_len = %0d, expected 16", phase, tr_rrlen);
            end

            // 5 - THE CAPTURE CYCLE. This is where the mux used to have
            //     already switched back to the idle image frame.
            stage_chk(tr_mac_seen, tr_macdata, exp_msg,
                      "5 tx_mac_msg_data at MAC_LOAD");
            checks++;
            if (tr_maclen !== 5'd16) begin
                errors++;
                $display("  ERROR [%s] 5 tx_mac_msg_len at MAC_LOAD = %0d, expected 16",
                         phase, tr_maclen);
            end

            // 6 - what tx_mac actually holds
            stage_chk(tr_macbuf_seen, tr_macbuf, exp_msg,
                      "6 u_tx_mac.msg_buf after capture");
            checks++;
            if (tr_maclast !== 4'd15) begin
                errors++;
                $display("  ERROR [%s] 6 u_tx_mac.last_idx = %0d, expected 15",
                         phase, tr_maclast);
            end

            // 7 - the bytes on the wire, in order
            checks++;
            if (tr_phy_n != 16) begin
                errors++;
                $display("  ERROR [%s] 7 phy_data byte count = %0d, expected 16",
                         phase, tr_phy_n);
            end
            else begin
                for (int i = 0; i < 16; i++)
                    chk_b(tr_phy[i], exp_msg[8*i +: 8],
                          $sformatf("7 phy_data byte %0d", i));
            end
        end

        // -------------------------------------------------------------
        banner("4 - final valid pixel");
        // -------------------------------------------------------------
        read_pixel_expect(IMG_HEIGHT-1, IMG_WIDTH-1);

        // A couple of interior pixels for good measure.
        read_pixel_expect(1, 0);
        read_pixel_expect(68, 160);
        read_pixel_expect(255, 0);
        read_pixel_expect(0, 255);

        // -------------------------------------------------------------
        banner("5 - out-of-range request");
        // -------------------------------------------------------------
        // Row one past the end. No SRAM access, no reply, and the design
        // must stay usable afterwards.
        build_pix_read(req, 24'(IMG_HEIGHT), 24'd0);
        send_frame(req, 16);
        expect_silence(40_000.0, "row = IMG_HEIGHT produced a reply");

        // Column one past the end.
        build_pix_read(req, 24'd0, 24'(IMG_WIDTH));
        send_frame(req, 16);
        expect_silence(40_000.0, "col = IMG_WIDTH produced a reply");

        // THE ALIASING CASE, end to end. Low ten bits are a legal 5; the
        // high byte is rubbish. If the parser truncates instead of
        // rejecting, a reply for row 5 appears here.
        build_pix_read(req, 24'hFF0005, 24'd0);
        send_frame(req, 16);
        expect_silence(40_000.0, "aliased row 0xFF0005 produced a reply");

        build_pix_read(req, 24'd0, 24'hFF0005);
        send_frame(req, 16);
        expect_silence(40_000.0, "aliased col 0xFF0005 produced a reply");

        // The design still answers a good request after all that.
        read_pixel_expect(3, 3);

        // -------------------------------------------------------------
        banner("6 - request while writes are pending");
        // -------------------------------------------------------------
        // Queue several pixel writes, then immediately ask for a read. The
        // read must be deferred behind the writes and answered correctly,
        // not dropped and not interleaved.
        for (int i = 0; i < 6; i++) begin
            build_pix_write(wfr, 24'(2000 + i), 24'h112233);
            send_frame(wfr, 11);
        end
        read_pixel_expect(9, 9);

        // Read back one of the pixels just written, which also confirms the
        // read and write paths agree about the byte lane.
        begin : write_then_read
            int idx, r, c;
            idx = 2000;
            r   = idx / IMG_WIDTH;
            c   = idx % IMG_WIDTH;
            build_pix_write(wfr, 24'(idx), 24'hA1B2C3);
            send_frame(wfr, 11);
            repeat (2000) @(posedge CLK100MHZ);

            begin : wtr_check
                logic [7:0] rep [16];
                int         got;
                build_pix_read(req, 24'(r), 24'(c));
                fork
                    send_frame(req, 16);
                    uart_recv_frame(rep, got, 16, 200_000.0);
                join
                checks++;
                if (got != 16) begin
                    errors++;
                    $display("  ERROR [%s] write-then-read got %0d bytes @%0t",
                             phase, got, $time);
                end
                else begin
                    chk_b(rep[12], 8'hA1, "write-then-read R");
                    chk_b(rep[13], 8'hB2, "write-then-read G");
                    chk_b(rep[14], 8'hC3, "write-then-read B");
                end
            end
        end

        // -------------------------------------------------------------
        banner("7 - request during a full-image readback");
        // -------------------------------------------------------------
        // Launch an image transfer, let it get properly under way, then
        // send a pixel read. The reply must NOT appear inside the image
        // stream -- doing so would desynchronise the host's byte count for
        // every packet that follows.
        //
        // A full image is ~1.5 s of simulated time, far more than this
        // testbench runs for, so the check here is the one that matters
        // and is affordable: the request is accepted and held, the image
        // stream is not interrupted, and nothing is lost. The reply itself
        // is confirmed after the transfer is abandoned by reset in
        // section 8.
        begin : image_readback

            build_legacy(req, 2, 0, 1);      // IMG_CTRL, write, start = 1
            send_frame(req, 16);

            // Let the image transfer get going.
            repeat (20000) @(posedge CLK100MHZ);
            chk(dut.tx_seq_busy === 1'b1, "image transfer did not start");

            // Now the pixel read request.
            build_pix_read(req, 24'd7, 24'd7);
            send_frame(req, 16);

            // The classifier must have accepted it...
            repeat (500) @(posedge CLK100MHZ);

            // ...and the controller must be holding it, not have lost it.
            chk(dut.u_pixel_rd_ctrl.busy === 1'b1,
                "pixel read was lost during the image readback");

            // The reply must not have been handed to the MAC while the
            // image path is busy.
            chk(dut.u_tx_reply_ctrl.reply_req === 1'b0,
                "reply offered to the MAC during an image transfer");

            // And the image transfer is still running undisturbed.
            chk(dut.tx_seq_busy === 1'b1,
                "image transfer stopped when the pixel read arrived");

            repeat (20000) @(posedge CLK100MHZ);
            chk(dut.u_pixel_rd_ctrl.busy === 1'b1,
                "pixel read did not stay deferred");
            chk(dut.tx_seq_busy === 1'b1,
                "image transfer ended unexpectedly early");
        end

        // -------------------------------------------------------------
        banner("8 - reset");
        // -------------------------------------------------------------
        // Reset out of the image transfer, then confirm the whole path
        // still works from cold.
        CPU_RESETN = 1'b0;
        repeat (50) @(posedge CLK100MHZ);
        CPU_RESETN = 1'b1;
        wait (dut.pll_locked === 1'b1);
        repeat (500) @(posedge CLK100MHZ);

        chk(dut.u_pixel_rd_ctrl.busy === 1'b0,
            "pixel controller still busy after reset");
        chk(dut.u_tx_reply_ctrl.reply_pending === 1'b0,
            "a reply survived reset");

        read_pixel_expect(0, 0);
        read_pixel_expect(4, 2);
        read_pixel_expect(IMG_HEIGHT-1, IMG_WIDTH-1);

        // -------------------------------------------------------------
        banner("9 - diagnostics stayed clean");
        // -------------------------------------------------------------
        chk(dut.u_pixel_rd_ctrl.req_overrun === 1'b0,
            "a pixel request was dropped as an overrun");
        chk(dut.u_tx_reply_ctrl.reply_overrun === 1'b0,
            "a reply was dropped as an overrun");

        // -------------------------------------------------------------
        $display("-------------------------------------------------");
        $display(" checks executed : %0d", checks);
        $display(" errors          : %0d", errors);
        $display(" RESULT: %s", (errors == 0) ? "PASS" : "FAIL");
        $display("=================================================");
        $finish;
    end

    // Global watchdog. Generous: a single 16-byte exchange at 8.125 Mbaud
    // is ~40 us of simulated time and section 7 deliberately runs an image
    // transfer for a while.
    initial begin
        #80_000_000;                       // 80 ms
        $display(" RESULT: FAIL -- testbench timeout");
        $fatal(1, "tb_pixel_read timed out");
    end

endmodule : tb_pixel_read
