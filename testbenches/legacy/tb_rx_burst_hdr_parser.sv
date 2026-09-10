// tb_rx_burst_hdr_parser.sv
// -------------------------
// M1 regression for rx_burst_hdr_parser. Self-checking, no design changes.
//
// The DUT is purely combinational, so there is no clock: stimulus is applied
// and sampled after a settling delta. A timeout guard is still present so a
// hang inside the testbench itself cannot masquerade as a pass.
//
// -----------------------------------------------------------------------
// WHAT THIS IS ACTUALLY PROVING
// -----------------------------------------------------------------------
// Three properties matter more than the individual vectors:
//
//   1. FRAMING AND DIMENSIONS ARE INDEPENDENT. hdr_frame_ok reports only
//      the seven fixed bytes; hdr_dims_ok reports only the extents. A test
//      that checked hdr_valid alone could not distinguish "malformed frame"
//      from "rectangle too large", and rx_burst_ctrl needs to report those
//      differently.
//
//   2. EXTRACTION IS INDEPENDENT OF VALIDATION. height and width are pure
//      slices and must be correct even when the header is rejected --
//      otherwise a diagnostic reading them on a rejected header would be
//      reporting noise.
//
//   3. BYTES 2..4 ARE GENUINELY DON'T-CARE. Scenario 8 sweeps them through
//      hostile values, including the frame delimiters themselves. If the
//      parser ever starts validating them it would reject traffic the
//      specification permits, and that is exactly the sort of change that
//      passes review unnoticed.
//
// -----------------------------------------------------------------------
// FRAME LAYOUT UNDER TEST
// -----------------------------------------------------------------------
//   [0]'{'  [1]'I'  [2..4]don't care  [5]','  [6]'H'  [7..9]H2,H1,H0
//   [10]','  [11]'W'  [12..14]W2,W1,W0  [15]'}'
//
// Byte n occupies msg_in[127 - 8*n -: 8].

`timescale 1ns/1ps

module tb_rx_burst_hdr_parser;

    import msg_pkg::*;
    import rx_burst_pkg::*;

    localparam int MAX_H = memory_pkg::IMG_HEIGHT;   // 256
    localparam int MAX_W = memory_pkg::IMG_WIDTH;    // 256

    logic [127:0]           msg_in;
    logic                   hdr_frame_ok, hdr_dims_ok, hdr_valid;
    logic [BURST_DIM_W-1:0] height, width;

    rx_burst_hdr_parser dut (
        .msg_in       (msg_in),
        .hdr_frame_ok (hdr_frame_ok),
        .hdr_dims_ok  (hdr_dims_ok),
        .hdr_valid    (hdr_valid),
        .height       (height),
        .width        (width)
    );

    // -----------------------------------------------------------------
    // Bookkeeping
    // -----------------------------------------------------------------
    int    errors = 0;
    int    checks = 0;
    string phase  = "init";

    task automatic banner(input string name);
        phase = name;
        $display("--- %s", name);
    endtask

    task automatic chk(input bit cond, input string what);
        checks++;
        if (!cond) begin
            errors++;
            $display("  FAIL (%s): %s", phase, what);
            $display("        msg_in=%032h  frame_ok=%0b dims_ok=%0b valid=%0b h=%06h w=%06h",
                     msg_in, hdr_frame_ok, hdr_dims_ok, hdr_valid, height, width);
        end
    endtask

    task automatic chk_h(input logic [23:0] got, input logic [23:0] exp,
                         input string what);
        checks++;
        if (got !== exp) begin
            errors++;
            $display("  FAIL (%s): %s -- got %06h, expected %06h",
                     phase, what, got, exp);
        end
    endtask

    // Check all three flags at once -- the common case.
    task automatic chk_flags(input bit exp_frame, input bit exp_dims,
                             input string what);
        chk(hdr_frame_ok === exp_frame,
            {what, " : hdr_frame_ok"});
        chk(hdr_dims_ok  === exp_dims,
            {what, " : hdr_dims_ok"});
        chk(hdr_valid    === (exp_frame && exp_dims),
            {what, " : hdr_valid"});
    endtask

    // -----------------------------------------------------------------
    // Frame construction
    // -----------------------------------------------------------------
    task automatic set_byte(input int idx, input logic [7:0] v);
        msg_in[127 - idx*8 -: 8] = v;
    endtask

    // Build a well-formed header. The three don't-care bytes default to
    // zero but can be driven to anything -- scenario 8 relies on that.
    task automatic build_header(input logic [23:0] h,
                                input logic [23:0] w,
                                input logic [7:0]  pad2 = 8'h00,
                                input logic [7:0]  pad3 = 8'h00,
                                input logic [7:0]  pad4 = 8'h00);
        msg_in = '0;
        set_byte(0,  CHAR_OPEN_BRACE);   // '{'
        set_byte(1,  CHAR_I);            // 'I'
        set_byte(2,  pad2);              // don't care
        set_byte(3,  pad3);              // don't care
        set_byte(4,  pad4);              // don't care
        set_byte(5,  CHAR_COMMA);        // ','
        set_byte(6,  CHAR_H);            // 'H'
        set_byte(7,  h[23:16]);
        set_byte(8,  h[15: 8]);
        set_byte(9,  h[ 7: 0]);
        set_byte(10, CHAR_COMMA);        // ','
        set_byte(11, CHAR_W);            // 'W'
        set_byte(12, w[23:16]);
        set_byte(13, w[15: 8]);
        set_byte(14, w[ 7: 0]);
        set_byte(15, CHAR_CLOSE_BRACE);  // '}'
        #1ps;                            // settle the combinational network
    endtask

    // Corrupt one byte of an already-built frame.
    task automatic corrupt(input int idx, input logic [7:0] v);
        set_byte(idx, v);
        #1ps;
    endtask

    // -----------------------------------------------------------------
    initial begin
        $display("=================================================");
        $display(" M1 REGRESSION: rx_burst_hdr_parser");
        $display(" MAX_HEIGHT=%0d  MAX_WIDTH=%0d", MAX_H, MAX_W);
        $display("=================================================");

        msg_in = '0;
        #1ps;

        // =============================================================
        banner("1 - valid 256x256 header");
        // The full-image case: H = W = 256 = 24'h000100.
        // =============================================================
        build_header(24'h00_0100, 24'h00_0100);
        chk_flags(1, 1, "256x256");
        chk_h(height, 24'h00_0100, "height extracted");
        chk_h(width,  24'h00_0100, "width extracted");

        // =============================================================
        banner("2 - valid 1x1 header");
        // The smallest legal rectangle. Also the lower boundary of the
        // non-zero check.
        // =============================================================
        build_header(24'h00_0001, 24'h00_0001);
        chk_flags(1, 1, "1x1");
        chk_h(height, 24'h00_0001, "height extracted");
        chk_h(width,  24'h00_0001, "width extracted");

        // A non-square rectangle, to prove H and W are not transposed.
        build_header(24'h00_0002, 24'h00_0003);
        chk_flags(1, 1, "2x3");
        chk_h(height, 24'h00_0002, "H=2 not swapped with W");
        chk_h(width,  24'h00_0003, "W=3 not swapped with H");

        // =============================================================
        banner("3 - height = 0 rejected");
        // A zero dimension must be refused, not silently treated as an
        // empty burst: it would arm bypass_active for a burst that can
        // never complete, leaving the receive path stuck reading normal
        // frames as pixel data.
        //
        // Framing is untouched, so hdr_frame_ok must STAY HIGH -- this is
        // the independence property.
        // =============================================================
        build_header(24'h00_0000, 24'h00_0100);
        chk_flags(1, 0, "H=0");
        chk_h(height, 24'h00_0000, "height still extracted when rejected");
        chk_h(width,  24'h00_0100, "width unaffected");

        // =============================================================
        banner("4 - width = 0 rejected");
        // =============================================================
        build_header(24'h00_0100, 24'h00_0000);
        chk_flags(1, 0, "W=0");
        chk_h(width, 24'h00_0000, "width still extracted when rejected");

        // Both zero.
        build_header(24'h00_0000, 24'h00_0000);
        chk_flags(1, 0, "H=0 and W=0");

        // =============================================================
        banner("5 - height above the image rejected");
        // 256 is the last legal value; 257 the first illegal one.
        // =============================================================
        build_header(BURST_DIM_W'(MAX_H), 24'h00_0100);
        chk_flags(1, 1, "H = MAX_HEIGHT (boundary, legal)");

        build_header(BURST_DIM_W'(MAX_H + 1), 24'h00_0100);
        chk_flags(1, 0, "H = MAX_HEIGHT+1 (boundary, illegal)");
        chk_h(height, BURST_DIM_W'(MAX_H + 1), "height extracted when over range");

        build_header(24'hFF_FFFF, 24'h00_0100);
        chk_flags(1, 0, "H = 0xFFFFFF");

        // =============================================================
        banner("6 - width above the image rejected");
        // =============================================================
        build_header(24'h00_0100, BURST_DIM_W'(MAX_W));
        chk_flags(1, 1, "W = MAX_WIDTH (boundary, legal)");

        build_header(24'h00_0100, BURST_DIM_W'(MAX_W + 1));
        chk_flags(1, 0, "W = MAX_WIDTH+1 (boundary, illegal)");

        build_header(24'h00_0100, 24'hFF_FFFF);
        chk_flags(1, 0, "W = 0xFFFFFF");

        // =============================================================
        banner("7 - malformed frames");
        // Each of the seven fixed bytes corrupted in turn. Dimensions are
        // left legal throughout, so hdr_dims_ok must STAY HIGH while
        // hdr_frame_ok drops -- the independence property from the other
        // direction.
        // =============================================================
        build_header(24'h00_0100, 24'h00_0100); corrupt(0,  8'h00);
        chk_flags(0, 1, "byte 0 not '{'");

        build_header(24'h00_0100, 24'h00_0100); corrupt(1,  CHAR_W);
        chk_flags(0, 1, "byte 1 'W' not 'I'");

        build_header(24'h00_0100, 24'h00_0100); corrupt(1,  CHAR_R);
        chk_flags(0, 1, "byte 1 'R' not 'I'");

        build_header(24'h00_0100, 24'h00_0100); corrupt(5,  8'h00);
        chk_flags(0, 1, "byte 5 not ','");

        build_header(24'h00_0100, 24'h00_0100); corrupt(6,  CHAR_C);
        chk_flags(0, 1, "byte 6 'C' not 'H'");

        build_header(24'h00_0100, 24'h00_0100); corrupt(10, 8'h00);
        chk_flags(0, 1, "byte 10 not ','");

        build_header(24'h00_0100, 24'h00_0100); corrupt(11, CHAR_P);
        chk_flags(0, 1, "byte 11 'P' not 'W'");

        build_header(24'h00_0100, 24'h00_0100); corrupt(15, 8'h00);
        chk_flags(0, 1, "byte 15 not '}'");

        // Malformed AND out of range -- both flags must be low.
        build_header(24'h00_0000, 24'hFF_FFFF); corrupt(1, 8'h00);
        chk_flags(0, 0, "malformed frame with illegal dimensions");

        // An all-zero frame is not a header.
        msg_in = '0; #1ps;
        chk_flags(0, 0, "all-zero frame");

        // Nor is an all-ones frame.
        msg_in = '1; #1ps;
        chk(!hdr_valid, "all-ones frame rejected");

        // =============================================================
        banner("8 - bytes 2..4 are genuinely don't-care");
        // The spec leaves group A of the burst-write header blank, so the
        // parser must not inspect it. Sweeping these through hostile
        // values -- including the frame delimiters and every opcode the
        // protocol uses -- must leave the verdict and the extracted
        // dimensions completely unchanged.
        //
        // If someone later "tightens" the parser by requiring these to be
        // zero, this scenario is what catches it.
        // =============================================================
        pad_sweep: begin
            automatic logic [7:0] hostile [12];
            hostile = '{8'h00, 8'hFF, 8'h55, 8'hAA,
                        CHAR_OPEN_BRACE, CHAR_CLOSE_BRACE, CHAR_COMMA,
                        CHAR_R, CHAR_C, CHAR_V, CHAR_P, CHAR_I};

            foreach (hostile[i]) begin
                build_header(24'h00_0100, 24'h00_0100,
                             hostile[i], hostile[i], hostile[i]);
                chk_flags(1, 1, $sformatf("pad bytes = %02h", hostile[i]));
                chk_h(height, 24'h00_0100, "height unaffected by pad bytes");
                chk_h(width,  24'h00_0100, "width unaffected by pad bytes");
            end

            // All three pad bytes different from each other.
            build_header(24'h00_0080, 24'h00_0040, 8'h7B, 8'h2C, 8'h7D);
            chk_flags(1, 1, "pad bytes all different, all delimiters");
            chk_h(height, 24'h00_0080, "height with delimiter pads");
            chk_h(width,  24'h00_0040, "width with delimiter pads");
        end

        // =============================================================
        banner("9 - payload bytes may hold delimiter values");
        // H and W are raw binary, so a dimension byte can legitimately
        // equal '{', '}' or ','. The fixed-byte checks must not be
        // confused by that -- they only ever look at positions payload
        // never occupies.
        //
        // 0x7B2C7D is far above the image, so the frame is well formed but
        // the rectangle is rejected: frame_ok high, dims_ok low.
        // =============================================================
        build_header(24'h7B_2C7D, 24'h00_0100);
        chk_flags(1, 0, "height bytes are delimiters");
        chk_h(height, 24'h7B_2C7D, "height extracted despite delimiter bytes");

        build_header(24'h00_0100, 24'h7D_7B_2C);
        chk_flags(1, 0, "width bytes are delimiters");
        chk_h(width, 24'h7D_7B_2C, "width extracted despite delimiter bytes");

        // A legal rectangle whose low bytes happen to be delimiter values.
        // 0x00007B = 123 and 0x00002C = 44, both within the image.
        build_header(24'h00_007B, 24'h00_002C);
        chk_flags(1, 1, "H=123 W=44 with delimiter-valued low bytes");
        chk_h(height, 24'h00_007B, "height = 123");
        chk_h(width,  24'h00_002C, "width  = 44");

        // =============================================================
        $display("-------------------------------------------------");
        $display(" checks executed : %0d", checks);
        $display(" errors          : %0d", errors);
        $display(" RESULT: %s", (errors == 0) ? "PASS" : "FAIL");
        $display("=================================================");
        if (errors != 0) $fatal(1, "tb_rx_burst_hdr_parser FAILED");
        $finish;
    end

    // Guard: a hang must not look like a pass.
    initial begin
        #1ms;
        $display(" RESULT: FAIL -- testbench timeout");
        $fatal(1, "tb_rx_burst_hdr_parser timed out");
    end

endmodule : tb_rx_burst_hdr_parser
