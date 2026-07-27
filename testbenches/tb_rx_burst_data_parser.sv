// tb_rx_burst_data_parser.sv
// --------------------------
// M2 regression for rx_burst_data_parser. Self-checking, no design changes.
//
// Purely combinational DUT, so no clock: stimulus is applied and sampled
// after a settling delta. A timeout guard is present so a hang inside the
// testbench cannot masquerade as a pass.
//
// -----------------------------------------------------------------------
// THE PROPERTY THAT MATTERS
// -----------------------------------------------------------------------
// pixel0 (bytes 1,2,3) and pixel3 (bytes 12,13,14) are contiguous slices,
// but pixel1 (bytes 4,6,7) and pixel2 (bytes 8,9,11) STRADDLE the commas at
// bytes 5 and 10. A transposition or an off-by-one would almost certainly
// land in those two, and a test using symmetric data would not see it.
//
// Scenario 3 therefore isolates every one of the twelve payload bytes in
// turn: one byte set to 0xFF, all others zero, and exactly one channel of
// exactly one pixel must respond. Twelve vectors, twelve independent
// assertions -- that is the strongest available proof of the mapping and it
// does not depend on choosing lucky test data.
//
// -----------------------------------------------------------------------
// FRAME UNDER TEST
// -----------------------------------------------------------------------
//   [0]'{'  [1]R0 [2]G0 [3]B0  [4]R1  [5]','  [6]G1 [7]B1 [8]R2 [9]G2
//   [10]','  [11]B2 [12]R3 [13]G3 [14]B3  [15]'}'

`timescale 1ns/1ps

module tb_rx_burst_data_parser;

    import msg_pkg::*;
    import rx_burst_pkg::*;

    logic [127:0] msg_in;
    logic         data_frame_ok, data_error;
    logic [BURST_PIX_PER_MSG-1:0][BURST_PIX_W-1:0] pixels;

    rx_burst_data_parser dut (
        .msg_in        (msg_in),
        .data_frame_ok (data_frame_ok),
        .data_error    (data_error),
        .pixels        (pixels)
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
            $display("        msg_in=%032h  frame_ok=%0b", msg_in, data_frame_ok);
            $display("        pixels = %06h %06h %06h %06h",
                     pixels[0], pixels[1], pixels[2], pixels[3]);
        end
    endtask

    task automatic chk_px(input int idx, input logic [23:0] exp,
                          input string what);
        checks++;
        if (pixels[idx] !== exp) begin
            errors++;
            $display("  FAIL (%s): %s -- pixels[%0d] = %06h, expected %06h",
                     phase, what, idx, pixels[idx], exp);
        end
    endtask

    task automatic chk_all(input logic [23:0] p0, input logic [23:0] p1,
                           input logic [23:0] p2, input logic [23:0] p3,
                           input string what);
        chk_px(0, p0, {what, " pixel0"});
        chk_px(1, p1, {what, " pixel1"});
        chk_px(2, p2, {what, " pixel2"});
        chk_px(3, p3, {what, " pixel3"});
    endtask

    // -----------------------------------------------------------------
    // Frame construction
    // -----------------------------------------------------------------
    task automatic set_byte(input int idx, input logic [7:0] v);
        msg_in[127 - idx*8 -: 8] = v;
    endtask

    // Build a well-formed data message from four pixels.
    task automatic build_data(input logic [23:0] p0, input logic [23:0] p1,
                              input logic [23:0] p2, input logic [23:0] p3);
        msg_in = '0;
        set_byte(0,  CHAR_OPEN_BRACE);   // '{'
        set_byte(1,  p0[23:16]);         // R0
        set_byte(2,  p0[15: 8]);         // G0
        set_byte(3,  p0[ 7: 0]);         // B0
        set_byte(4,  p1[23:16]);         // R1
        set_byte(5,  CHAR_COMMA);        // ','
        set_byte(6,  p1[15: 8]);         // G1
        set_byte(7,  p1[ 7: 0]);         // B1
        set_byte(8,  p2[23:16]);         // R2
        set_byte(9,  p2[15: 8]);         // G2
        set_byte(10, CHAR_COMMA);        // ','
        set_byte(11, p2[ 7: 0]);         // B2
        set_byte(12, p3[23:16]);         // R3
        set_byte(13, p3[15: 8]);         // G3
        set_byte(14, p3[ 7: 0]);         // B3
        set_byte(15, CHAR_CLOSE_BRACE);  // '}'
        #1ps;
    endtask

    // Well-formed frame with a single payload byte raised, everything else
    // zero. Used to isolate each byte position independently.
    task automatic build_isolated(input int payload_byte);
        msg_in = '0;
        set_byte(0,  CHAR_OPEN_BRACE);
        set_byte(5,  CHAR_COMMA);
        set_byte(10, CHAR_COMMA);
        set_byte(15, CHAR_CLOSE_BRACE);
        set_byte(payload_byte, 8'hFF);
        #1ps;
    endtask

    task automatic corrupt(input int idx, input logic [7:0] v);
        set_byte(idx, v);
        #1ps;
    endtask

    // -----------------------------------------------------------------
    initial begin
        $display("=================================================");
        $display(" M2 REGRESSION: rx_burst_data_parser");
        $display("=================================================");

        msg_in = '0;
        #1ps;

        // =============================================================
        banner("1 - well-formed message, all twelve bytes distinct");
        // Values chosen so the expected pixel is readable at a glance:
        // pixel N uses 0xN0,0xN1,0xN2. Any transposition between pixels or
        // between channels produces an obviously wrong value.
        // =============================================================
        build_data(24'h10_1112, 24'h20_2122, 24'h30_3132, 24'h40_4142);
        chk(data_frame_ok,  "data_frame_ok high");
        chk(!data_error,    "data_error low");
        chk_all(24'h10_1112, 24'h20_2122, 24'h30_3132, 24'h40_4142,
                "distinct values:");

        // =============================================================
        banner("2 - channel order within a pixel");
        // Only the R channel differs between pixels; G and B are held
        // constant. If R and B were swapped this collapses immediately.
        // =============================================================
        build_data(24'h01_0000, 24'h02_0000, 24'h03_0000, 24'h04_0000);
        chk_all(24'h01_0000, 24'h02_0000, 24'h03_0000, 24'h04_0000,
                "R channel only:");

        build_data(24'h00_0001, 24'h00_0002, 24'h00_0003, 24'h00_0004);
        chk_all(24'h00_0001, 24'h00_0002, 24'h00_0003, 24'h00_0004,
                "B channel only:");

        build_data(24'h00_0100, 24'h00_0200, 24'h00_0300, 24'h00_0400);
        chk_all(24'h00_0100, 24'h00_0200, 24'h00_0300, 24'h00_0400,
                "G channel only:");

        // =============================================================
        banner("3 - byte isolation: each payload byte independently");
        // The decisive test. One payload byte raised to 0xFF, everything
        // else zero, for all twelve positions. Exactly one channel of one
        // pixel must respond and the other three pixels must stay zero.
        //
        // Bytes 4/6/7 and 8/9/11 are the straddling cases -- pixel1 and
        // pixel2 cross the commas at bytes 5 and 10.
        // =============================================================
        begin : isolation
            // payload byte index -> expected {pixel, channel}
            //   byte : 1  2  3  4  6  7  8  9  11 12 13 14
            //   pix  : 0  0  0  1  1  1  2  2  2  3  3  3
            //   chan : R  G  B  R  G  B  R  G  B  R  G  B
            automatic int  b_idx  [12] = '{1,2,3, 4,6,7, 8,9,11, 12,13,14};
            automatic int  b_pix  [12] = '{0,0,0, 1,1,1, 2,2,2,  3,3,3};
            automatic int  b_chan [12] = '{0,1,2, 0,1,2, 0,1,2,  0,1,2};

            foreach (b_idx[i]) begin
                automatic logic [23:0] exp;
                exp = 24'h00_0000;
                case (b_chan[i])
                    0: exp = 24'hFF_0000;   // R
                    1: exp = 24'h00_FF00;   // G
                    2: exp = 24'h00_00FF;   // B
                endcase

                build_isolated(b_idx[i]);
                chk(data_frame_ok,
                    $sformatf("byte %0d isolated: frame still valid", b_idx[i]));

                for (int p = 0; p < BURST_PIX_PER_MSG; p++) begin
                    chk_px(p, (p == b_pix[i]) ? exp : 24'h00_0000,
                           $sformatf("byte %0d -> pixel%0d chan%0d",
                                     b_idx[i], b_pix[i], b_chan[i]));
                end
            end
        end : isolation

        // =============================================================
        banner("4 - payload may hold delimiter and opcode values");
        // Colour channels are raw binary, so every one of the 256 values is
        // legal in every payload position. A frame whose payload is made
        // entirely of '{', '}' and ',' must parse exactly like any other.
        // =============================================================
        build_data(24'h7B_7D2C, 24'h2C_7B7D, 24'h7D_2C7B, 24'h7B_7B7B);
        chk(data_frame_ok, "delimiter payload: frame valid");
        chk_all(24'h7B_7D2C, 24'h2C_7B7D, 24'h7D_2C7B, 24'h7B_7B7B,
                "delimiter payload:");

        build_data(24'h52_4356, 24'h50_5749, 24'h48_5243, 24'h56_5057);
        chk(data_frame_ok, "opcode payload: frame valid");
        chk_all(24'h52_4356, 24'h50_5749, 24'h48_5243, 24'h56_5057,
                "opcode payload (R,C,V,P,W,I,H):");

        // =============================================================
        banner("5 - extreme payload values");
        // =============================================================
        build_data(24'h00_0000, 24'h00_0000, 24'h00_0000, 24'h00_0000);
        chk(data_frame_ok, "all-zero payload: frame valid");
        chk_all(24'h00_0000, 24'h00_0000, 24'h00_0000, 24'h00_0000,
                "all-zero payload:");

        build_data(24'hFF_FFFF, 24'hFF_FFFF, 24'hFF_FFFF, 24'hFF_FFFF);
        chk(data_frame_ok, "all-ones payload: frame valid");
        chk_all(24'hFF_FFFF, 24'hFF_FFFF, 24'hFF_FFFF, 24'hFF_FFFF,
                "all-ones payload:");

        // The orange used throughout the Stage 2C hardware tests.
        build_data(24'hFF_A500, 24'hFF_A500, 24'hFF_A500, 24'hFF_A500);
        chk_all(24'hFF_A500, 24'hFF_A500, 24'hFF_A500, 24'hFF_A500,
                "orange x4:");

        // =============================================================
        banner("6 - malformed frames");
        // Each of the four fixed bytes corrupted in turn. Payload is left
        // intact, so extraction must still work -- framing and extraction
        // are independent, exactly as in the header parser.
        // =============================================================
        build_data(24'h10_1112, 24'h20_2122, 24'h30_3132, 24'h40_4142);
        corrupt(0, 8'h00);
        chk(!data_frame_ok, "byte 0 not '{' -> rejected");
        chk(data_error,     "data_error asserted");
        chk_all(24'h10_1112, 24'h20_2122, 24'h30_3132, 24'h40_4142,
                "extraction still correct when rejected:");

        build_data(24'h10_1112, 24'h20_2122, 24'h30_3132, 24'h40_4142);
        corrupt(5, 8'h00);
        chk(!data_frame_ok, "byte 5 not ',' -> rejected");

        build_data(24'h10_1112, 24'h20_2122, 24'h30_3132, 24'h40_4142);
        corrupt(10, 8'h00);
        chk(!data_frame_ok, "byte 10 not ',' -> rejected");

        build_data(24'h10_1112, 24'h20_2122, 24'h30_3132, 24'h40_4142);
        corrupt(15, 8'h00);
        chk(!data_frame_ok, "byte 15 not '}' -> rejected");

        // A well-formed header must NOT pass as a data message: its byte 10
        // is a comma but byte 5 is too, so it depends on byte 15 and byte 0.
        // Build the 256x256 header and confirm the outcome is deliberate.
        msg_in = '0;
        set_byte(0, CHAR_OPEN_BRACE); set_byte(1, CHAR_I);
        set_byte(5, CHAR_COMMA);      set_byte(6, CHAR_H);
        set_byte(7, 8'h00); set_byte(8, 8'h01); set_byte(9, 8'h00);
        set_byte(10, CHAR_COMMA);     set_byte(11, CHAR_W);
        set_byte(12, 8'h00); set_byte(13, 8'h01); set_byte(14, 8'h00);
        set_byte(15, CHAR_CLOSE_BRACE);
        #1ps;
        // Both message types share the same four fixed positions, so a
        // header is structurally a valid data frame. That is expected and
        // harmless: rx_msg_decode decides which parser is authoritative via
        // bypass_active, and this module is only consulted for frames
        // already classified MSG_BURST_DATA.
        chk(data_frame_ok, "header shares the data framing (documented)");

        // =============================================================
        banner("7 - only bytes 0/5/10/15 are inspected");
        // Sweep every payload position through the frame delimiters while
        // keeping the four fixed bytes correct. The verdict must never
        // change -- if someone later adds a payload check, this catches it.
        // =============================================================
        begin : payload_sweep
            automatic int payload [12] = '{1,2,3,4,6,7,8,9,11,12,13,14};
            automatic logic [7:0] hostile [3] =
                '{CHAR_OPEN_BRACE, CHAR_CLOSE_BRACE, CHAR_COMMA};

            foreach (hostile[h]) begin
                msg_in = '0;
                set_byte(0, CHAR_OPEN_BRACE);
                set_byte(5, CHAR_COMMA);
                set_byte(10, CHAR_COMMA);
                set_byte(15, CHAR_CLOSE_BRACE);
                foreach (payload[i]) set_byte(payload[i], hostile[h]);
                #1ps;
                chk(data_frame_ok,
                    $sformatf("all payload = %02h still valid", hostile[h]));
            end
        end : payload_sweep

        // =============================================================
        $display("-------------------------------------------------");
        $display(" checks executed : %0d", checks);
        $display(" errors          : %0d", errors);
        $display(" RESULT: %s", (errors == 0) ? "PASS" : "FAIL");
        $display("=================================================");
        if (errors != 0) $fatal(1, "tb_rx_burst_data_parser FAILED");
        $finish;
    end

    // Guard: a hang must not look like a pass.
    initial begin
        #1ms;
        $display(" RESULT: FAIL -- testbench timeout");
        $fatal(1, "tb_rx_burst_data_parser timed out");
    end

endmodule : tb_rx_burst_data_parser
