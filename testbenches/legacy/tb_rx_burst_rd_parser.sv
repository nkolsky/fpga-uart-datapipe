// tb_rx_burst_rd_parser.sv
// ------------------------
// Unit test for the Image Burst Read request parser.
//
// The parser is purely combinational, so this drives msg_in and checks the
// outputs after a settle delay. There is no clock.
//
// Two families of case matter most here:
//
//   ALIASING. Each of A, H and W is a 24-bit field. If any of them were
//   narrowed before being range-checked, high-order rubbish would alias
//   into a legal low value -- A2 = 0xFF with a low byte of 5 read as
//   address 5. Section 7 fails loudly if that ever returns.
//
//   EXTENT. rx_burst_ctrl on the write side never needed a base+extent
//   check because its base is hardcoded to zero. Burst Read takes the base
//   from the host, so a region near the right edge can walk off its row
//   and wrap into the next one -- returning pixels the host never asked
//   for while still reporting the requested H and W. Section 9 covers
//   both axes, and the horizontal case is the dangerous one.

`timescale 1ns/1ps

module tb_rx_burst_rd_parser;

    import msg_pkg::*;
    import memory_pkg::*;

    logic [127:0] msg_in;

    logic        br_frame_ok, br_dims_ok, br_addr_ok, br_extent_ok;
    logic        br_valid, br_err;
    logic [23:0] br_addr_raw, br_h_raw, br_w_raw;
    logic [9:0]  br_base_row, br_base_col, br_height, br_width;

    rx_burst_rd_parser dut (
        .msg_in       (msg_in),
        .br_frame_ok  (br_frame_ok),  .br_dims_ok   (br_dims_ok),
        .br_addr_ok   (br_addr_ok),   .br_extent_ok (br_extent_ok),
        .br_valid     (br_valid),     .br_err       (br_err),
        .br_addr_raw  (br_addr_raw),  .br_h_raw     (br_h_raw),
        .br_w_raw     (br_w_raw),
        .br_base_row  (br_base_row),  .br_base_col  (br_base_col),
        .br_height    (br_height),    .br_width     (br_width)
    );

    int checks = 0, errors = 0;
    string phase = "";
    task automatic banner(input string n); phase = n; $display("--- %s", n); endtask
    task automatic chk(input bit c, input string w);
        checks++;
        if (!c) begin errors++; $display("  ERROR [%s] %s", phase, w); end
    endtask

    // -----------------------------------------------------------------
    // Frame construction. A, H and W are written at FULL 24-bit width so
    // a test can place anything it likes in the high bytes.
    // Byte 0 sits at msg_in[127:120] -- the receive convention.
    // -----------------------------------------------------------------
    function automatic logic [127:0] build(input logic [23:0] a,
                                           input logic [23:0] h,
                                           input logic [23:0] w);
        logic [7:0]   b [0:15];
        logic [127:0] m;
        b[0]  = CHAR_OPEN_BRACE; b[1]  = CHAR_R;
        b[2]  = a[23:16]; b[3]  = a[15:8]; b[4]  = a[7:0];
        b[5]  = CHAR_COMMA;      b[6]  = CHAR_H;
        b[7]  = h[23:16]; b[8]  = h[15:8]; b[9]  = h[7:0];
        b[10] = CHAR_COMMA;      b[11] = CHAR_W;
        b[12] = w[23:16]; b[13] = w[15:8]; b[14] = w[7:0];
        b[15] = CHAR_CLOSE_BRACE;
        for (int i = 0; i < 16; i++) m[127 - 8*i -: 8] = b[i];
        return m;
    endfunction

    task automatic drive(input logic [127:0] m); msg_in = m; #1; endtask

    // Expect ACCEPT, and check the decomposed geometry.
    task automatic ok(input int a, input int h, input int w, input string tag);
        drive(build(24'(a), 24'(h), 24'(w)));
        chk(br_frame_ok  === 1'b1, {tag, ": frame_ok low"});
        chk(br_dims_ok   === 1'b1, {tag, ": dims_ok low"});
        chk(br_addr_ok   === 1'b1, {tag, ": addr_ok low"});
        chk(br_extent_ok === 1'b1, {tag, ": extent_ok low"});
        chk(br_valid     === 1'b1, {tag, ": valid low"});
        chk(br_err       === 1'b0, {tag, ": err high"});
        chk(br_base_row === 10'(a / IMG_WIDTH),
            $sformatf("%s: base_row %0d, expected %0d", tag, br_base_row, a / IMG_WIDTH));
        chk(br_base_col === 10'(a % IMG_WIDTH),
            $sformatf("%s: base_col %0d, expected %0d", tag, br_base_col, a % IMG_WIDTH));
        chk(br_height === 10'(h), $sformatf("%s: height %0d, expected %0d", tag, br_height, h));
        chk(br_width  === 10'(w), $sformatf("%s: width %0d, expected %0d",  tag, br_width,  w));
    endtask

    // Expect REJECT on geometry with framing intact.
    task automatic reject(input logic [23:0] a, input logic [23:0] h,
                          input logic [23:0] w, input string tag);
        drive(build(a, h, w));
        chk(br_frame_ok === 1'b1, {tag, ": framing should still be ok"});
        chk(br_valid    === 1'b0, {tag, ": should be rejected"});
        chk(br_err      === 1'b1, {tag, ": err should be high"});
        // A rejected request must not present usable geometry.
        chk(br_base_row === 10'd0, {tag, ": base_row not zeroed"});
        chk(br_base_col === 10'd0, {tag, ": base_col not zeroed"});
        chk(br_height   === 10'd0, {tag, ": height not zeroed"});
        chk(br_width    === 10'd0, {tag, ": width not zeroed"});
    endtask

    initial begin
        $display("=================================================");
        $display(" UNIT: rx_burst_rd_parser");
        $display("=================================================");

        banner("1 - valid small regions at the origin");
        ok(0, 1, 1, "1x1@0");
        ok(0, 1, 2, "1x2@0");
        ok(0, 2, 3, "2x3@0");
        ok(0, 3, 3, "3x3@0");
        ok(0, 4, 4, "4x4@0");
        ok(0, IMG_HEIGHT, IMG_WIDTH, "full frame");

        banner("2 - nonzero and non-word-aligned start address");
        ok(1,    1, 1, "A=1  (lane 1)");
        ok(2,    2, 2, "A=2  (lane 2)");
        ok(3,    1, 1, "A=3  (lane 3)");
        ok(1026, 2, 3, "A=1026 -> (4,2)");
        ok(257,  2, 2, "A=257 -> (1,1)");
        ok(255,  1, 1, "A=255 -> (0,255) right edge");

        banner("3 - final valid start address");
        ok(IMG_HEIGHT*IMG_WIDTH - 1, 1, 1, "last pixel, 1x1");
        ok((IMG_HEIGHT-1)*IMG_WIDTH, 1, IMG_WIDTH, "last row, full width");
        ok(IMG_WIDTH - 1, IMG_HEIGHT, 1, "last column, full height");

        banner("4 - zero height");
        reject(24'd0,   24'd0, 24'd4, "H=0");
        reject(24'd100, 24'd0, 24'd1, "H=0 with nonzero A");

        banner("5 - zero width");
        reject(24'd0,   24'd4, 24'd0, "W=0");
        reject(24'd100, 24'd1, 24'd0, "W=0 with nonzero A");
        reject(24'd0,   24'd0, 24'd0, "both zero");

        banner("6 - dimensions beyond the image");
        reject(24'd0, 24'(IMG_HEIGHT + 1), 24'd1, "H = IMG_HEIGHT+1");
        reject(24'd0, 24'd1, 24'(IMG_WIDTH + 1),  "W = IMG_WIDTH+1");
        reject(24'd0, 24'h00_0200, 24'd1, "H = 512");
        reject(24'd0, 24'd1, 24'h00_0200, "W = 512");

        banner("7 - start address outside the framebuffer");
        reject(24'(IMG_HEIGHT*IMG_WIDTH), 24'd1, 24'd1, "A = 65536");
        reject(24'h01_0000, 24'd1, 24'd1, "A = 0x010000");
        reject(24'hFF_FFFF, 24'd1, 24'd1, "A all ones");

        banner("8 - full 24-bit aliasing on A, H and W");
        // Low bits legal, high bytes rubbish. Each of these would be
        // ACCEPTED by a parser that narrowed before range-checking.
        reject(24'hFF_0005, 24'd1, 24'd1, "A 0xFF0005 aliases to 5");
        reject(24'h01_0000, 24'd1, 24'd1, "A 0x010000 aliases to 0");
        reject(24'h01_00FF, 24'd1, 24'd1, "A 0x0100FF aliases to 255");
        reject(24'd0, 24'hFF_0002, 24'd2, "H 0xFF0002 aliases to 2");
        reject(24'd0, 24'h01_0001, 24'd1, "H 0x010001 aliases to 1");
        reject(24'd0, 24'd2, 24'hFF_0002, "W 0xFF0002 aliases to 2");
        reject(24'd0, 24'd1, 24'h01_0001, "W 0x010001 aliases to 1");
        // H = 0x000100 is exactly 256, which IS legal -- the boundary
        // between aliasing and a genuine maximum.
        ok(0, 256, 256, "H=W=0x000100 is the legal maximum");

        banner("9 - extent overflow");
        // The extent rule is base + extent <= limit, so a full-span region
        // starting at zero EXACTLY fits and must be accepted. Only a
        // nonzero base pushes it over. The two axes are checked
        // symmetrically below: the accepted maximum first, then the same
        // dimension shifted by one, which must be rejected.

        // ---- vertical ----
        ok(0, IMG_HEIGHT, 1, "row 0, H=256 exactly fits");
        reject(24'(IMG_WIDTH), 24'(IMG_HEIGHT), 24'd1, "row 1, H=256 overflows");
        reject(24'((IMG_HEIGHT-1)*IMG_WIDTH), 24'd2, 24'd1, "last row, H=2 overflows");
        reject(24'((IMG_HEIGHT-2)*IMG_WIDTH), 24'd3, 24'd1, "row 254, H=3 overflows");
        // ...and the vertical boundaries that must still be accepted.
        ok((IMG_HEIGHT-1)*IMG_WIDTH, 1, 1, "last row, H=1 exactly fits");
        ok((IMG_HEIGHT-2)*IMG_WIDTH, 2, 1, "row 254, H=2 exactly fits");

        // ---- horizontal ----
        // The row-wrap case. Without the column check these would be
        // accepted and would silently return the wrong pixels.
        ok(0, 1, IMG_WIDTH, "col 0, W=256 exactly fits");
        reject(24'd1, 24'd1, 24'(IMG_WIDTH), "col 1, W=256 wraps");
        reject(24'(IMG_WIDTH-1), 24'd1, 24'd2, "col 255, W=2 wraps");
        reject(24'(IMG_WIDTH-2), 24'd1, 24'd3, "col 254, W=3 wraps");
        reject(24'd250, 24'd1, 24'd10, "col 250, W=10 wraps");
        // ...and the exact boundaries that must still be accepted.
        ok(IMG_WIDTH-2, 1, 2, "col 254, W=2 exactly fits");
        ok(IMG_WIDTH-1, 1, 1, "col 255, W=1 exactly fits");
        ok(250, 1, 6, "col 250, W=6 exactly fits");
        // A multi-row region that fits horizontally at every row.
        ok(254 + 10*IMG_WIDTH, 3, 2, "3x2 at (10,254) fits");
        reject(24'(254 + 10*IMG_WIDTH), 24'd3, 24'd3, "3x3 at (10,254) wraps");

        banner("10 - every fixed framing byte");
        begin : framing_sweep
            logic [127:0] good = build(24'd0, 24'd2, 24'd2);
            logic [127:0] bad;
            int    pos [0:6];
            string nm  [0:6];
            pos[0]=0;  nm[0]="byte 0 '{'";
            pos[1]=1;  nm[1]="byte 1 'R'";
            pos[2]=5;  nm[2]="byte 5 ','";
            pos[3]=6;  nm[3]="byte 6 'H'";
            pos[4]=10; nm[4]="byte 10 ','";
            pos[5]=11; nm[5]="byte 11 'W'";
            pos[6]=15; nm[6]="byte 15 '}'";
            for (int i = 0; i < 7; i++) begin
                bad = good;
                bad[127 - 8*pos[i] -: 8] = 8'h00;
                drive(bad);
                chk(br_frame_ok === 1'b0, {nm[i], ": frame_ok should be low"});
                chk(br_valid    === 1'b0, {nm[i], ": valid should be low"});
                // br_err means "framed but unusable"; a framing failure is
                // reported through !br_valid instead, so err stays low.
                chk(br_err      === 1'b0, {nm[i], ": err should be low"});
                chk(br_height   === 10'd0, {nm[i], ": height not zeroed"});
                chk(br_width    === 10'd0, {nm[i], ": width not zeroed"});
            end
        end

        banner("11 - Burst Write header vs Burst Read request");
        begin : hdr_discrimination
            logic [127:0] m;
            // The Burst WRITE header is byte-identical except byte 1 = 'I'.
            // It must NOT parse as a burst read.
            m = build(24'd0, 24'd2, 24'd2);
            m[119:112] = CHAR_I;
            drive(m);
            chk(br_frame_ok === 1'b0, "Burst Write header parsed as a burst read");
            chk(br_valid    === 1'b0, "Burst Write header reported valid");

            // A Single Pixel Read is also 16 bytes and also starts {R -- it
            // differs at byte 6 ('C' not 'H') and byte 11 ('P' not 'W').
            m = build(24'd0, 24'd2, 24'd2);
            m[79:72] = CHAR_C;
            drive(m);
            chk(br_frame_ok === 1'b0, "byte 6 'C' accepted as a burst read");

            m = build(24'd0, 24'd2, 24'd2);
            m[39:32] = CHAR_P;
            drive(m);
            chk(br_frame_ok === 1'b0, "byte 11 'P' accepted as a burst read");

            // Legacy {Rnnn,Cnnn,Vnnn}: bytes 6 and 11 both differ.
            m = build(24'd0, 24'd2, 24'd2);
            m[79:72] = CHAR_C; m[39:32] = CHAR_V;
            drive(m);
            chk(br_frame_ok === 1'b0, "legacy RGF frame accepted as a burst read");
        end

        banner("12 - reset / quiescent bus");
        // Combinational module: "reset" is the bus value the design presents
        // when nothing has been received.
        drive(128'd0);
        chk(br_frame_ok === 1'b0, "all-zero bus reported framing ok");
        chk(br_valid    === 1'b0, "all-zero bus reported valid");
        chk(br_err      === 1'b0, "all-zero bus reported a geometry error");
        chk(br_height   === 10'd0, "all-zero bus produced a nonzero height");
        chk(br_width    === 10'd0, "all-zero bus produced a nonzero width");

        drive({128{1'b1}});
        chk(br_frame_ok === 1'b0, "all-ones bus reported framing ok");
        chk(br_valid    === 1'b0, "all-ones bus reported valid");

        // Stateless: a known-good frame after all that gives the same answer.
        ok(0, 2, 3, "re-drive 2x3 after the degenerate buses");

        $display("-------------------------------------------------");
        $display(" checks executed : %0d", checks);
        $display(" errors          : %0d", errors);
        $display(" RESULT: %s", (errors == 0) ? "PASS" : "FAIL");
        $display("=================================================");
        $finish;
    end

endmodule : tb_rx_burst_rd_parser
