// tb_burst_msg_composer.sv
// ------------------------
// Round-trips burst_msg_composer through rx_burst_data_parser.
//
// The packed 4-pixel format straddles its comma separators -- R1 ends group
// one while G1 and B1 begin group two -- so writing the composer from the
// format string is easy to get subtly wrong in a way that inspection does
// not catch.
//
// rx_burst_data_parser already unpacks this exact format on the Image Burst
// WRITE path and is hardware-verified. Composing four pixels, byte-reversing
// into receive order, and requiring the parser to return the SAME four
// pixels therefore checks this module against working silicon rather than
// against a reading of the specification.
//
// The byte reverse is the only bridge needed: the composer emits TRANSMIT
// order (byte 0 at msg[7:0], what tx_mac consumes) while the parser expects
// RECEIVE order (byte 0 at msg_in[127:120]).

`timescale 1ns/1ps

module tb_burst_msg_composer;

    import msg_pkg::*;
    import rx_burst_pkg::*;

    logic [BURST_PIX_PER_MSG-1:0][BURST_PIX_W-1:0] pixels_in;
    logic [127:0] msg_tx;

    burst_msg_composer dut (.pixels(pixels_in), .msg(msg_tx));

    logic [127:0] msg_rx;
    logic         data_frame_ok, data_error;
    logic [BURST_PIX_PER_MSG-1:0][BURST_PIX_W-1:0] pixels_out;

    rx_burst_data_parser ref_parser (
        .msg_in        (msg_rx),
        .data_frame_ok (data_frame_ok),
        .data_error    (data_error),
        .pixels        (pixels_out)
    );

    // Transmit order -> receive order.
    function automatic logic [127:0] byte_reverse(input logic [127:0] v);
        logic [127:0] r;
        for (int i = 0; i < 16; i++) r[8*i +: 8] = v[8*(15-i) +: 8];
        return r;
    endfunction

    assign msg_rx = byte_reverse(msg_tx);

    int checks = 0, errors = 0;
    string phase = "";
    task automatic banner(input string n); phase = n; $display("--- %s", n); endtask
    task automatic chk(input bit c, input string w);
        checks++;
        if (!c) begin errors++; $display("  ERROR [%s] %s", phase, w); end
    endtask

    task automatic round_trip(input logic [23:0] p0, input logic [23:0] p1,
                              input logic [23:0] p2, input logic [23:0] p3,
                              input string tag);
        pixels_in[0] = p0; pixels_in[1] = p1;
        pixels_in[2] = p2; pixels_in[3] = p3;
        #1;
        chk(data_frame_ok === 1'b1, {tag, ": parser rejected the framing"});
        chk(data_error    === 1'b0, {tag, ": parser flagged a data error"});
        chk(pixels_out[0] === p0, $sformatf("%s: p0 %06h != %06h", tag, pixels_out[0], p0));
        chk(pixels_out[1] === p1, $sformatf("%s: p1 %06h != %06h", tag, pixels_out[1], p1));
        chk(pixels_out[2] === p2, $sformatf("%s: p2 %06h != %06h", tag, pixels_out[2], p2));
        chk(pixels_out[3] === p3, $sformatf("%s: p3 %06h != %06h", tag, pixels_out[3], p3));
    endtask

    initial begin
        $display("=================================================");
        $display(" UNIT: burst_msg_composer (round-trip)");
        $display("=================================================");

        banner("1 - fixed bytes are in the right places");
        pixels_in = '0; #1;
        // Transmit order: byte 0 at [7:0].
        chk(msg_tx[  7:  0] === CHAR_OPEN_BRACE,  "byte 0 is not '{'");
        chk(msg_tx[ 47: 40] === CHAR_COMMA,       "byte 5 is not ','");
        chk(msg_tx[ 87: 80] === CHAR_COMMA,       "byte 10 is not ','");
        chk(msg_tx[127:120] === CHAR_CLOSE_BRACE, "byte 15 is not '}'");

        banner("2 - round-trip through rx_burst_data_parser");
        round_trip(24'h000000, 24'h000000, 24'h000000, 24'h000000, "all zero");
        round_trip(24'hFFFFFF, 24'hFFFFFF, 24'hFFFFFF, 24'hFFFFFF, "all ones");
        // Distinct per channel per slot: catches any lane or group swap.
        round_trip(24'h010203, 24'h040506, 24'h070809, 24'h0A0B0C, "sequential");
        round_trip(24'hAABBCC, 24'hDDEEFF, 24'h112233, 24'h445566, "distinct");
        // One slot at a time, so a misrouted slot cannot hide behind another.
        round_trip(24'hFF0000, 24'h000000, 24'h000000, 24'h000000, "p0 only");
        round_trip(24'h000000, 24'h00FF00, 24'h000000, 24'h000000, "p1 only");
        round_trip(24'h000000, 24'h000000, 24'h0000FF, 24'h000000, "p2 only");
        round_trip(24'h000000, 24'h000000, 24'h000000, 24'hFFFFFF, "p3 only");
        // One channel at a time within a slot, catching an R/G/B rotation
        // that a whole-pixel test would miss.
        round_trip(24'hFF0000, 24'h00FF00, 24'h0000FF, 24'hFF00FF, "channel walk");

        banner("3 - byte values that collide with the framing characters");
        // Payload bytes equal to '{', '}' and ',' must not disturb framing.
        round_trip(24'h7B7D2C, 24'h2C7B7D, 24'h7D2C7B, 24'h7B7B7B, "brace/comma payload");

        banner("4 - exhaustive-ish randomised sweep");
        begin
            logic [23:0] a, b, c, d;
            for (int i = 0; i < 512; i++) begin
                a = 24'($urandom); b = 24'($urandom);
                c = 24'($urandom); d = 24'($urandom);
                round_trip(a, b, c, d, $sformatf("rand%0d", i));
            end
        end

        $display("-------------------------------------------------");
        $display(" checks executed : %0d", checks);
        $display(" errors          : %0d", errors);
        $display(" RESULT: %s", (errors == 0) ? "PASS" : "FAIL");
        $display("=================================================");
        $finish;
    end

endmodule : tb_burst_msg_composer
