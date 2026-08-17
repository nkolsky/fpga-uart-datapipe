`timescale 1ps/1ps
package uart_pkg;

    // ----------------------------------------------------------------
    // UART timing used by tx_phy and rx_phy.
    //
    // tx_phy sends one bit every DIV_TX system clocks. rx_phy oversamples
    // the line at 16x and generates one sample tick every DIV_RX system
    // clocks. The current implementation targets a 256 MHz system clock and
    // 8 MHz UART baud.
    //
    // BOTH DIVISIONS MUST COME OUT EXACT. They truncate silently: a pair
    // that does not divide evenly still elaborates and still looks like a
    // working UART, but every bit period drifts against the host. Check by
    // hand when changing either value.
    //
    //     256_000_000 / 8_000_000        = 32   exact
    //     256_000_000 / (8_000_000 * 16) =  2   exact
    //
    // DRIFT. The FPGA side is exact by the two divisions above. The host's
    // FT2232H divides 12 MHz by 1.5 and also reaches 8 Mbaud exactly, so the
    // link runs at 0% mismatch -- a frame absorbs a few percent, so there is
    // no drift budget being spent here at all.
    // ----------------------------------------------------------------
    localparam int CLK_FREQ_HZ = 256_000_000; // pll_clk_out, not CLK100MHZ
    localparam int BAUD_RATE   = 8_000_000;

    localparam int DIV_TX = CLK_FREQ_HZ / BAUD_RATE;         // 32 clocks per bit
    localparam int DIV_RX = CLK_FREQ_HZ / (BAUD_RATE * 16);  // 2 clocks per x16 tick

    // ----------------------------------------------------------------
    // THE CLOCKING WIZARD MUST AGREE WITH CLK_FREQ_HZ.
    //
    // clk_wiz_0 is a Vivado IP: its configuration lives in the project, not
    // in this repository, so nothing here can check it and no elaboration
    // error will be raised if it disagrees. If the IP still produces the
    // earlier 130 MHz output, DIV_TX is 32 by declaration while the link runs
    // at 4.06 Mbaud in fact -- below the 5 MHz floor the spec sets, and
    // mismatched against a host at 8 Mbaud.
    //
    // clocking_subsystem's ports are still named clk_130_out / rst_130_n from
    // that earlier configuration. The names are stale; this constant is not.
    // ----------------------------------------------------------------

    // ----------------------------------------------------------------
    // FRAME: start + 8 data + even parity + stop = 11 bit times.
    //
    // Every cycle count elsewhere in the design derives from this and DIV_TX:
    //
    //     byte             11 x 32 =  352 clocks    1.375 us
    //     16-byte message  16 x 352 = 5632 clocks     22 us
    //
    // rx_burst_ctrl states the 5632 figure directly; the CDC and flow-control
    // comments size their margins against it.
    // ----------------------------------------------------------------

    // ----------------------------------------------------------------
    // Compatibility constant only. Frame length is defined by msg_format_pkg;
    // this package does not enforce a fixed UART message size. Nothing in the
    // design reads it -- retained so an external import does not break.
    // ----------------------------------------------------------------
    localparam int MSG_BYTES = 16; // compatibility only

endpackage : uart_pkg
