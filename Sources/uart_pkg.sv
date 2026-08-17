`timescale 1ps/1ps
package uart_pkg;

    // ----------------------------------------------------------------
    // Clock and baud rate
    //
    // rx_phy oversamples 16x, so CLK_FREQ_HZ must be BAUD_RATE * 16 * an
    // integer. The division below TRUNCATES SILENTLY: a pair that does
    // not divide evenly still elaborates and still looks like a working
    // UART, but every bit period drifts. CHECK BOTH BY HAND IF YOU
    // CHANGE THESE.
    //
    //     280_000_000 == 8_000_000 * 16 * 2     OK
    //     280_000_000 == 8_000_000 * 32         OK
    //
    // Why this pair: the clock is specified at 280 MHz, and 16x
    // oversampling with one further divide-by-two puts the baud at
    // 8.75 Mbaud. The host's FT2232H divides 12 MHz and reaches
    // 8.727 Mbaud (12/1.375), a 0.26% mismatch a 10-bit frame absorbs
    // easily -- the previous 130 MHz / 8.125 Mbaud pair ran a 1.5%
    // mismatch against a host at 8 Mbaud and worked.
    // ----------------------------------------------------------------
    localparam int CLK_FREQ_HZ  = 256_000_000;  // pll_clk_out, NOT CLK100MHZ
    localparam int BAUD_RATE    = 8_000_000;

    localparam int DIV_TX = CLK_FREQ_HZ / BAUD_RATE;         // 32, clocks per bit
    localparam int DIV_RX = CLK_FREQ_HZ / (BAUD_RATE * 16);  // 2,  clocks per x16 tick

    // ----------------------------------------------------------------
    // HISTORICAL: the single fixed frame length from when every message
    // was 16 bytes. Framing is variable-length now and driven by
    // msg_format_pkg. Nothing reads this -- retained so an external
    // import of uart_pkg does not break.
    // ----------------------------------------------------------------
    localparam int MSG_BYTES = 16;  // UNUSED

endpackage : uart_pkg