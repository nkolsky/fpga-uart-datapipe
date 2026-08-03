
`timescale 1ps/1ps
package uart_pkg;

    // ----------------------------------------------------------------
    // Clock and baud rate
    // ----------------------------------------------------------------
    localparam int CLK_FREQ_HZ  = 130_000_000;  // 130 MHz UART domain clock
                                                // (pll_clk_out, NOT CLK100MHZ)
    localparam int BAUD_RATE    = 8_125_000;    // 8.125 Mbaud

    // ----------------------------------------------------------------
    // Baud rate divisors
    //
    // The PLL output was chosen so that CLK_FREQ_HZ is EXACTLY
    // BAUD_RATE * 16 (130 MHz = 8.125 MHz * 16). Both divisors are
    // therefore integers with no rounding error at all:
    //
    // TX: one tick per bit period
    //     130 MHz / 8.125 MHz = 16
    // RX: 16 ticks per bit period (oversampling for start/stop
    //     validation)
    //     130 MHz / (8.125 MHz * 16) = 1
    //
    // DIV_RX = 1 is intentional and not a degenerate case: the 130 MHz
    // clock IS the baud x16 tick, so rx_phy's baud_cnt reloads every
    // cycle and its 4-bit tick_q counter advances once per clock,
    // giving 16 clocks (123.08 ns) per bit period. See chip_top.sv's
    // PLL status block for why the MMCM was configured this way.
    // ----------------------------------------------------------------
    localparam int DIV_TX = CLK_FREQ_HZ / BAUD_RATE;           // 16
    localparam int DIV_RX = CLK_FREQ_HZ / (BAUD_RATE * 16);    // 1

    // ----------------------------------------------------------------
    // Message format
    //
    // HISTORICAL: this was the single fixed frame length back when every
    // message was 16 bytes. Framing is now variable-length and is driven
    // entirely by rx_msg_pkg::MSG_BYTES_MAX / MSG_BYTES_REG_READ /
    // MSG_BYTES_PIX_WRITE, decoded per frame by rx_msg_decode. Nothing
    // in the design reads MSG_BYTES any more -- it is retained only so
    // that an external import of uart_pkg does not break.
    // ----------------------------------------------------------------
    localparam int MSG_BYTES = 16;  // UNUSED - see note above

endpackage : uart_pkg