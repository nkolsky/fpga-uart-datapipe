
`timescale 1ps/1ps
package uart_pkg;

    // ----------------------------------------------------------------
    // Clock and baud rate
    // ----------------------------------------------------------------
    localparam int CLK_FREQ_HZ  = 130_000_000;  // 130 MHz system clock
    localparam int BAUD_RATE    = 8_125_000;     // 8 Mbps

    // ----------------------------------------------------------------
    // Baud rate divisors
    // TX: one tick per bit period
    //     100 MHz / 1 MHz = 100
    // RX: 16 ticks per bit period (oversampling for start/stop validation)
    //     100 MHz / (1 MHz * 16) = 6.25 -> round to 6
    // ----------------------------------------------------------------
    localparam int DIV_TX = CLK_FREQ_HZ / BAUD_RATE;           // 100
    localparam int DIV_RX = CLK_FREQ_HZ / (BAUD_RATE * 16);    // 6

    // ----------------------------------------------------------------
    // Message format
    // ----------------------------------------------------------------
    localparam int MSG_BYTES = 16;  // full UART message length in bytes

endpackage : uart_pkg