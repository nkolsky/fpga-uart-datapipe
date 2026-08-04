// rx_burst_ctrl_pkg.sv
// --------------------
// State encoding for the RX burst-write controller FSM.
//
// Named to match its module, exactly as rx_mac_pkg is to rx_mac and
// rx_phy_pkg is to rx_phy. FSM encodings live in packages throughout this
// design; keeping this one in a package rather than as a module-local typedef
// keeps that convention intact.
//
// -----------------------------------------------------------------------
// WHY THIS IS A NEW PACKAGE RATHER THAN rx_burst_pkg
// -----------------------------------------------------------------------
// burst_state_t originally lived in rx_burst_pkg, alongside BURST_DIM_W,
// BURST_PIX_W, BURST_ADDR_W and BURST_PIX_PER_MSG. Those constants are
// message-format facts and have moved to msg_format_pkg, which rx_burst_ctrl
// now wildcard-imports.
//
// Re-importing rx_burst_pkg just to reach the state type would pull the OLD
// copies of those same constants back into the same scope, giving two
// definitions of BURST_DIM_W and friends resolved by compile order. This
// design already carries three collisions of exactly that kind -- ADDR_WIDTH
// in fifo_pkg and rgf_pkg, mac_state_t in tx_pkg and tx_mac_pkg, state_t in
// rom_sequencer_pkg and tx_seq_pkg -- and there is no reason to add a fourth.
//
// A package holding only the FSM state cannot collide with anything.
//
// -----------------------------------------------------------------------
// WHY THE STATES ARE PREFIXED
// -----------------------------------------------------------------------
// The old names were B_IDLE / B_ACTIVE / B_EMIT / B_DONE. burst_rd_ctrl --
// the burst READ controller on the 100 MHz memory side -- declares its own
// FSM with a module-local typedef whose members include B_IDLE and B_DONE.
// The two never collided only because burst_rd_ctrl's were local.
//
// Putting these in a package removes that protection, so they are prefixed
// RXBURST_ to be unambiguous: this is the RECEIVE-side burst WRITE path, not
// the memory-side burst read path.

`timescale 1ns/1ps

package rx_burst_ctrl_pkg;

    // RXBURST_IDLE   : no burst in progress; waiting for a valid header.
    // RXBURST_ACTIVE : bypass on, burst armed, waiting for the next data
    //                  frame. Pixel data may arrive at any time.
    // RXBURST_EMIT   : unpacking one data frame into individual
    //                  {address, pixel} commands, one per clock.
    // RXBURST_DONE   : final command accepted; drop bypass and report.
    typedef enum logic [2:0] {
        RXBURST_IDLE   = 3'd0,
        RXBURST_ACTIVE = 3'd1,
        RXBURST_EMIT   = 3'd2,
        RXBURST_DONE   = 3'd3
    } rx_burst_state_t;

endpackage : rx_burst_ctrl_pkg
