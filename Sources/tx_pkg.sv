`timescale 1ns / 1ps

package tx_pkg;

    typedef enum logic [2:0] {
        MAC_IDLE      = 3'd0,
        MAC_LOAD      = 3'd1,
        MAC_TRIG_BYTE = 3'd2,
        MAC_WAIT_BUSY = 3'd3,
        MAC_WAIT_BYTE = 3'd4,
        MAC_CHK_DONE  = 3'd5,
        MAC_DONE      = 3'd6
    } mac_state_t;

    typedef enum logic [2:0] {
        PHY_IDLE       = 3'b000,
        PHY_START_BIT  = 3'b001,
        PHY_DATA_BITS  = 3'b010,
        PHY_PARITY_BIT = 3'b011,
        PHY_STOP_BIT   = 3'b100
    } phy_state_t;

endpackage