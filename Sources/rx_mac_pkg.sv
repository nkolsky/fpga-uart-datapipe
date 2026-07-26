// rx_mac_pkg.sv
// -------------
// State encoding for the RX MAC FSM.
//
// State flow:
//   MAC_IDLE     : wait for byte_valid pulse from RX PHY
//   MAC_STORE    : write received byte into msg_buf at current byte_idx;
//                  increment byte_idx
//   MAC_CHK_DONE : check if all 16 bytes received;
//                  if byte_idx == 15 → MAC_DONE, else → MAC_IDLE
//   MAC_DONE     : assert msg_valid for one cycle, output msg_data;
//                  return to MAC_IDLE

`timescale 1ns/1ps

package rx_mac_pkg;
    typedef enum logic [1:0] {
        MAC_IDLE     = 2'b00,
        MAC_STORE    = 2'b01,
        MAC_CHK_DONE = 2'b10,
        MAC_DONE     = 2'b11
    } rx_mac_state_t;
endpackage : rx_mac_pkg
