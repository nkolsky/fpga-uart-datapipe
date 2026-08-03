// rx_mac_pkg.sv
// -------------
// State encoding for the RX MAC FSM.
//
// The FSM is VARIABLE-LENGTH. It has no fixed frame size and no protocol
// knowledge of its own: rx_msg_decode derives expected_len combinationally
// from the live buffer, and this FSM simply counts up to it. Frames in the
// current message set are 6, 11 or 16 bytes.
//
// State flow:
//   MAC_IDLE     : wait for byte_valid pulse from RX PHY
//   MAC_STORE    : write received byte into msg_buf at the lane named by
//                  byte_idx; increment byte_idx
//   MAC_CHK_DONE : if byte_idx == expected_len → MAC_DONE, else → MAC_IDLE.
//                  byte_idx counts BYTES ALREADY STORED (the newest byte was
//                  committed on the edge entering this state), so the
//                  comparison is against the length itself, not length-1.
//                  This previously read "check if all 16 bytes received;
//                  if byte_idx == 15" -- wrong on both counts since the
//                  variable-length rework.
//   MAC_DONE     : assert msg_valid for one cycle; publish msg_data and
//                  latch msg_kind_q from msg_kind_prov; clear msg_buf and
//                  byte_idx; return to MAC_IDLE (or straight to MAC_STORE
//                  if the next byte is already arriving)
//
// SOFT RESET: par_val_rst (rx_phy's parity_err_pulse) forces the FSM back
// to MAC_IDLE and clears the buffer from any state, abandoning a frame
// corrupted by a failed-parity byte.

`timescale 1ns/1ps

package rx_mac_pkg;
    typedef enum logic [1:0] {
        MAC_IDLE     = 2'b00,
        MAC_STORE    = 2'b01,
        MAC_CHK_DONE = 2'b10,
        MAC_DONE     = 2'b11
    } rx_mac_state_t;
endpackage : rx_mac_pkg
