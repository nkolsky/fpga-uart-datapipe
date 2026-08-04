// rx_mac_pkg.sv
// -------------
// State encoding for the RX MAC FSM.
//
// The FSM is VARIABLE-LENGTH and now owns FRAMING outright. It no longer
// receives a length from the parser: it finds the end of the frame itself by
// inspecting the delimiter positions.
//
// The MAC checks bytes 0, 5, 10 and 15. The parser checks bytes 1, 6 and 11
// and extracts 2-4, 7-9 and 12-14. Neither looks at the other's bytes.
//
// State flow:
//   MAC_IDLE  : wait for byte_valid from the RX PHY. At a frame start only
//               '{' is accepted; any other byte is dropped where it stands.
//   MAC_STORE : write the byte into the lane named by byte_idx; increment.
//   MAC_CHK   : inspect the byte just stored. byte_idx counts bytes ALREADY
//               STORED, so byte_idx of 6, 11 or 16 means the newest byte sits
//               at index 5, 10 or 15 -- a delimiter position.
//                 not a delimiter position  -> MAC_IDLE, keep collecting
//                 '}'                       -> MAC_DONE
//                 ',' and not the 16th byte -> MAC_IDLE, keep collecting
//                 anything else             -> MAC_ERR
//   MAC_DONE  : assert frame_done for one cycle. The buffer still holds the
//               complete frame, so downstream reads it directly.
//   MAC_ERR   : assert frame_err for one cycle. The frame is abandoned and
//               byte_idx cleared, so the start gate discards the remainder
//               until the next real '{'.
//
// From MAC_DONE or MAC_ERR the FSM goes straight to MAC_STORE if the next
// frame's '{' is already arriving, otherwise to MAC_IDLE.
//
// WHY MAC_ERR EXISTS. Aborting a malformed frame silently would reintroduce
// the exact bug this rework removed: a frame that vanishes with no error
// pulse. frame_err is routed onward so a framing fault is still counted.
//
// SOFT RESET: par_val_rst (rx_phy's parity_err_pulse) forces MAC_IDLE and
// clears byte_idx from any state, abandoning a frame corrupted by a
// failed-parity byte.
//
// NOTE: five states no longer fit in two bits. The encoding is three bits
// wide; the previous 2-bit values are NOT preserved, since nothing outside
// this package depends on the numeric encoding.

`timescale 1ns/1ps

package rx_mac_pkg;
    typedef enum logic [2:0] {
        MAC_IDLE  = 3'd0,
        MAC_STORE = 3'd1,
        MAC_CHK   = 3'd2,
        MAC_DONE  = 3'd3,
        MAC_ERR   = 3'd4
    } rx_mac_state_t;
endpackage : rx_mac_pkg
