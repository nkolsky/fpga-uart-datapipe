// rx_phy_pkg.sv
// -------------
// State encoding for the RX PHY FSM.
//
// State flow:
//   RX_IDLE    : line idle-high; waiting for falling edge (start bit detected)
//   RX_START   : validate start bit at tick_q==8 (centre of bit period);
//                abort back to IDLE if line is not low (noise/glitch)
//   RX_DATA    : sample rx_in at tick_q==8 for each of 8 data bits;
//                shift into byte register LSB-first
//   RX_PARITY  : wait a full bit period (same as every other state -- the
//                parity bit takes a full bit period to physically arrive
//                on the wire, same as any other bit), then check parity
//                at tick_q==10 against shift_reg (the byte just received).
//                byte_valid is gated on this result: a byte that fails
//                parity is never forwarded to the MAC.
//   RX_STOP    : validate stop bit at tick_q==10;
//                abort back to IDLE if line is not high (framing error)
//   RX_DONE    : assert byte_valid for one cycle (only if parity_valid)
//                and present rx_byte; also asserts parity_err_pulse for
//                one cycle if parity failed. Returns to IDLE on the next
//                clock edge.

`timescale 1ns/1ps

package rx_phy_pkg;
    typedef enum logic [2:0] {
        RX_IDLE   = 3'b000,
        RX_START  = 3'b001,
        RX_DATA   = 3'b010,
        RX_PARITY = 3'b011,
        RX_STOP   = 3'b100,
        RX_DONE   = 3'b101
    } rx_phy_state_t;
endpackage : rx_phy_pkg
