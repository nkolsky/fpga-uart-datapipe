// rx_burst_pkg.sv
// ---------------
// Stage 3 (Image Burst Write): shared types and constants.
//
// Deliberately dependency-free -- it defines only burst-specific values, so
// the geometry limits stay in memory_pkg where they belong and this package
// can be included anywhere without pulling in the memory architecture.
//
// -----------------------------------------------------------------------
// MESSAGE FORMATS (Final Project spec, section 11)
// -----------------------------------------------------------------------
// Header, 16 bytes:
//
//   {I<0,0,0>, H<H2,H1,H0>, W<W2,W1,W0>}
//
//   byte  0  '{'                      byte  8  H1
//   byte  1  'I'   burst-write opcode byte  9  H0
//   byte  2  don't care               byte 10  ','
//   byte  3  don't care               byte 11  'W'
//   byte  4  don't care               byte 12  W2
//   byte  5  ','                      byte 13  W1
//   byte  6  'H'                      byte 14  W0
//   byte  7  H2                       byte 15  '}'
//
// 7 fixed bytes, 9 payload, 3 of them don't-care -- matching the spec's own
// tabulation exactly.
//
// Data, 16 bytes:
//
//   {<R0,G0,B0,R1>, <G1,B1,R2,G2>, <B2,R3,G3,B3>}
//
// Strip bytes 0, 5, 10 and 15 and the remaining twelve are four consecutive
// RGB triplets in order. The <...> groupings are frame punctuation, not a
// reordering -- the commas simply land mid-pixel, which is why the spec
// describes it as three 32-bit groups.
//
// ASSUMPTION (A4): the data message carries NO opcode at byte 1 -- that
// position holds R0, a raw pixel value. This is what makes bypass_active
// necessary, and it follows from the spec's "will enable opcode bypass for
// the duration of HxW pixels". If the data message turns out to carry an
// opcode after all, the bypass mechanism is unnecessary and the receive-side
// design changes shape.
//
// ASSUMPTION (A2): H and W are BINARY big-endian, three bytes each. Three
// ASCII decimal digits would cap a dimension at 999, and msg_composer
// already establishes that <X2,X1,X0> means three raw bytes.

`timescale 1ns/1ps

package rx_burst_pkg;

    // -----------------------------------------------------------------
    // Field widths as they appear on the wire
    // -----------------------------------------------------------------
    localparam int BURST_DIM_W       = 24;  // H and W, three bytes each
    localparam int BURST_PIX_W       = 24;  // {R,G,B}
    localparam int BURST_ADDR_W      = 24;  // linear pixel index, matches
                                            // the Stage 2C command format

    // Pixels carried by one Burst Data message.
    localparam int BURST_PIX_PER_MSG = 4;
    localparam int BURST_SLOT_W      = $clog2(BURST_PIX_PER_MSG);  // 2

    // -----------------------------------------------------------------
    // Burst controller states
    //
    // B_IDLE   : not in a burst. bypass_active low, so incoming frames are
    //            decoded normally and a header can be recognised.
    // B_ACTIVE : bypass_active high, waiting for the next Burst Data frame.
    // B_EMIT   : unpacking one Burst Data frame into up to four ordinary
    //            {address, pixel} commands, one per clock.
    // B_DONE   : the final valid pixel has been emitted; drop bypass_active
    //            and return to idle so the next frame decodes normally.
    //
    // Recovery is funnelled through a single burst_abort term so a timeout
    // or an explicit abort command can be added later without restructuring
    // the FSM:
    //
    //     if (burst_done || burst_abort) state <= B_IDLE;
    //
    // Initially burst_abort is tied inactive and reset is the only recovery
    // path, which is the agreed first-implementation scope.
    // -----------------------------------------------------------------
    typedef enum logic [2:0] {
        B_IDLE   = 3'd0,
        B_ACTIVE = 3'd1,
        B_EMIT   = 3'd2,
        B_DONE   = 3'd3
    } burst_state_t;

endpackage : rx_burst_pkg
