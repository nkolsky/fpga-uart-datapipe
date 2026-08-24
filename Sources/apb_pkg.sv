`timescale 1ns/1ps
// apb_pkg.sv
// ==========
// Single source of truth for the APB address space: signal widths, the
// aperture map, and the decode helpers. apb_master, apb_bar and every APB
// slave read from this file and nowhere else.
//
// ADDRESS SPLIT
//   PADDR[15:8]   aperture (which slave)
//   PADDR[ 7:0]   offset   (which register inside it)
//
// So aperture N occupies 0xNN00..0xNNFF and the offset is exactly the 8-bit
// address rgf already uses -- IMG_STATUS stays 0x00, PARITY_FAULT_CNT stays
// 0x14, now at 0x0000 and 0x0014 in the system space. The split is on a byte
// boundary so the decode is one 8-bit equality compare, off the critical path.
//
// THE WIRE PROTOCOL IS UNCHANGED
// A Register Write is {W<A2,A1,A0>, V<..>, V<..>} -- the address group carries
// 24 bits, of which rx_classifier requires the top 16 to be zero. That check
// is NOT relaxed. addr_from_msg zero-extends the 8 bits that survive it, so
// every host-issued access lands in aperture 0 by construction. The system
// space is wider than the wire on purpose: apertures 1..255 are reachable by
// any future on-chip requester, and opening them to the host is a one-line
// change to rgf_addr_ok if something is ever mapped there.
//
// SLAVE TABLE
//   0x00xx  RGF, 6 x 32-bit configuration registers        IMPLEMENTED
//   0x01xx .. 0xFFxx  reserved
//
// An access to a reserved aperture is NOT dropped. apb_bar's default responder
// completes it in one cycle with PSLVERR. That is a deadlock guard, not a
// courtesy: with nothing driving PREADY the master would sit in ACCESS
// forever, holding busy, stalling mem_msg_router and the crossing, and
// UART_CTS would never release.
//
// WIDENING PADDR
// ADDR_W is 16 because 256 apertures of 256 bytes is more space than this
// design will use. AMBA permits up to 32. Widening is a single edit here --
// SEL_W and every function below follow -- and costs nothing in silicon while
// the message path is the only requester, since apb_master's upper address
// flops are fed constant zero and trim away.

package apb_pkg;

    // -----------------------------------------------------------------
    // Bus widths
    //
    // OFFSET_W and DATA_W must match rgf_pkg::ADDR_WIDTH and
    // rgf_pkg::DATA_WIDTH. They are declared here rather than derived so the
    // bus does not inherit its shape from one slave; apb_slave_rgf asserts
    // the equality at elaboration, the same way rx_classifier does for
    // ADDR_W_RGF.
    // -----------------------------------------------------------------
    localparam int ADDR_W   = 16;                 // full system address
    localparam int DATA_W   = 32;
    localparam int OFFSET_W = 8;                  // PADDR[7:0]
    localparam int SEL_W    = ADDR_W - OFFSET_W;  // PADDR[15:8]

    // Byte lanes. The RGF is word-only so every access drives all four, but
    // the signal exists so a byte-addressable slave added later has something
    // real to connect to rather than needing the bus widened around it.
    localparam int STRB_W   = DATA_W / 8;

    // -----------------------------------------------------------------
    // Slave table
    //
    // NUM_SLAVES sizes the PSEL vector and apb_bar's response mux. Adding an
    // aperture means adding an enum member and a SLAVE_BASE entry -- the
    // decode and the mux are generated from this table, so nothing else in
    // the bus changes.
    // -----------------------------------------------------------------
    localparam int NUM_SLAVES = 1;

    typedef enum int {
        SLAVE_RGF = 0
    } slave_id_e;

    // Indices are integers rather than enum members: Verilator 5.x rejects an
    // enum-keyed assignment pattern ("Assignment pattern key not supported"),
    // and Lab 11 asks for a clean lint report. The enum above still names the
    // index everywhere it is used.
    localparam logic [SEL_W-1:0] SLAVE_BASE [NUM_SLAVES] = '{
        0 : 8'h00      // SLAVE_RGF
    };

    // -----------------------------------------------------------------
    // Decode helpers
    // -----------------------------------------------------------------
    // Both take a full address and return one field, so each leaves the other
    // field unread. That is the point of them; silence the -Wall complaint
    // rather than reshape the interface.
    /* verilator lint_off UNUSEDSIGNAL */
    function automatic logic [SEL_W-1:0] addr_sel(input logic [ADDR_W-1:0] a);
        return a[ADDR_W-1 -: SEL_W];
    endfunction

    function automatic logic [OFFSET_W-1:0] addr_offset(input logic [ADDR_W-1:0] a);
        return a[OFFSET_W-1:0];
    endfunction
    /* verilator lint_on UNUSEDSIGNAL */

    // Zero-extend a message-borne register address into the system space.
    // Aperture 0 by construction -- see the header.
    function automatic logic [ADDR_W-1:0] addr_from_msg(
        input logic [OFFSET_W-1:0] msg_addr
    );
        return {{SEL_W{1'b0}}, msg_addr};
    endfunction

endpackage : apb_pkg
