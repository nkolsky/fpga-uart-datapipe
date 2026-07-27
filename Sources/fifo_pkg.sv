// async_fifo_pkg.sv
// -----------------
// Parameters and gray-code helper functions for the Lab 9 async FIFO.
//
// Sizing rationale (see design discussion):
//   DEPTH = 64        - unchanged from the sync FIFO; burst pattern from
//                        rom_sequencer (4 pixels per ROM word) is unchanged,
//                        so the same buffering depth still applies.
//   AF    = 52         - derived from worst-case overshoot past the AF
//                        threshold before rom_sequencer reacts:
//                          + up to 3 pixels: rom_sequencer only checks
//                            almost_full at NEXT_ADDR, after a full 4-pixel
//                            ROM word has already been pushed (burst
//                            granularity overshoot, same in sync/async)
//                          + up to 1 pixel: synchronizer lag on rd_ptr
//                            (2 read-clock cycles @ 1 read/4 cycles ≈ 1
//                            pixel of stale visibility on the write side)
//                        Worst case occupancy reached = AF + 4 = 56,
//                        leaving 8 slots of true headroom before DEPTH=64.
//   AE    = 8           - unchanged from the sync FIFO. The read-side
//                        equivalent lag only delays almost_empty's
//                        assertion (a throughput cost, not a correctness
//                        risk), so no widening was required here.
//
// Pointer width:
//   Pointers are ADDR_WIDTH+1 bits wide (one extra MSB beyond the address
//   range) to allow full/empty disambiguation when wr_ptr and rd_ptr wrap
//   around the circular buffer - standard Cummings async FIFO technique.

`timescale 1ns/1ps

package fifo_pkg;

    // -----------------------------------------------------------------
    // FIFO sizing parameters
    // -----------------------------------------------------------------
    localparam int FIFO_DATA_WIDTH = 24;            // R0,G0,B0 packed pixel
    localparam int DEPTH      = 64;            // number of entries
    localparam int ADDR_WIDTH = $clog2(DEPTH); // 6 bits for depth=64

    localparam int AF_THRESHOLD = 52;          // assert almost_full at this occupancy
    localparam int AE_THRESHOLD = 8;           // assert almost_empty at this occupancy

    // Pointer width: one extra bit beyond ADDR_WIDTH for wrap disambiguation
    localparam int PTR_WIDTH = ADDR_WIDTH + 1;

    // -----------------------------------------------------------------
    // Synchronizer depth (number of flop stages in each NDFF sync chain)
    // -----------------------------------------------------------------
    localparam int SYNC_STAGES = 2;

    // -----------------------------------------------------------------
    // Gray code conversion functions
    // -----------------------------------------------------------------

    // Binary to Gray: gray = binary ^ (binary >> 1)
    // Only one bit changes between consecutive gray-coded values, making
    // it safe to sample across an asynchronous clock-domain boundary
    // without risking a multi-bit transition glitch (metastability-safe
    // value, even if the sampling flop itself can still go metastable -
    // that risk is handled by the multi-stage synchronizer, not by gray
    // coding alone).
    function automatic logic [PTR_WIDTH-1:0] bin2gray(
        input logic [PTR_WIDTH-1:0] bin
    );
        return bin ^ (bin >> 1);
    endfunction

    // Gray to Binary: reconstructs the binary value via XOR-cascade.
    // Needed when converting a synchronized gray pointer back to binary
    // for occupancy / almost_full / almost_empty arithmetic.
    function automatic logic [PTR_WIDTH-1:0] gray2bin(
        input logic [PTR_WIDTH-1:0] gray
    );
        logic [PTR_WIDTH-1:0] bin;
        int i;
        begin
            bin[PTR_WIDTH-1] = gray[PTR_WIDTH-1];
            for (i = PTR_WIDTH-2; i >= 0; i--)
                bin[i] = bin[i+1] ^ gray[i];
            return bin;
        end
    endfunction

endpackage : fifo_pkg