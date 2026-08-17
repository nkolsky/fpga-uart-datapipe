// fifo_pkg.sv
// -----------
// Shared FIFO config and Gray-code helpers for the image FIFO.
//
// Depth, thresholds and pointer width are fixed so the Gray/bin helpers stay
// consistent across the dual-clock queue.

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
    // Gray-code helpers
    // -----------------------------------------------------------------

    // Binary to Gray: gray = binary ^ (binary >> 1)
    // Only one bit changes between consecutive values, so the pointer can be
    // transferred safely across the CDC boundary.
    function automatic logic [PTR_WIDTH-1:0] bin2gray(
        input logic [PTR_WIDTH-1:0] bin
    );
        return bin ^ (bin >> 1);
    endfunction

    // Gray to Binary: reconstruct the binary pointer for occupancy checks.
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