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
    // ONE 32-BIT CHANNEL WORD PER ENTRY, not one packed pixel.
    //
    // There are three of these now, one per colour channel. An entry is a
    // whole SRAM word -- four consecutive pixel values of ONE channel -- so a
    // pixel is assembled on the READ side by taking the same byte lane from
    // all three FIFOs. That is the shape a per-channel INCR4 burst delivers:
    // four beats of R, then four of G, then four of B, so the channels arrive
    // at different times and each needs its own buffer.
    localparam int FIFO_DATA_WIDTH = 32;            // one SRAM word, 4 px of one channel
    // 16 entries x 4 pixels = 64 pixels buffered per channel -- the same
    // depth of image the single 24-bit FIFO held, at a quarter the entries.
    //
    // Sized for bursts, not for rate. An INCR4 is 4 entries, so 16 gives four
    // bursts of headroom; the thresholds below leave room for one burst
    // already in flight when almost_full asserts, since a burst cannot be
    // stopped part way. Rate is irrelevant here -- the UART consumes a pixel
    // every 22 us against a producer that makes one every few clocks.
    localparam int DEPTH      = 16;            // number of entries
    localparam int ADDR_WIDTH = $clog2(DEPTH); // 4 bits for depth=16

    // 12 of 16: leaves 4 slots, exactly one INCR4, for a burst that is
    // already running when almost_full asserts.
    localparam int AF_THRESHOLD = 12;          // assert almost_full at this occupancy
    // 4 of 16: resume once a burst's worth has drained. The resume signal
    // crosses back through cdc_level_sync, which the reader's 22 us pixel
    // period dwarfs.
    localparam int AE_THRESHOLD = 4;           // assert almost_empty at this occupancy

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