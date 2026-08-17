// -----------------------------------------------------------------------------
// memory_pkg.sv
//
// Shared geometry and memory-organisation constants for the image SRAMs.
// IMG_WIDTH and IMG_HEIGHT define the image size; the remaining values are
// derived from them so the packer, reader, and SRAM ports all agree on the same
// word packing and address width.
//
// The SRAM array is three separate single-channel memories: R, G, and B. Each
// 32-bit word stores four 8-bit pixels of one channel, with the MSB byte being
// the leftmost pixel in the row.
// -----------------------------------------------------------------------------

`timescale 1ns/1ps

package memory_pkg;

    // -----------------------------------------------------------------
    // Image geometry
    //
    // SIMULATION is defined by the testbench build (see the UVM Makefile,
    // which passes -DSIMULATION) and is never defined for synthesis.
    //
    // WHY A SMALLER IMAGE IN SIMULATION
    // A 256x256 burst is 65536 pixels. At four pixels per message and 176
    // clocks per UART byte that is roughly 46 million clocks -- unusable for
    // waveform inspection and slow even without tracing. An 8x8 image is 64
    // pixels, 16 data messages, and every burst address fits on one screen.
    //
    // Nothing else needs to change: every bound is compared against
    // IMG_WIDTH / IMG_HEIGHT directly rather than against a hardcoded 256,
    // and the derived memory organisation below follows automatically.
    //
    // WHAT ALSO MOVES, AND WHY IT MATTERS
    // SRAM_DEPTH and SRAM_ADDR_WIDTH are derived, so they shrink too:
    //
    //     hardware    256x256   SRAM_DEPTH 16384   SRAM_ADDR_WIDTH 14
    //     simulation    8x8     SRAM_DEPTH    16   SRAM_ADDR_WIDTH  4
    //
    // Any SRAM init file sized for the hardware geometry will NOT match the
    // simulation build. Generate init data from these parameters rather than
    // assuming 16384 words.
    //
    // NOTE: four signals in chip_top -- rom_addr, pix_sram_rd_addr,
    // sram_rd_addr_mux and brd_sram_rd_addr -- are still declared as literal
    // [13:0] rather than using SRAM_ADDR_WIDTH. Those do NOT track this
    // parameter. They remain wide enough for either geometry, so simulation
    // is correct, but they are parameterised in name only. See
    // DEFERRED_CLEANUP.md.
    // -----------------------------------------------------------------
`ifdef SIMULATION
    localparam int IMG_WIDTH       = 8;     // pixels per row
    localparam int IMG_HEIGHT      = 8;     // rows per image
`else
    localparam int IMG_WIDTH       = 256;   // pixels per row
    localparam int IMG_HEIGHT      = 256;   // rows per image
`endif

    // -----------------------------------------------------------------
    // SRAM INITIALISATION FILES
    //
    // The file must contain exactly SRAM_DEPTH words, and SRAM_DEPTH is
    // derived from the geometry above -- 16384 for hardware, 16 for
    // simulation. A hardware-sized file loaded into a 16-deep array is a
    // silent mismatch, so the NAME is selected by the same define that
    // selects the geometry.
    //
    // Both sets are produced by tools/gen_sram_init.py from these
    // parameters. Do not hand-edit them: a file that disagrees with
    // SRAM_DEPTH is exactly the "parameterised in name only" failure that
    // already bit rom_sequencer and the [13:0] address literals.
    // -----------------------------------------------------------------
`ifdef SIMULATION
    localparam string SRAM_INIT_R = "red_hex_sim.mem";
    localparam string SRAM_INIT_G = "green_hex_sim.mem";
    localparam string SRAM_INIT_B = "blue_hex_sim.mem";
`else
    localparam string SRAM_INIT_R = "red_hex.mem";
    localparam string SRAM_INIT_G = "green_hex.mem";
    localparam string SRAM_INIT_B = "blue_hex.mem";
`endif

    // -----------------------------------------------------------------
    // Pixel packing
    // -----------------------------------------------------------------
    localparam int CHANNEL_WIDTH   = 8;     // bits per colour channel sample
    localparam int PIXELS_PER_WORD = 4;     // channel samples packed per word

    // -----------------------------------------------------------------
    // Derived memory organisation (per colour channel)
    // -----------------------------------------------------------------
    localparam int SRAM_DATA_WIDTH = CHANNEL_WIDTH * PIXELS_PER_WORD;
                                            // = 32 bits

    localparam int SRAM_DEPTH      = (IMG_WIDTH * IMG_HEIGHT) / PIXELS_PER_WORD;
                                            // = 16384 words

    localparam int SRAM_ADDR_WIDTH = $clog2(SRAM_DEPTH);
                                            // = 14 bits

endpackage : memory_pkg
