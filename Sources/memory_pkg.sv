// memory_pkg.sv
// -------------
// Shared project-level image and memory architecture constants.
//
// Scope: this package holds values that are FIXED PROPERTIES OF THE
// PROJECT ARCHITECTURE -- the image geometry and the resulting memory
// organisation. It deliberately does NOT hold per-instance
// configuration: the rgb_sram module keeps DATA_WIDTH / DEPTH /
// INIT_FILE as its own parameters so it stays independently
// configurable and reusable outside this design. chip_top.sv passes the
// values below in at each instantiation, and passes the per-channel
// initialisation filenames separately.
//
// Derivation chain (nothing below is duplicated -- each value is
// computed from the ones above it):
//
//   IMG_WIDTH x IMG_HEIGHT        pixels in the image
//   / PIXELS_PER_WORD             pixels packed into each memory word
//   = SRAM_DEPTH                  words per colour channel
//
//   CHANNEL_WIDTH x PIXELS_PER_WORD = SRAM_DATA_WIDTH
//
// Memory organisation (unchanged from the Lab 10 ROM implementation):
// three independent single-channel memories (R, G, B). Each word holds
// PIXELS_PER_WORD consecutive pixels of ONE colour channel, most
// significant byte = leftmost pixel. A full RGB pixel is therefore
// assembled by reading the same address from all three memories and
// selecting the same byte lane from each -- which is exactly what
// rom_sequencer.sv's LATCH state does.
//
// ADDRESS WIDTH -- PARTIALLY ADOPTED IN chip_top.sv, BY HISTORY NOT DESIGN
//
// SRAM_ADDR_WIDTH is derived with $clog2 and evaluates to 14 for the
// current geometry. chip_top.sv is currently MIXED: signals added later
// use the parameter, while the original Lab 10 declarations are still
// written as literal [13:0]:
//
//   uses SRAM_ADDR_WIDTH   sram_wr_addr
//   still hardcoded [13:0] rom_addr, pix_sram_rd_addr,
//                          sram_rd_addr_mux, brd_sram_rd_addr
//
// This is harmless TODAY only because the two values coincide at 14. They
// are not linked by anything: change IMG_WIDTH, IMG_HEIGHT or
// PIXELS_PER_WORD below and SRAM_ADDR_WIDTH moves while the four literals
// do not, silently truncating or over-widening those buses. Vivado reports
// that as a width warning at most, never an elaboration error.
//
// Converting the four remaining declarations is a zero-risk follow-up and
// is the single change that would make the geometry above genuinely the
// only place image size is defined. Deliberately not done here, since the
// RTL is frozen -- but it should be recorded as a known limitation rather
// than left looking parameterised when it is only half so.

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
