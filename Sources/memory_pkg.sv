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
// Stage 1 note: SRAM_ADDR_WIDTH is derived with $clog2 and evaluates to
// 14 for the current geometry, matching the existing 14-bit rom_addr in
// chip_top.sv. The chip_top signal declarations were intentionally left
// hardcoded in this pass to keep the Stage 1 diff minimal; switching
// them to SRAM_ADDR_WIDTH is a zero-risk follow-up.

`timescale 1ns/1ps

package memory_pkg;

    // -----------------------------------------------------------------
    // Image geometry
    // -----------------------------------------------------------------
    localparam int IMG_WIDTH       = 256;   // pixels per row
    localparam int IMG_HEIGHT      = 256;   // rows per image

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
