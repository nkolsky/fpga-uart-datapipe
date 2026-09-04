`timescale 1ns/1ps
// ahb_pkg.sv
// ==========
// Single source of truth for the AHB-Lite bus: protocol encodings, address
// map, and decode helpers. ahb_master, ahb_decoder and every slave read from
// this file.
//
// ADDRESS MAP -- byte addressed, as AHB requires
//
//   HADDR[17:16]  channel   00=R  01=G  10=B  11=unmapped
//   HADDR[15:2]   word index into that channel's SRAM (16384 words)
//   HADDR[1:0]    byte within the word -- always 00, this bus is word-only
//
//   0x0_0000 .. 0x0_FFFF   R SRAM   64 KB
//   0x1_0000 .. 0x1_FFFF   G SRAM
//   0x2_0000 .. 0x2_FFFF   B SRAM
//   0x3_0000 .. 0x3_FFFF   unmapped -> default slave, HRESP = ERROR
//
// This is a SEPARATE address space from apb_pkg's. The two buses have their
// own decoders and never see each other's transfers, so the RGF at APB
// 0x0000 and the R SRAM at AHB 0x00000 do not collide -- they are not in the
// same space. See the note in apb_pkg.sv.
//
// HREADY vs HREADYOUT -- the classic AHB-Lite mistake
//
//   HREADYOUT  an output of each subordinate: "I am done"
//   HREADY     an input to every subordinate: the AGGREGATED ready of the
//              segment, i.e. the selected slave's HREADYOUT muxed back out
//
// A subordinate needs HREADY to know when the address phase it is watching
// actually completes. Wiring a slave's own HREADYOUT back into its HREADY
// works only while there is one slave, and fails silently the moment a
// second is added.

package ahb_pkg;

    // -----------------------------------------------------------------
    // Bus widths
    // -----------------------------------------------------------------
    localparam int ADDR_W  = 32;   // AMBA standard; upper bits trim away
    localparam int DATA_W  = 32;   // matches memory_pkg::SRAM_DATA_WIDTH

    localparam int SEL_LSB = 16;                 // HADDR[17:16] picks a channel
    localparam int SEL_W   = 2;
    localparam int WORD_W  = 14;                 // HADDR[15:2], 16384 words

    // -----------------------------------------------------------------
    // Protocol encodings (AMBA 3 AHB-Lite)
    // -----------------------------------------------------------------
    typedef enum logic [1:0] {
        HTRANS_IDLE   = 2'b00,   // no transfer
        HTRANS_BUSY   = 2'b01,   // manager stalling INSIDE a burst -- the
                                 // burst is NOT over, an arbiter must hold
                                 // the grant across it
        HTRANS_NONSEQ = 2'b10,   // single transfer, or first beat of a burst
        HTRANS_SEQ    = 2'b11    // remaining beats of a burst
    } htrans_e;

    typedef enum logic [2:0] {
        HBURST_SINGLE = 3'b000,
        HBURST_INCR   = 3'b001,  // undefined length: ends when HTRANS returns
                                 // to IDLE/NONSEQ, so it cannot be counted
        HBURST_WRAP4  = 3'b010,
        HBURST_INCR4  = 3'b011,  // what this design uses
        HBURST_WRAP8  = 3'b100,
        HBURST_INCR8  = 3'b101,
        HBURST_WRAP16 = 3'b110,
        HBURST_INCR16 = 3'b111
    } hburst_e;

    typedef enum logic [2:0] {
        HSIZE_BYTE  = 3'b000,
        HSIZE_HALF  = 3'b001,
        HSIZE_WORD  = 3'b010,    // 32-bit, the only size this bus issues
        HSIZE_DWORD = 3'b011
    } hsize_e;

    // AHB-Lite response is one bit, unlike full AHB's two.
    localparam logic HRESP_OKAY  = 1'b0;
    localparam logic HRESP_ERROR = 1'b1;

    localparam int BEATS_PER_BURST = 4;          // INCR4

    // -----------------------------------------------------------------
    // Slave table
    //
    // Integer keys, not enum members: Verilator rejects an enum-keyed
    // assignment pattern, and Lab 12 wants a clean lint report. The enum
    // still names the index everywhere it is used.
    // -----------------------------------------------------------------
    localparam int NUM_SLAVES = 3;

    typedef enum int {
        SLAVE_R = 0,
        SLAVE_G = 1,
        SLAVE_B = 2
    } slave_id_e;

    localparam logic [SEL_W-1:0] SLAVE_BASE [NUM_SLAVES] = '{
        0 : 2'b00,      // SLAVE_R
        1 : 2'b01,      // SLAVE_G
        2 : 2'b10       // SLAVE_B
    };

    // -----------------------------------------------------------------
    // Decode helpers
    // -----------------------------------------------------------------
    /* verilator lint_off UNUSEDSIGNAL */
    // Each takes a full address and returns one field, so each leaves the
    // rest unread. That is the point of them.
    function automatic logic [SEL_W-1:0] addr_sel(input logic [ADDR_W-1:0] a);
        return a[SEL_LSB +: SEL_W];
    endfunction

    function automatic logic [WORD_W-1:0] addr_word(input logic [ADDR_W-1:0] a);
        return a[2 +: WORD_W];
    endfunction
    /* verilator lint_on UNUSEDSIGNAL */

    // Build a byte address from a channel and a word index. The two low bits
    // are always zero -- word-aligned, HSIZE_WORD.
    function automatic logic [ADDR_W-1:0] addr_of(
        input logic [SEL_W-1:0]  channel,
        input logic [WORD_W-1:0] word_idx
    );
        logic [ADDR_W-1:0] a;
        a = '0;
        a[SEL_LSB +: SEL_W] = channel;
        a[2 +: WORD_W]      = word_idx;
        return a;
    endfunction

endpackage : ahb_pkg
