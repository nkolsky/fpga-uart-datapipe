// rgb_sram.sv
// -----------
// Single-channel image SRAM. Drop-in replacement for the read side of
// rgb_rom.sv's `rom` module, plus a byte-enabled write port.
//
// THE WRITE PORT IS LIVE. It was tied to 1'b0 when this module was first
// introduced; it is now driven by mem_write_subsystem, which serves both
// the Single Pixel Write path and the Image Burst Write path -- both arrive
// as MESSAGES on cdc_msg_sync, and mem_msg_writer turns each into a
// rectangle that pixel_word_packer fills. All three channel instances share
// one wr_en / wr_be / wr_addr and take their own wr_data.
//
// Earlier revisions named sram_wr_ctrl here, fed by a 48-bit command FIFO.
// Both are gone.
//
// -----------------------------------------------------------------------
// TIMING CONTRACT -- must match rgb_rom.sv exactly
// -----------------------------------------------------------------------
// The read interface is bit- and cycle-identical to the ROM it replaces,
// because rom_sequencer.sv is NOT being modified and depends on all four
// of these properties:
//
//   1. One clock cycle of read latency. rd_addr is captured on the
//      rising edge; rd_data is valid on the following cycle.
//   2. Registered output. rd_data is a flip-flop, not combinational.
//   3. rd_data HOLDS ITS PREVIOUS VALUE while rd_en is low. It does not
//      clear, does not go to X, and does not follow rd_addr.
//   4. No additional output pipeline stage.
//
// Property 3 is load-bearing, not cosmetic. Tracing rom_sequencer.sv:
// rom_rd_en and rom_addr are REGISTERED outputs assigned during the
// READ_ROM state, so they are actually asserted during WAIT_ROM. The
// memory captures at the end of that cycle. By the time the FSM reaches
// LATCH and samples red_data/green_data/blue_data into its pixels[]
// array, rom_rd_en is already back low. A memory that zeroed or X'd its
// output on enable deassert would break the design on the first pixel.
//
// On 7-series block RAM this behaviour is native, not synthesised:
// rd_en maps onto the block RAM's port enable pin, and a disabled port
// simply does not update its output register.
//
// There is deliberately no reset on rd_data -- matching rgb_rom.sv.
// Adding one would change the power-up value for no benefit here.
//
// -----------------------------------------------------------------------
// READ / WRITE COLLISION POLICY -- PROHIBITED BY CONSTRUCTION
// -----------------------------------------------------------------------
// The RTL below reads mem[rd_addr] on the right-hand side of a
// non-blocking assignment, so in SIMULATION a same-address, same-cycle
// read-while-write returns the OLD contents (read-first). Statement
// order within the always_ff block does not change this.
//
// That guarantee DOES NOT SURVIVE TO HARDWARE. On 7-series block RAM, a
// simultaneous access to the same address from two ports where one is
// writing yields INVALID read data -- the RTL simulates deterministically
// while the silicon returns something arbitrary. (Stored contents are
// only at risk when both ports write the same address, which cannot
// happen here: the read port never writes.)
//
// Rather than paper over that mismatch with write-forwarding logic, the
// collision is declared ILLEGAL and enforced from two directions:
//
//   1. STRUCTURALLY, by mem_interlock.sv. It is the single arbiter of
//      this memory and grants exclusive ownership to exactly one of the
//      three clients -- the full-frame reader (rom_sequencer), the
//      writer (mem_write_subsystem, serving both single-pixel and burst
//      writes), and the read-port borrower (pixel_rd_ctrl or
//      burst_rd_ctrl, arbitrated ahead of the interlock inside
//      memory_subsystem).
//      wr_allowed and read_go are never both live, so rd_en and wr_en
//      cannot be high in the same cycle at all, let alone at the same
//      address.
//
//   2. BY ASSERTION, via the simulation-only check at the bottom of this
//      file. This is the backstop: if a future change to the interlock
//      or to memory_subsystem's read mux breaks the exclusion above, the
//      assertion turns a silent hardware-only bug into a loud simulation
//      failure.
//
// The condition is no longer vacuous. wr_en is genuinely exercised now,
// so the assertion is doing real work in every testbench that writes.
//
// -----------------------------------------------------------------------
// SYNTHESIS NOTE
// -----------------------------------------------------------------------
// With the write port live, this no longer constant-propagates back into
// a ROM: Vivado must infer true byte-enabled simple-dual-port block RAM
// from the always_ff template below. Expect utilisation to differ from
// the read-only Lab 10 baseline, and check the synthesis report for
// three things in particular:
//
//   - the RAM is inferred as block, not distributed (the ram_style
//     attribute below asks for block; a warning here means the byte-write
//     loop failed to match UG901's template)
//   - the byte-write enables map onto the BRAM WE pins rather than
//     becoming a read-modify-write in fabric
//   - no unexpected extra BRAM from a failed inference falling back to
//     LUTRAM
//
// Read-side timing and the four properties above are unchanged by the
// write port going live -- the read port is still a plain enabled,
// registered output.

`timescale 1ns/1ps

module rgb_sram #(
    parameter int    DATA_WIDTH = 32,
    parameter int    DEPTH      = 16384,
    parameter string INIT_FILE  = ""        // "" = no initialisation
)(
    input  logic                     clk,

    // -----------------------------------------------------------------
    // Read port
    // -----------------------------------------------------------------
    input  logic                     rd_en,
    input  logic [$clog2(DEPTH)-1:0] rd_addr,
    output logic [DATA_WIDTH-1:0]    rd_data,

    // -----------------------------------------------------------------
    // Write port -- LIVE, driven by pixel_word_packer.
    // wr_be selects which byte lanes of the addressed word are updated;
    // a lane whose bit is low retains its current contents. This exists
    // because each 32-bit channel word packs four 8-bit pixels, so the
    // write path can fill a word one pixel at a time without a
    // read-modify-write. pixel_word_packer computes the lane index as
    // idx = (NLANE-1) - pixel_index[1:0] and sets wr_be[idx] -- MSB lane
    // is the LEFTMOST pixel; see the LANE ORIENTATION section of
    // pixel_word_packer.sv. rom_sequencer, pixel_rd_ctrl and burst_rd_ctrl
    // all read with the same orientation.
    // -----------------------------------------------------------------
    input  logic                     wr_en,
    input  logic [DATA_WIDTH/8-1:0]  wr_be,
    input  logic [$clog2(DEPTH)-1:0] wr_addr,
    input  logic [DATA_WIDTH-1:0]    wr_data
);

    localparam int NUM_BYTES = DATA_WIDTH / 8;

    // -----------------------------------------------------------------
    // Storage array.
    //
    // The [0:DEPTH-1] ASCENDING range is REQUIRED, not stylistic.
    // $readmemh without explicit start/finish addresses fills from the
    // LEFT-HAND bound of the array. Declaring [DEPTH-1:0] instead would
    // load mem[16383] first and fill downward, producing a perfectly
    // functional memory containing a reversed image -- a silent failure
    // that a module-to-module equivalence check cannot catch if both
    // modules share the mistake. rgb_rom.sv uses [0:DEPTH-1]; this
    // matches it. The equivalence testbench additionally checks both
    // DUTs against an independently loaded reference array to close
    // that hole.
    // -----------------------------------------------------------------
    (* ram_style = "block" *)
    logic [DATA_WIDTH-1:0] mem [0:DEPTH-1];

    // -----------------------------------------------------------------
    // Optional initialisation.
    //
    // Guarded by a generate rather than called unconditionally:
    // $readmemh("") raises a file-open error rather than silently doing
    // nothing, which would defeat the purpose of the parameter. With
    // INIT_FILE left at its default the block is not elaborated at all,
    // and the memory powers up uninitialised.
    // -----------------------------------------------------------------
    generate
        if (INIT_FILE != "") begin : g_init
            initial begin
                $readmemh(INIT_FILE, mem);
            end
        end
    endgenerate

    // -----------------------------------------------------------------
    // Memory access.
    //
    // Write and read live in the same always_ff block: this is the
    // standard Vivado simple-dual-port inference template (UG901), with
    // a byte-write loop on the write side and a port enable on the read
    // side. No reset anywhere -- a reset on the array would prevent
    // block RAM inference outright.
    // -----------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (wr_en) begin
            for (int i = 0; i < NUM_BYTES; i++) begin
                if (wr_be[i]) begin
                    mem[wr_addr][i*8 +: 8] <= wr_data[i*8 +: 8];
                end
            end
        end

        if (rd_en) begin
            rd_data <= mem[rd_addr];
        end
    end

    // -----------------------------------------------------------------
    // Simulation-only collision check (see policy note in the header).
    // Zero synthesis footprint.
    // -----------------------------------------------------------------
`ifndef SYNTHESIS
    a_no_rw_collision: assert property (
        @(posedge clk) !(rd_en && wr_en && (rd_addr == wr_addr))
    )
    else $error("%m: illegal same-address read/write collision at addr %0d, time %0t",
                rd_addr, $time);
`endif

endmodule : rgb_sram
