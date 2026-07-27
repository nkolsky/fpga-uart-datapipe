// mem_interlock.sv
// ----------------
// Stage 2C: minimal mutual exclusion between the image-read path and the
// single-pixel write path on the 100 MHz memory domain.
//
// This is NOT the full arbiter. There is no drain barrier, no CTS hold-off
// and no burst handling -- those belong to a later stage. It is the smallest
// structure that makes read/write exclusion a property of the design rather
// than an accident of UART timing.
//
// -----------------------------------------------------------------------
// WHAT IT PREVENTS
// -----------------------------------------------------------------------
// rgb_sram is a simple dual-port memory. A same-address access where one
// port writes yields INVALID READ DATA on 7-series block RAM -- the stored
// contents stay safe, but the outgoing image word is corrupted.
//
// Gating writes on !rom_seq_busy alone is not sufficient. rom_seq_busy
// asserts one cycle before rom_rd_en, but the write pipeline is two deep, so
// a pop issued in the same cycle a start arrives still lands its write while
// the first read is active:
//
//   cycle N    pop issued (rom_seq_busy still low), start_pulse arrives
//   cycle N+1  rom_seq_busy = 1, pop_q = 1
//   cycle N+2  rom_rd_en = 1  AND  wr_en = 1     <-- collision
//
// And this is not a rare address coincidence: rom_sequencer always begins at
// address 0, so a queued write to word 0 collides deterministically. The
// Stage 2C test writes pixel index 0, which is word 0.
//
// The realistic trigger is the host sending pixel writes during a capture.
// They queue (blocked by the interlock), the capture ends, the controller
// starts draining, and a new start arrives mid-drain.
//
// -----------------------------------------------------------------------
// HOW IT WORKS
// -----------------------------------------------------------------------
//   wr_pending  = commands queued OR a command in the write pipeline
//   read_go     = a start is pending AND nothing is writing
//   read_active = read_go OR rom_seq_busy
//   wr_allowed  = NOT read_active
//
// read_go can only fire when the write pipeline is completely empty, and
// once it fires wr_allowed drops immediately, so no new pop can begin. The
// write port is therefore provably quiet before rom_rd_en asserts.
//
// The read_go/rom_seq_busy handover is contiguous with no gap, because
// rom_sequencer registers busy from the NEXT state:
//
//     busy <= (next_state != IDLE);          // rom_sequencer.sv:203
//
// so busy is already high in the cycle after read_go. Were it registered
// from the CURRENT state there would be a one-cycle hole here needing an
// extra delay stage.
//
// -----------------------------------------------------------------------
// WHY rom_seq_busy CANNOT GATE read_go
// -----------------------------------------------------------------------
// rom_seq_busy tracks the ADDRESS WALK, not the transfer. It drops as soon
// as the last word is pushed, while up to 64 pixels remain in the pixel FIFO
// and tx_sequencer is still shifting them out -- roughly 1.4 ms of image
// still to go.
//
// Gating read_go on !rom_seq_busy would therefore launch a queued start the
// instant the walk ended, restarting rom_sequencer at address 0 while the
// previous image was still draining. The two images would interleave in the
// FIFO and both would be corrupted.
//
// Conversely, NOT gating it at all lets read_go assert while the sequencer
// is mid-walk: rom_sequencer ignores a start unless it is in IDLE, so the
// pulse does nothing, but start_pending is cleared anyway and the request is
// silently lost.
//
// Both failures come from the same mistake -- treating the address walk as
// the transfer. img_in_flight uses the real completion event instead:
// tx_img_done, recovered into this clock domain by the Change B toggle
// synchroniser.
//
// -----------------------------------------------------------------------
// DUPLICATE STARTS ARE REJECTED, NOT QUEUED
// -----------------------------------------------------------------------
// rgf.sv gates IMG_CTRL.start on (!complete && !error). Both are only SET
// at the end of a transfer by status_wen, so during a transfer they are
// still clear and the interlock PASSES -- a mid-transfer start really does
// reach this module as a start_req pulse. The RGF interlock guards the
// post-transfer retry, not the mid-transfer duplicate.
//
// Pre-Stage-2C that pulse went straight to rom_sequencer, which only
// samples `start` in IDLE, so it was silently ignored. That is the
// established Lab 10 semantic and it is preserved here: start_pending is
// only set when img_in_flight is low.
//
// Queueing such a start would be actively wrong. It would launch on
// tx_img_done -- the very event that SETS complete -- so the second
// transfer would run with complete set, having never been cleared by the
// host. That is exactly the condition the RGF interlock exists to forbid,
// and honouring it in the RGF while routing around it here would make the
// two disagree.
//
// start_pending therefore exists for one legitimate case only: a start
// that arrives with no transfer in flight but with pixel writes still
// draining. That request is genuinely deferred, not rejected, because
// start_req is a one-cycle pulse and would otherwise be lost.
//
// -----------------------------------------------------------------------
// WHY start_pending MUST BE STICKY
// -----------------------------------------------------------------------
// start_req is a single-cycle pulse from the RGF. If a start arrives while
// writes are draining, read_go cannot fire that cycle and an unlatched pulse
// would be lost outright -- the transfer would simply never happen and the
// host would sit waiting for a completion that never comes.
//
// -----------------------------------------------------------------------
// NO COMBINATIONAL LOOP
// -----------------------------------------------------------------------
// wr_busy comes only from registered state inside sram_wr_ctrl (pop_q,
// wr_en), never from its combinational cmd_rd_en. The cycle in which
// cmd_rd_en is asserted is covered here by !cmd_empty instead, since a pop
// can only be issued when the FIFO is non-empty. That breaks what would
// otherwise be a loop through wr_allowed.
//
// -----------------------------------------------------------------------
// LIVENESS
// -----------------------------------------------------------------------
// Writes drain at one command per clock, so wr_pending clears within at most
// DEPTH+2 cycles and a pending read always proceeds. A read always completes
// and releases wr_allowed. Neither side can deadlock.
//
// Writes have priority over a pending read, so a saturating write stream
// could in principle starve reads. It cannot happen at UART rates -- one
// command per ~15 us against a one-cycle drain -- and the later arbiter
// addresses it properly with a drain barrier.

`timescale 1ns/1ps

module mem_interlock (
    input  logic clk,
    input  logic rst_n,

    // ---- read side -----------------------------------------------------
    input  logic start_req,      // one-cycle start pulse from the RGF
    input  logic rom_seq_busy,   // rom_sequencer address walk active
    input  logic img_done,       // tx_img_done, recovered on this clock
    output logic read_go,        // -> rom_sequencer.start

    // ---- write side ----------------------------------------------------
    input  logic cmd_empty,      // command FIFO, 100 MHz read side
    input  logic wr_busy,        // sram_wr_ctrl pipeline occupied
    output logic wr_allowed      // -> sram_wr_ctrl
);

    logic start_pending;
    logic img_in_flight;
    logic wr_pending;
    logic read_active;

    assign wr_pending  = !cmd_empty || wr_busy;

    // A read may launch only when the PREVIOUS transfer is completely done
    // and nothing is writing. See the header for why rom_seq_busy is not a
    // sufficient completion signal.
    assign read_go     = start_pending && !wr_pending && !img_in_flight;

    // Collision gate. Deliberately a DIFFERENT condition from read_go: once
    // rom_sequencer has read a word its data is already safe in the pixel
    // FIFO, so writes only have to stand off for the address walk, not for
    // the ~1.5 s of transmission that follows.
    assign read_active = read_go || rom_seq_busy;
    assign wr_allowed  = !read_active;

    // -----------------------------------------------------------------
    // Transfer-in-flight tracking.
    //
    // Set when a read launches, cleared by tx_img_done recovered into this
    // domain by cdc_pulse_sync (Change B). This is what holds a queued start
    // across the whole transfer rather than only across the address walk.
    // -----------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)         img_in_flight <= 1'b0;
        else if (read_go)   img_in_flight <= 1'b1;
        else if (img_done)  img_in_flight <= 1'b0;
    end

    // -----------------------------------------------------------------
    // Sticky start request.
    //
    // A start is latched ONLY when no transfer is in flight AND none is
    // being launched this cycle. A duplicate arriving mid-transfer is
    // REJECTED, not queued -- see the protocol note above.
    //
    // The !read_go term closes a one-cycle hole: img_in_flight is only set
    // at the edge read_go fires, so without it a start_req arriving in that
    // exact cycle would still be latched and then deferred. That needs two
    // start pulses 10 ns apart and cannot happen at UART rates, but the
    // guarantee is worth having structurally rather than by argument.
    //
    // Set takes priority over clear so a start arriving in the same cycle
    // read_go fires is not lost. read_go is one cycle wide because
    // start_pending clears at the end of the cycle it asserts.
    // -----------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)                           start_pending <= 1'b0;
        else if (start_req && !img_in_flight && !read_go)
                                              start_pending <= 1'b1;
        else if (read_go)                     start_pending <= 1'b0;
    end

`ifndef SYNTHESIS
    // The core mutual-exclusion property.
    a_excl: assert property (
        @(posedge clk) disable iff (!rst_n)
        !(wr_allowed && rom_seq_busy)
    ) else $error("%m: writes allowed while rom_sequencer is reading");

    // A read may only launch with the write pipeline completely idle.
    a_read_needs_idle: assert property (
        @(posedge clk) disable iff (!rst_n)
        read_go |-> (cmd_empty && !wr_busy)
    ) else $error("%m: read_go asserted with writes still pending");

    // read_go must never coincide with an active address walk. If it did,
    // rom_sequencer would ignore the start (it only samples `start` in IDLE)
    // and start_pending would be cleared for nothing -- the request would be
    // silently lost. img_in_flight is what makes this unreachable.
    a_no_go_while_busy: assert property (
        @(posedge clk) disable iff (!rst_n)
        !(read_go && rom_seq_busy)
    ) else $error("%m: read_go asserted while rom_sequencer was already busy");

    // A duplicate start arriving mid-transfer must leave no trace: it is
    // rejected on arrival, never stored for later execution.
    a_duplicate_start_rejected: assert property (
        @(posedge clk) disable iff (!rst_n)
        (start_req && (img_in_flight || read_go)) |=> !start_pending
    ) else $error("%m: mid-transfer duplicate start was queued instead of rejected");
`endif

endmodule : mem_interlock
