// mem_interlock.sv
// ----------------
// Sole arbiter of the three channel SRAMs on the 100 MHz memory domain.
// Every access to that memory is granted here; nothing reaches it any
// other way.
//
// This began as a minimal read/write interlock and has since grown to
// cover every client. It now arbitrates THREE:
//
//   1. Full-frame image read   rom_sequencer, via read_go.
//                              Owns the read port for a whole frame.
//   2. Write path              sram_rmw, via wr_port_grant. Serves BOTH
//                              Single Pixel Write and Image Burst Write --
//                              they are the same commands arriving through
//                              the same 48-bit FIFO, so there is no
//                              separate burst-write client here.
//   3. Read-port borrower      pixel_rd_ctrl OR burst_rd_ctrl, via
//                              pix_rd_req / pix_rd_gnt / pix_rd_done.
//                              These two are arbitrated against each other
//                              in chip_top BEFORE this module sees them,
//                              and arrive as one shared request. The port
//                              names still say "pix" for that reason -- see
//                              the port comments below.
//
// Burst handling IS present: burst_active closes the inter-frame gaps in an
// Image Burst Write (see the STAGE 3 / M4 section below). What is still
// absent is a drain barrier and CTS hold-off -- writes retain priority over
// a pending read, so a saturating write stream could in principle starve
// reads. See LIVENESS at the end for why that cannot happen at UART rates.
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
//   wr_port_grant  = NOT read_active
//
// read_go can only fire when the write pipeline is completely empty, and
// once it fires wr_port_grant drops immediately, so no new pop can begin. The
// write port is therefore provably quiet before rom_rd_en asserts.
//
// The read_go/rom_seq_busy handover is contiguous with no gap, because
// rom_sequencer registers busy from the NEXT state:
//
//     busy <= (next_state != IDLE);   // rom_sequencer.sv, final always_ff
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
// a request is asserted is covered here by wr_port_req instead, since one
// can only be issued when the FIFO is non-empty. That breaks what would
// otherwise be a loop through wr_port_grant.
//
// -----------------------------------------------------------------------
// STAGE 3 / M4: burst_active
// -----------------------------------------------------------------------
// An Image Burst Write arrives as many separate 16-byte frames. Each frame
// yields four commands that the write path drains in a handful of cycles,
// then ~2816 receive-clock cycles pass before the next frame. So wr_port_req
// genuinely goes HIGH between frames, and wr_busy with it.
//
// Without burst_active, a start_req arriving in one of those gaps would see
// wr_pending low, fire read_go, and launch rom_sequencer over an image that
// is only half written -- producing a torn capture with no error anywhere.
//
// burst_active closes the gaps. It appears ONLY in wr_pending, which gates
// the start of a READ. It deliberately does NOT appear in read_active, which
// gates writes: including it there would make a burst block its own writes
// and deadlock immediately.
//
// -----------------------------------------------------------------------
// LIVENESS
// -----------------------------------------------------------------------
// Writes drain at one command per clock, so wr_pending clears within at most
// DEPTH+2 cycles and a pending read always proceeds. A read always completes
// and releases wr_port_grant. Neither side can deadlock.
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
    //
    // Writes do NOT need the read port. rgb_sram has a per-byte write
    // enable, so changing a single pixel is one write with three lanes
    // masked off -- no read, and no read-modify-write.
    //
    // They still stand off while a reader owns the memory, because rgb_sram
    // forbids a read and a write to the same address in one cycle and the
    // readers walk the whole image.
    input  logic wr_port_req,    // the write path has a word to place
    input  logic wr_busy,        // a rectangle is in progress
    // A burst is in progress on the receive side, already synchronised into
    // this clock domain. See the note below for why a request signal alone
    // is not sufficient.
    input  logic burst_active,
    output logic wr_port_grant,  // -> sram_rmw.port_grant

    // -----------------------------------------------------------------
    // READ-PORT BORROWER: third client of the memory. It borrows the SRAM
    // READ port, so it must exclude both the image reader (same port) and
    // the writer (same memory).
    //
    // THE "pix" PREFIX IS HISTORICAL. This port pair was added for
    // pixel_rd_ctrl alone, but burst_rd_ctrl is now a second borrower with
    // identical needs. The two are arbitrated against each other inside
    // chip_top and presented here as ONE shared request -- see
    // shared_rd_req / shared_rd_gnt / shared_rd_done at the instantiation.
    // This module deliberately does not know which of the two it is
    // granting; from here they are one client, which is why no fourth
    // port pair was added.
    //
    // Duration differs sharply between them and that is fine: a single
    // pixel read holds the port for a handful of cycles, while a Burst
    // Read holds it for the whole rectangle -- deliberately, so the
    // returned region is a coherent snapshot (see burst_rd_ctrl.sv). Both
    // are just "granted until done" from this module's point of view.
    //
    // The request is a LEVEL, held by the borrower until granted. A grant
    // latches pix_rd_active, which blocks read_go and wr_port_grant until
    // done is asserted.
    // -----------------------------------------------------------------
    input  logic pix_rd_req,
    input  logic pix_rd_done,
    output logic pix_rd_gnt,

    // Ownership of the SRAM READ PORT, for the chip_top read mux.
    //
    // Exported rather than recomputed at the mux, so that the arbiter and
    // the datapath cannot disagree about who owns the port. This is the
    // same registered pix_rd_active that gates read_go and wr_port_grant
    // below -- one source of truth, three consumers.
    output logic pix_rd_owner
);

    logic start_pending;
    logic img_in_flight;
    logic pix_rd_active;

    // The read-port mux in chip_top follows this exactly.
    assign pix_rd_owner = pix_rd_active;
    logic wr_pending;
    logic read_active;

    // burst_active is included so a read cannot slip into the QUIET GAPS
    // BETWEEN BURST DATA MESSAGES. Those gaps are ~2816 receive-clock cycles
    // long while an update completes in a handful, so wr_port_req really
    // does go low between frames -- without this term a start arriving
    // mid-burst would launch rom_sequencer with half an image written.
    //
    // wr_busy is what protects an update in progress: while sram_rmw is
    // between its read and its write, nothing else may take the port.
    assign wr_pending  = wr_port_req || wr_busy || burst_active;

    // A read may launch only when the PREVIOUS transfer is completely done
    // and nothing is writing. See the header for why rom_seq_busy is not a
    // sufficient completion signal.
    assign read_go     = start_pending && !wr_pending && !img_in_flight
                                        && !pix_rd_active && !pix_rd_gnt;

    // Collision gate. Deliberately a DIFFERENT condition from read_go: once
    // rom_sequencer has read a word its data is already safe in the pixel
    // FIFO, so writes only have to stand off for the address walk, not for
    // the ~1.5 s of transmission that follows.
    assign read_active = read_go || rom_seq_busy;

    // The write path stands off while either reader owns the memory. The
    // ports are physically independent, but rgb_sram's own assertion forbids
    // a read and a write to the same address in one cycle, and a reader
    // walking the image would eventually collide.
    assign wr_port_grant = !read_active && !pix_rd_active;

    // Granted only when the image reader is idle AND nothing is writing or
    // waiting to write. wr_pending covers a pending request, an update in
    // progress and an active burst, so this is one term, not three.
    // Qualified on rom_seq_busy and img_in_flight rather than read_active,
    // which would close a combinational loop:
    //     read_go -> read_active -> pix_rd_gnt -> read_go
    // Both terms are registered, so the grant is loop-free. A same-cycle tie
    // is broken in favour of the pixel read, which finishes in a few cycles;
    // read_go carries !pix_rd_gnt and simply waits.
    assign pix_rd_gnt  = pix_rd_req && !rom_seq_busy && !img_in_flight
                                    && !wr_pending && !pix_rd_active;

    // A single-pixel read in progress also blocks a new image transfer, so
    // the two can never contend for the read port.
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)            pix_rd_active <= 1'b0;
        else if (pix_rd_gnt)   pix_rd_active <= 1'b1;
        else if (pix_rd_done)  pix_rd_active <= 1'b0;
    end

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
    // Only one client may own the port at a time.
    a_one_owner: assert property (
        @(posedge clk) disable iff (!rst_n)
        $onehot0({wr_port_grant && wr_port_req, read_active, pix_rd_active})
    ) else $error("%m: more than one client owns the memory port");

    // The core mutual-exclusion property.
    a_excl: assert property (
        @(posedge clk) disable iff (!rst_n)
        !(wr_port_grant && rom_seq_busy)
    ) else $error("%m: write granted the port while rom_sequencer is reading");

    // A read may only launch with the write pipeline completely idle.
    a_read_needs_idle: assert property (
        @(posedge clk) disable iff (!rst_n)
        read_go |-> (!wr_port_req && !wr_busy && !burst_active)
    ) else $error("%m: read_go asserted with writes pending or a burst active");

    // A burst must never be interrupted by an image read.
    // The image reader and the single-pixel reader can never both own the
    // SRAM read port.
    a_read_port_exclusive: assert property (
        @(posedge clk) disable iff (!rst_n)
        !(pix_rd_active && (read_go || rom_seq_busy))
    ) else $error("%m: single-pixel read and image read own the port together");

    // No write may proceed while a single-pixel read holds the memory.
    a_no_write_during_pix_rd: assert property (
        @(posedge clk) disable iff (!rst_n)
        pix_rd_active |-> !wr_port_grant
    ) else $error("%m: write allowed during a single-pixel read");

    // A grant is never manufactured without a request.
    a_gnt_implies_req: assert property (
        @(posedge clk) disable iff (!rst_n)
        pix_rd_gnt |-> pix_rd_req
    ) else $error("%m: pixel read granted without a request");

    // A grant is only issued into a genuinely quiet memory.
    a_gnt_needs_idle: assert property (
        @(posedge clk) disable iff (!rst_n)
        pix_rd_gnt |-> (!rom_seq_busy && !img_in_flight &&
                        !wr_port_req && !wr_busy && !burst_active)
    ) else $error("%m: pixel read granted with the memory in use");

    // Ownership is never asserted for more than one transaction at a time:
    // a second grant cannot arrive while the first is still held.
    a_no_double_grant: assert property (
        @(posedge clk) disable iff (!rst_n)
        pix_rd_active |-> !pix_rd_gnt
    ) else $error("%m: pixel read granted while one was already active");

    a_no_read_during_burst: assert property (
        @(posedge clk) disable iff (!rst_n)
        burst_active |-> !read_go
    ) else $error("%m: read_go asserted during an active burst");

    // ...but a burst must not block its own writes.
    //
    // The antecedent carries !pix_rd_active as well as !read_go and
    // !rom_seq_busy. wr_port_grant now has three blockers, not two, so
    // without this term the property asserts something that is simply
    // untrue: a single-pixel read that was granted BEFORE the burst began
    // legitimately holds the memory for a few more cycles after
    // burst_active rises, and writes are correctly blocked for that
    // window. The grant condition itself carries !wr_pending, which
    // includes burst_active, so a pixel read can never START during a
    // burst -- but one already in progress can still overlap the start of
    // one, and that overlap is legal.
    a_burst_may_write: assert property (
        @(posedge clk) disable iff (!rst_n)
        (burst_active && !read_go && !rom_seq_busy && !pix_rd_active)
            |-> wr_port_grant
    ) else $error("%m: writes blocked during a burst with no read in progress");

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
