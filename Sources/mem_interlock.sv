// -----------------------------------------------------------------------------
// mem_interlock.sv
//
// Memory-port arbiter for the 100 MHz SRAM domain. The read sequencer, the
// write path, and the shared read-borrower all funnel through here so the SRAM
// never sees a read and a write to the same address in one cycle.
//
// The key rule is simple: a read can start only when no write is active, and a
// burst write keeps a new read deferred until the write traffic is done.
// -----------------------------------------------------------------------------

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
    output logic wr_port_grant,  // -> mem_write_subsystem.wr_allowed,
                                 //    i.e. pixel_word_packer's wr_ready

    // -----------------------------------------------------------------
    // READ-PORT BORROWER: third client of the memory. It borrows the SRAM
    // read port, so it must exclude both the image reader and the writer.
    //
    // The borrower request is a shared one: pixel_rd_ctrl and burst_rd_ctrl
    // are arbitrated upstream and presented here as a single read-borrower
    // request. This block only cares that the port is free and that the request
    // stays active until the borrower is done.
    //
    // The request is a level, held by the borrower until it is granted. Once a
    // grant is latched, it blocks both read_go and wr_port_grant until the
    // borrower clears its done signal.
    // -----------------------------------------------------------------
    input  logic pix_rd_req,
    input  logic pix_rd_done,
    output logic pix_rd_gnt,

    // Ownership of the SRAM READ PORT, for memory_subsystem's read mux.
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

    // The read-port mux in memory_subsystem follows this exactly.
    assign pix_rd_owner = pix_rd_active;
    logic wr_pending;
    logic read_active;

    // burst_active is included so a read cannot slip into the QUIET GAPS
    // BETWEEN BURST DATA MESSAGES. Those gaps are ~2816 receive-clock cycles
    // long while an update completes in a handful, so wr_port_req really
    // does go low between frames -- without this term a start arriving
    // mid-burst would launch rom_sequencer with half an image written.
    //
    // wr_busy is what protects an update in progress: while a rectangle is
    // open in pixel_word_packer -- pixels absorbed but the accumulated word
    // not yet issued -- nothing else may take the port.
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
