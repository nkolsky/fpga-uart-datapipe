// rx_burst_ctrl.sv
// ----------------
// Stage 3, M3: Image Burst Write controller.
//
// Consumes a validated burst header and a stream of validated burst data
// frames, and emits ordinary Stage 2C {address, pixel} write commands -- one
// per clock -- into the existing 48-bit command FIFO. Nothing downstream of
// the FIFO changes: sram_wr_ctrl, mem_interlock, rgb_sram and the whole
// hardware-validated Stage 2C write path are reused untouched.
//
// =======================================================================
// COMMAND OUTPUT TIMING
// =======================================================================
// cmd_valid is REGISTERED and one cycle wide, with cmd_addr/cmd_pixel valid
// in the same cycle -- deliberately the same shape as rx_classifier's
// cmd_valid, so chip_top can mux the two sources with a plain 2:1 select and
// no re-timing.
//
//   cycle  state    slot  action at the edge                cmd_valid during
//   N      B_ACTIVE   -    latch 4 pixels, slot<=0              0
//   N+1    B_EMIT     0    load cmd(slot0), slot<=1             0
//   N+2    B_EMIT     1    accept slot0, load slot1, slot<=2    1  <- slot 0
//   N+3    B_EMIT     2    accept slot1, load slot2, slot<=3    1  <- slot 1
//   N+4    B_EMIT     3    accept slot2, load slot3, slot<=4    1  <- slot 2
//   N+5    B_EMIT     4    accept slot3, walk complete -> exit  1  <- slot 3
//   N+6    B_ACTIVE   -                                         0
//        or B_DONE
//
// cmd_valid still occupies N+2..N+5 exactly as before, so the interface
// contract chip_top depends on is unchanged.
//
// With cmd_ready low at any point the presented command is simply held and
// every later cycle shifts; nothing is skipped and nothing is withdrawn.
//
// =======================================================================
// WHY THE STATE EXIT WAITS FOR slot == BURST_PIX_PER_MSG
// =======================================================================
// The exit condition is the completion of the slot WALK, evaluated under
// cmd_slot_free -- not the loading of the last slot. That distinction is
// what makes the final command safe.
//
// Exiting on the load would allow:
//
//   cycle  slot  action                     cmd_valid  state     bypass
//   T       3    load final cmd, -> B_DONE      1       B_EMIT      1
//   T+1     -    cmd_ready low, held           1       B_DONE      1
//   T+2     -    still held                    1       B_IDLE      0   <-- !
//
// At T+2 the command is still on the interface but bypass_active has gone
// low, so chip_top's command-source mux would have switched away from this
// module and withdrawn the command before the FIFO ever accepted it. The
// pixel would vanish with no error anywhere, and burst_done would already
// have pulsed at T+1 claiming the burst was complete.
//
// Exiting from slot == BURST_PIX_PER_MSG under cmd_slot_free guarantees the
// final command has FIRED, so:
//
//   * bypass_active and burst_active stay high for the whole hold;
//   * burst_done means every real command has been accepted by the FIFO,
//     not merely presented.
//
// First command two cycles after msg_valid; four commands on four
// consecutive cycles. A frame is fully consumed in five cycles against ~176
// cycles between frames at 8.125 Mbaud, so B_EMIT is never still busy when
// the next frame arrives.
//
// =======================================================================
// BACKPRESSURE -- STANDARD READY/VALID
// =======================================================================
// cmd_ready is !cmd_fifo_full. The interface obeys ordinary ready/valid
// semantics:
//
//   * a command is PRESENTED by loading cmd_addr/cmd_pixel and raising
//     cmd_valid;
//   * it is HELD, bit-stable, until cmd_ready is also high;
//   * the TRANSFER occurs on (cmd_valid && cmd_ready);
//   * the slot counter and the row/column position advance ONLY when the
//     command that was presented is loaded, and a new command is only
//     loaded when the output register is free or being freed this cycle.
//
// This matters because of a near-full race. async_fifo drops a write
// silently when full:
//
//     if (wr_en && !full) fifo_mem[...] <= wr_data;
//
// and full is combinational off the write pointer. An earlier design that
// sampled cmd_ready at the moment it REGISTERED a command would advance past
// a command that the FIFO then refused one cycle later:
//
//   cycle  occupancy  full  cmd_valid   controller           FIFO
//   T-1       62       0    1 (A)       ready -> load B      writes A -> 63
//   T         63       0    1 (B)       ready -> load C      writes B -> 64
//   T+1       64       1    1 (C)       stalls               C DROPPED
//
// The command presented at T+1 is rejected, but the controller has already
// advanced past it. No error, no counter, nothing -- a silently lost pixel.
// Holding until accepted removes the race entirely: the position cannot run
// ahead of what the FIFO has taken.
//
// This condition is not hypothetical. It arises whenever mem_interlock
// blocks writes during an image read, which is precisely what burst write
// makes possible.
//
// CTS remains the protocol-level mechanism that stops the host in the first
// place; this is the local guard that makes losing a pixel impossible rather
// than merely unlikely.
//
// =======================================================================
// CONFIGURABLE ORIGIN
// =======================================================================
// The burst origin is held in base_row / base_col, loaded on header accept.
// This version loads zero, because the spec's group A for a burst write is
// three don't-care bytes and pixel 0 is the only reading consistent with
// that (assumption A1).
//
// Addresses are generated RELATIVE to the base:
//
//     pixel_row  = base_row + burst_row
//     pixel_col  = base_col + burst_col
//     pixel_addr = pixel_row * IMG_W + pixel_col
//
// so if the origin later becomes a header field, only the base-load logic at
// the marked point changes -- not the FSM, the counters, the serializer or
// anything downstream. For IMG_W = 256 the multiply reduces to concatenation
// and costs no hardware.
//
// NOTE: with base = 0 the region is guaranteed to fit, because
// rx_burst_hdr_parser already validates height <= IMG_H and width <= IMG_W.
// When the base becomes non-zero a base+extent check must be added to the
// header-accept condition below -- the point is marked. Writing that check
// now would be dead logic, so it is documented rather than coded.
//
// =======================================================================
// RECTANGLE SEMANTICS
// =======================================================================
// height and width describe a row-major rectangle, not a linear run. H=2,
// W=3 emits (0,0) (0,1) (0,2) (1,0) (1,1) (1,2) -- addresses 0,1,2 then
// 256,257,258 for a 256-wide image, NOT 0..5.
//
// Completion is tracked by POSITION, not by counting to H*W. Comparing
// against h_last/w_last avoids a 24x24 multiplier entirely.
//
// =======================================================================
// RECOVERY
// =======================================================================
// Every exit funnels through one term so a timeout or an explicit abort
// command can be added later without restructuring the FSM:
//
//     if (burst_abort) state <= B_IDLE;
//
// burst_abort is a real port, expected to be tied inactive for now, leaving
// reset as the only external recovery path -- the agreed first-implementation
// scope. Later it becomes (timeout || abort_command || ...) with no change
// here.

`timescale 1ns/1ps

module rx_burst_ctrl
    import memory_pkg::*;
    import rx_msg_pkg::*;
    import rx_burst_pkg::*;
#(
    parameter int IMG_W = IMG_WIDTH,    // 256
    parameter int IMG_H = IMG_HEIGHT    // 256
)(
    input  logic clk,
    input  logic rst_n,

    // ---- from rx_mac ---------------------------------------------------
    input  logic      msg_valid,     // one-cycle pulse, frame complete
    input  msg_kind_t msg_kind,      // rx_mac's LATCHED msg_kind_q

    // ---- from rx_burst_hdr_parser (combinational on the same frame) ----
    input  logic                   hdr_valid,
    input  logic [BURST_DIM_W-1:0] height,
    input  logic [BURST_DIM_W-1:0] width,

    // ---- from rx_burst_data_parser (combinational on the same frame) ---
    input  logic                                          data_frame_ok,
    input  logic [BURST_PIX_PER_MSG-1:0][BURST_PIX_W-1:0] pixels,

    // ---- recovery ------------------------------------------------------
    input  logic burst_abort,

    // ---- command FIFO write side --------------------------------------
    input  logic                    cmd_ready,   // !cmd_fifo_full
    output logic                    cmd_valid,   // one-cycle pulse
    output logic [BURST_ADDR_W-1:0] cmd_addr,
    output logic [BURST_PIX_W-1:0]  cmd_pixel,

    // ---- mode ----------------------------------------------------------
    output logic bypass_active,   // -> rx_msg_decode  (this clock domain)
    output logic burst_active,    // -> mem_interlock  (needs synchronising)

    // ---- status --------------------------------------------------------
    output logic burst_done,        // one cycle, normal completion
    output logic err_hdr_invalid,   // header arrived but failed validation
    output logic err_data_invalid,  // data frame arrived but was malformed
    output logic err_unexpected     // traffic arrived in the wrong state
);

    localparam int ROW_W     = $clog2(IMG_H);            // 8
    localparam int COL_W     = $clog2(IMG_W);            // 8
    localparam int LAST_SLOT = BURST_PIX_PER_MSG - 1;    // 3

    // -----------------------------------------------------------------
    // State
    // -----------------------------------------------------------------
    burst_state_t state;

    logic [ROW_W-1:0] base_row, burst_row, h_last;
    logic [COL_W-1:0] base_col, burst_col, w_last;

    // One bit wider than the slot index so the counter can reach
    // BURST_PIX_PER_MSG, giving "walk complete" its own distinct value.
    // Exiting B_EMIT from that value -- rather than from the last slot --
    // is what guarantees the final command has been ACCEPTED, not merely
    // loaded, before the mode outputs drop.
    logic [BURST_SLOT_W:0] slot;
    logic [BURST_PIX_PER_MSG-1:0][BURST_PIX_W-1:0] px_latch;

    // Set once every real pixel of the rectangle has been emitted. Any
    // remaining slots in the current frame are host padding and are skipped.
    logic done_flag;

    // -----------------------------------------------------------------
    // Address generation, relative to the configurable origin.
    //
    // For IMG_W = 256 the multiply is a shift and synthesises to
    // concatenation: {pixel_row, pixel_col}. Written as arithmetic so a
    // non-power-of-two width still produces correct hardware.
    // -----------------------------------------------------------------
    logic [ROW_W-1:0]        pixel_row;
    logic [COL_W-1:0]        pixel_col;
    logic [BURST_ADDR_W-1:0] row_term;
    logic [BURST_ADDR_W-1:0] pixel_addr;

    assign pixel_row = base_row + burst_row;
    assign pixel_col = base_col + burst_col;

    // Computed entirely in BURST_ADDR_W so the arithmetic never promotes to
    // 32-bit int. For IMG_W = 256 the multiply is a shift and the whole
    // expression collapses to {pixel_row, pixel_col}.
    assign row_term   = BURST_ADDR_W'(pixel_row) * BURST_ADDR_W'(IMG_W);
    assign pixel_addr = row_term + BURST_ADDR_W'(pixel_col);

    // -----------------------------------------------------------------
    // Position predicates
    // -----------------------------------------------------------------
    logic at_last_col, at_last_pixel;

    assign at_last_col   = (burst_col == w_last);
    assign at_last_pixel = (burst_row == h_last) && at_last_col;

    // -----------------------------------------------------------------
    // Frame acceptance
    //
    // A header is accepted ONLY in B_IDLE. Once bypass_active is asserted,
    // rx_msg_decode classifies every frame as MSG_BURST_DATA, so a header
    // frame arriving mid-burst is genuinely pixel data by the spec's own
    // opcode-bypass rule. The msg_kind == MSG_BURST_HDR test below is
    // therefore defensive rather than load-bearing.
    // -----------------------------------------------------------------
    logic hdr_seen, data_seen;

    assign hdr_seen  = msg_valid && (msg_kind == MSG_BURST_HDR);
    assign data_seen = msg_valid && (msg_kind == MSG_BURST_DATA);

    // -----------------------------------------------------------------
    // Handshake. A command transfers only when both sides agree.
    // -----------------------------------------------------------------
    logic cmd_fire;
    assign cmd_fire = cmd_valid && cmd_ready;

    // The output register is available when nothing is pending, or when the
    // pending command is being accepted this very cycle -- which is what
    // allows one command per clock with no bubble.
    logic cmd_slot_free;
    assign cmd_slot_free = !cmd_valid || cmd_fire;

    // -----------------------------------------------------------------
    // Mode outputs. Currently identical; kept as separate ports because
    // they cross into different domains and may need to diverge.
    //
    // burst_active deliberately does NOT extend to cover a command still
    // pending in the output register. The controller only holds a command
    // when cmd_ready is low, i.e. when the FIFO is FULL -- and a full FIFO
    // is necessarily non-empty, so mem_interlock's wr_pending term
    // (!cmd_empty) already blocks a read for exactly that window.
    // -----------------------------------------------------------------
    assign bypass_active = (state != B_IDLE);
    assign burst_active  = (state != B_IDLE);
    assign burst_done    = (state == B_DONE);

    // -----------------------------------------------------------------
    // Main sequential block
    // -----------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state            <= B_IDLE;
            base_row         <= '0;
            base_col         <= '0;
            burst_row        <= '0;
            burst_col        <= '0;
            h_last           <= '0;
            w_last           <= '0;
            slot             <= '0;
            px_latch         <= '0;
            done_flag        <= 1'b0;
            cmd_valid        <= 1'b0;
            cmd_addr         <= '0;
            cmd_pixel        <= '0;
            err_hdr_invalid  <= 1'b0;
            err_data_invalid <= 1'b0;
            err_unexpected   <= 1'b0;
        end
        else begin
            // Status strobes default low; asserted for one cycle below.
            err_hdr_invalid  <= 1'b0;
            err_data_invalid <= 1'b0;
            err_unexpected   <= 1'b0;

            // -----------------------------------------------------------
            // Ready/valid: a presented command is cleared ONLY when it is
            // accepted. It is never withdrawn, never modified while
            // pending, and never skipped. B_EMIT below may re-raise it in
            // the same cycle to present the next slot with no bubble.
            // -----------------------------------------------------------
            if (cmd_fire) cmd_valid <= 1'b0;

            // -----------------------------------------------------------
            // Recovery. Highest priority, effective from any state.
            //
            // A presented command IS withdrawn here, and this is the one
            // deliberate exception to the hold rule. Leaving cmd_valid
            // asserted while bypass_active drops would strand a command on
            // an interface that chip_top has already muxed away from --
            // exactly the withdrawal race this module exists to avoid.
            // Abort is a recovery action, so a clean withdrawal is both
            // safer and the conventional handshake exception, alongside
            // reset. Nothing downstream ever sees a half-transferred
            // command.
            // -----------------------------------------------------------
            if (burst_abort) begin
                state     <= B_IDLE;
                cmd_valid <= 1'b0;
            end
            else begin
                unique case (state)

                // -------------------------------------------------------
                // B_IDLE -- not in a burst. bypass_active is low, so frames
                // are opcode-decoded normally and a header can be seen.
                // -------------------------------------------------------
                B_IDLE: begin
                    if (hdr_seen) begin
                        if (hdr_valid) begin
                            // ORIGIN LOAD. This is the ONLY place the base
                            // is set. When the spec's group A turns out to
                            // carry a start address, it is loaded here --
                            // and the base+extent range check belongs here
                            // too. Nothing else in this module changes.
                            base_row  <= '0;
                            base_col  <= '0;

                            // Store the LAST index rather than the count, so
                            // the comparisons stay narrow. height and width
                            // are already validated in [1, IMG_H/IMG_W], so
                            // height-1 fits in ROW_W bits.
                            // Subtract in the field width, then narrow.
                            // height/width are validated in [1, IMG_H/IMG_W],
                            // so the decremented value always fits.
                            h_last    <= ROW_W'(height - BURST_DIM_W'(1));
                            w_last    <= COL_W'(width  - BURST_DIM_W'(1));

                            burst_row <= '0;
                            burst_col <= '0;
                            done_flag <= 1'b0;
                            state     <= B_ACTIVE;
                        end
                        else begin
                            err_hdr_invalid <= 1'b1;   // stay idle
                        end
                    end
                    else if (data_seen) begin
                        // Burst data with no burst in progress.
                        err_unexpected <= 1'b1;
                    end
                end

                // -------------------------------------------------------
                // B_ACTIVE -- bypass on, waiting for the next data frame.
                // -------------------------------------------------------
                B_ACTIVE: begin
                    if (data_seen) begin
                        if (data_frame_ok) begin
                            px_latch <= pixels;
                            slot     <= '0;
                            state    <= B_EMIT;
                        end
                        else begin
                            // Malformed frame: no commands, position and
                            // counters untouched, burst stays armed.
                            err_data_invalid <= 1'b1;
                        end
                    end
                    else if (hdr_seen) begin
                        // Defensive: cannot occur while bypass is on.
                        err_unexpected <= 1'b1;
                    end
                end

                // -------------------------------------------------------
                // B_EMIT -- serialise up to four pixels, one per clock.
                //
                // All four slots are visited uniformly. Slots at or beyond
                // completion are host padding: they consume a cycle but
                // emit nothing and do not advance the position.
                // -------------------------------------------------------
                B_EMIT: begin
                    if (msg_valid) begin
                        // A frame arrived while still serialising. Cannot
                        // happen at UART rates (5 cycles of work against
                        // ~176 between frames) but must not be silent.
                        err_unexpected <= 1'b1;
                    end

                    // Only touch anything when the output register is free
                    // or is being freed by an accept this cycle. Otherwise
                    // hold: cmd_valid stays high, cmd_addr/cmd_pixel stay
                    // bit-stable, and slot/row/column do not move. This is
                    // what stops the position running ahead of what the
                    // FIFO has actually taken.
                    if (cmd_slot_free) begin
                        if (slot == (BURST_SLOT_W+1)'(BURST_PIX_PER_MSG)) begin
                            // All four slots processed AND nothing pending
                            // -- cmd_slot_free guarantees the last command
                            // either never existed or is firing this cycle.
                            // Only now may the mode outputs drop.
                            state <= done_flag ? B_DONE : B_ACTIVE;
                        end
                        else if (done_flag) begin
                            // Padding slot -- consumes a cycle, emits
                            // nothing, does not advance the position.
                            slot <= slot + 1'b1;
                        end
                        else begin
                            // Present this slot. The address is captured
                            // BEFORE the position advances, so the held
                            // command always describes the pixel it was
                            // loaded for even though the counters move on.
                            cmd_valid <= 1'b1;
                            cmd_addr  <= pixel_addr;
                            cmd_pixel <= px_latch[slot[BURST_SLOT_W-1:0]];

                            if (at_last_pixel) begin
                                done_flag <= 1'b1;
                            end
                            else if (at_last_col) begin
                                burst_col <= '0;
                                burst_row <= burst_row + 1'b1;
                            end
                            else begin
                                burst_col <= burst_col + 1'b1;
                            end

                            // Advance only. The state exit is decided at
                            // slot == BURST_PIX_PER_MSG above, one cycle
                            // later, by which time this command has fired.
                            slot <= slot + 1'b1;
                        end
                    end
                    // else: pending command not yet accepted -- hold.
                end

                // -------------------------------------------------------
                // B_DONE -- one cycle. burst_done pulses here and the mode
                // outputs drop as the state returns to idle.
                // -------------------------------------------------------
                B_DONE: begin
                    state <= B_IDLE;
                end

                default: state <= B_IDLE;
                endcase
            end
        end
    end

    // -----------------------------------------------------------------
    // Simulation-only invariants
    // -----------------------------------------------------------------
`ifndef SYNTHESIS
    // THE HANDSHAKE CONTRACT. A presented command must stay asserted and
    // bit-stable until it is accepted -- never withdrawn, never modified.
    a_hold_when_not_ready: assert property (
        @(posedge clk) disable iff (!rst_n)
        (cmd_valid && !cmd_ready && !burst_abort) |=>
            (cmd_valid && $stable(cmd_addr) && $stable(cmd_pixel))
    ) else $error("%m: a pending command was withdrawn or modified");

    // The position may only advance on a load, which may only happen when
    // the output register is free -- so it can never run ahead of the FIFO.
    a_no_load_while_pending: assert property (
        @(posedge clk) disable iff (!rst_n)
        (cmd_valid && !cmd_ready && !burst_abort) |=> $stable(slot)
    ) else $error("%m: slot advanced while a command was still pending");

    // burst_done means EVERY real command has been accepted by the FIFO,
    // not merely presented. B_DONE is unreachable with a command pending.
    a_done_means_accepted: assert property (
        @(posedge clk) disable iff (!rst_n)
        (state == B_DONE) |-> !cmd_valid
    ) else $error("%m: burst_done pulsed with a command still pending");

    // The mode outputs must not drop while a command is on the interface,
    // or chip_top's command-source mux would withdraw it mid-handshake.
    a_mode_holds_pending_cmd: assert property (
        @(posedge clk) disable iff (!rst_n)
        (cmd_valid && !burst_abort) |-> (bypass_active && burst_active)
    ) else $error("%m: mode outputs dropped with a command still pending");

    // Padding must never produce a command.
    a_no_cmd_after_done: assert property (
        @(posedge clk) disable iff (!rst_n)
        (state == B_EMIT && done_flag && cmd_slot_free) |=> !cmd_valid
    ) else $error("%m: command emitted for a padding slot");

    // The mode outputs must track the FSM exactly.
    a_bypass_tracks_state: assert property (
        @(posedge clk) disable iff (!rst_n)
        bypass_active == (state != B_IDLE)
    ) else $error("%m: bypass_active does not track the FSM");

    // Abort must reach idle in one cycle from anywhere.
    a_abort_is_immediate: assert property (
        @(posedge clk) disable iff (!rst_n)
        burst_abort |=> (state == B_IDLE)
    ) else $error("%m: burst_abort did not return to B_IDLE");
`endif

endmodule : rx_burst_ctrl
