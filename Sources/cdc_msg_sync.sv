// cdc_msg_sync.sv
// ===============
// Carries one message at a time across a clock domain boundary, with full
// handshake back-pressure in both directions.
//
//   src_valid / src_ready  ->  [ CDC ]  ->  dst_valid / dst_ready
//
// Replaces the asynchronous command FIFO between the RX and memory domains.
//
// -----------------------------------------------------------------------
// WHY NOT A FIFO
// -----------------------------------------------------------------------
// A FIFO absorbs a rate mismatch. There is no rate mismatch here. At
// 8.125 Mbaud a 16-byte message takes about 2816 UART-domain clocks and
// carries four pixels, so a pixel arrives roughly every 700 clocks against a
// memory side running at 100 MHz. The consumer is three orders of magnitude
// faster than the producer; there is nothing to buffer.
//
// There is also nothing to buffer INTO. The RX side holds no image data --
// it is protocol only -- so a FIFO there is storage in the wrong place.
//
// AREA. The command FIFO was 16 entries of 48 bits plus Gray-coded pointers,
// their synchronisers and the full/empty comparators. This is one data
// register, two toggle bits and four synchroniser flops. For a 99-bit
// message that is roughly 105 flops against roughly 800.
//
// POWER. A message crosses ONCE. The previous scheme split every burst data
// message into four pixel commands and pushed each through the FIFO
// separately, so a burst paid four crossings per message.
//
// LATENCY. A two-phase handshake costs about two destination clocks out and
// two source clocks back, so roughly four to six cycles per message. Against
// 2816 cycles of message arrival time that is invisible.
//
// -----------------------------------------------------------------------
// WHY BACK-PRESSURE, GIVEN MESSAGES ARRIVE SO SLOWLY
// -----------------------------------------------------------------------
// Not because the source is fast. It is not: ~2816 clocks between messages
// against a ~5 clock handshake. The reason is that the DESTINATION CAN BLOCK
// FOR FAR LONGER THAN A MESSAGE INTERVAL.
//
// The SRAMs have one port, arbitrated by mem_interlock:
//
//     wr_allowed  = !read_active && !pix_rd_active
//     read_active = read_go || rom_seq_busy
//
// so a read in progress blocks writes outright. An image burst read walks the
// whole image and its data leaves over UART, so it is TRANSMITTER rate
// limited -- 65536 pixels at 256x256, millions of clocks. A pixel write
// arriving while an image read streams out waits for all of it.
//
// The previous design had no way to say so. Its 16-deep command FIFO turned
// "lose the next command" into "lose the seventeenth", and cmd_ovf_sticky
// exists -- wired to LED[13] -- precisely to report when that happened. A
// sticky bit for silent data loss is evidence the problem was known, not
// that it was solved.
//
// src_ready replaces a silent overflow with an explicit stall. The intended
// chain is
//
//     memory blocked -> crossing not ready -> classifier holds its command
//                    -> MAC stops accepting frames -> RTS deasserts
//                    -> PC pauses
//
// Nothing is discarded; the transfer takes longer. Every link is one register
// deep, so no queue is needed anywhere.
//
// NOTE: the RX side does not yet consume src_ready. rx_classifier still
// pulses its command for one cycle and assumes it was taken. Until that chain
// is built, src_ready is correct but unused on this side of the boundary.
//
// -----------------------------------------------------------------------
// HOW IT WORKS -- TWO-PHASE (TOGGLE) REQ/ACK
// -----------------------------------------------------------------------
//   1. src accepts a message when src_ready is high: data is registered and
//      req_tgl flips.
//   2. req_tgl is synchronised into the destination domain. An edge means a
//      new message has arrived; dst_valid rises.
//   3. dst_valid stays high until dst_ready, then ack_tgl flips.
//   4. ack_tgl is synchronised back. src_ready returns high when the two
//      toggles agree.
//
// src_ready = (req_tgl == ack_sync), so the source cannot issue a second
// message until the first has been taken. No message is dropped, none is
// duplicated, and neither side needs to know the other's clock.
//
// THE DATA PATH IS NOT SYNCHRONISED, DELIBERATELY.
// src_data is captured in a plain register and held UNCHANGED for the entire
// handshake. The destination samples it only after req_tgl has passed through
// two synchroniser flops, by which time it has been stable for at least two
// destination clocks. This is the standard multi-cycle path formulation:
// synchronise the control bit, let the data ride along. Putting synchronisers
// on 99 data bits would cost 198 flops and would NOT make it safer -- each
// bit could resolve differently and produce a word that was never sent.
//
// The data register needs a false path or max-delay constraint in the XDC;
// without one the tool will try to close it as a single-cycle path between
// asynchronous clocks and report failure.

`timescale 1ns/1ps

module cdc_msg_sync #(
    parameter int WIDTH       = 100,
    parameter int SYNC_STAGES = 2
)(
    // ---- source domain --------------------------------------------------
    input  logic             src_clk,
    input  logic             src_rst_n,
    input  logic             src_valid,
    output logic             src_ready,
    input  logic [WIDTH-1:0] src_data,

    // ---- destination domain ---------------------------------------------
    input  logic             dst_clk,
    input  logic             dst_rst_n,
    output logic             dst_valid,
    input  logic             dst_ready,
    output logic [WIDTH-1:0] dst_data
);

    // -------------------------------------------------------------------
    // Source domain
    // -------------------------------------------------------------------
    logic             req_tgl;
    logic [WIDTH-1:0] data_reg;
    logic [SYNC_STAGES-1:0] ack_sync;

    logic src_accept;
    assign src_ready  = (req_tgl == ack_sync[SYNC_STAGES-1]);
    assign src_accept = src_valid && src_ready;

    always_ff @(posedge src_clk or negedge src_rst_n) begin
        if (!src_rst_n) begin
            req_tgl  <= 1'b0;
            data_reg <= '0;
        end else if (src_accept) begin
            // Data and toggle change on the SAME edge. The destination
            // cannot observe the toggle for at least two of its own clocks,
            // so the data is long settled by the time it is sampled.
            data_reg <= src_data;
            req_tgl  <= ~req_tgl;
        end
    end

    // -------------------------------------------------------------------
    // Destination domain
    // -------------------------------------------------------------------
    logic [SYNC_STAGES-1:0] req_sync;
    logic                   req_seen;      // last toggle value acted on
    logic                   ack_tgl;

    logic req_edge;
    assign req_edge = (req_sync[SYNC_STAGES-1] != req_seen);

    always_ff @(posedge dst_clk or negedge dst_rst_n) begin
        if (!dst_rst_n) begin
            req_sync  <= '0;
            req_seen  <= 1'b0;
            dst_valid <= 1'b0;
            dst_data  <= '0;
            ack_tgl   <= 1'b0;
        end else begin
            req_sync <= {req_sync[SYNC_STAGES-2:0], req_tgl};

            if (req_edge && !dst_valid) begin
                // Capture and present. data_reg has been stable throughout
                // the synchroniser delay.
                dst_data  <= data_reg;
                dst_valid <= 1'b1;
                req_seen  <= req_sync[SYNC_STAGES-1];
            end
            else if (dst_valid && dst_ready) begin
                // Taken. Release the source.
                dst_valid <= 1'b0;
                ack_tgl   <= ~ack_tgl;
            end
        end
    end

    // Acknowledge back into the source domain.
    always_ff @(posedge src_clk or negedge src_rst_n) begin
        if (!src_rst_n) ack_sync <= '0;
        else            ack_sync <= {ack_sync[SYNC_STAGES-2:0], ack_tgl};
    end

`ifndef SYNTHESIS
    // The source never issues while busy.
    a_src_no_overrun: assert property (
        @(posedge src_clk) disable iff (!src_rst_n)
        src_accept |-> src_ready
    ) else $error("%m: message accepted while the crossing was busy");

    // Data is stable for the whole handshake.
    a_data_stable: assert property (
        @(posedge src_clk) disable iff (!src_rst_n)
        !src_ready |=> $stable(data_reg)
    ) else $error("%m: data changed mid-handshake");

    // dst_valid holds until taken.
    a_dst_holds: assert property (
        @(posedge dst_clk) disable iff (!dst_rst_n)
        (dst_valid && !dst_ready) |=> dst_valid
    ) else $error("%m: dst_valid dropped before dst_ready");

    // ...and the payload holds with it.
    a_dst_data_stable: assert property (
        @(posedge dst_clk) disable iff (!dst_rst_n)
        (dst_valid && !dst_ready) |=> $stable(dst_data)
    ) else $error("%m: dst_data changed while waiting for dst_ready");
`endif

endmodule : cdc_msg_sync
