// cdc_msg_sync.sv
// ===============
// One-message CDC with full back-pressure.
//
// src_valid/src_ready and dst_valid/dst_ready form a toggle-based handshake.
// The source pushes a complete message only when the crossing is ready, and
// the destination holds the message until it is accepted.
//
// -----------------------------------------------------------------------
// Handshake
// -----------------------------------------------------------------------
// req_tgl flips when a new message is accepted. The destination sees that
// toggle after the synchronizer and raises dst_valid. When dst_ready is high,
// the destination toggles ack_tgl; the source sees that return toggle and
// clears src_ready until the handshake is complete.
//
// -----------------------------------------------------------------------
// Data handling
// -----------------------------------------------------------------------
// data_reg captures src_data on the same edge as the request toggle. The data
// stays stable for the whole handshake, and the destination samples it only
// after the request has crossed. This is the usual safe pattern: synchronize
// the control bit, then pass the payload with it.
//
// -----------------------------------------------------------------------
// Timing note
// -----------------------------------------------------------------------
// The data register is a real CDC path. The XDC should treat it as a
// multi-cycle path rather than a single-cycle synchronous path.

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
