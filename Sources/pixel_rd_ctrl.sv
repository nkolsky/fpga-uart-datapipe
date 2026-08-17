// pixel_rd_ctrl.sv
// ----------------
// Single Pixel Read controller, 100 MHz memory domain.
//
// Owns the SRAM read port for a handful of cycles, extracts one pixel from
// the three channel words, and hands the finished reply to tx_reply_ctrl
// across a held request/accept handshake.
//
// -----------------------------------------------------------------------
// SEQUENCE
// -----------------------------------------------------------------------
//   P_IDLE   latch a validated row/col
//   P_REQ    hold pix_rd_req until mem_interlock grants ownership
//   P_ADDR   drive rd_en + rd_addr for one cycle
//   P_WAIT   rgb_sram has a one-cycle registered read
//   P_CAP    capture the three channel words, select the byte lane,
//            and RELEASE memory ownership
//   P_SEND   hold rpy_valid and the payload until rpy_accept
//   P_DONE   handshake recovery, then back to idle
//
// pix_rd_req is a LEVEL held until granted -- a one-cycle pulse could be
// missed while the interlock is servicing a write or an image read.
//
// -----------------------------------------------------------------------
// THE REPLY HANDSHAKE -- WHY rpy_valid IS A LEVEL
// -----------------------------------------------------------------------
// An earlier revision pulsed rpy_valid for a single cycle in P_SEND and
// advanced unconditionally. That silently loses the reply whenever
// tx_reply_ctrl cannot take it, and tx_reply_ctrl very often cannot: it
// holds exactly ONE pending reply and it refuses to drive the MAC at all
// while tx_seq_busy or mac_busy is high. During an image transfer that is
// roughly 1.5 seconds of continuous refusal. A one-cycle strobe into a
// consumer that is busy for 1.5 s is not a handover, it is a discard.
//
// So the contract is now four-phase and explicit:
//
//   pixel_rd_ctrl  raises rpy_valid and holds it, with rpy_row, rpy_col
//                  and rpy_pixel stable, until it sees rpy_accept
//   tx_reply_ctrl  raises pix_accept for one cycle when it has actually
//                  latched the payload into its pending slot
//   pixel_rd_ctrl  drops rpy_valid and recovers
//
// The payload is written in P_CAP, one full cycle BEFORE rpy_valid rises
// in P_SEND. That ordering matters because the level crosses to the
// 256 MHz domain through a two-flop synchroniser while the payload crosses
// unsynchronised: by the time the destination can possibly observe
// rpy_valid high, the payload has been stable for at least three
// destination clocks. It then stays stable until the accept returns.
//
// P_DONE holds for HS_RECOVER cycles rather than one. tx_reply_ctrl arms
// its next pixel acceptance on the FALLING edge of the synchronised
// rpy_valid, which takes two to three 256 MHz clocks to propagate back.
// Waiting here removes any need to argue from UART pacing that the next
// request cannot arrive too soon.
//
// -----------------------------------------------------------------------
// MEMORY OWNERSHIP IS RELEASED AT CAPTURE, NOT AT SEND
// -----------------------------------------------------------------------
// pix_rd_done pulses on the P_CAP -> P_SEND transition. Everything the
// SRAM is needed for has happened by then; the pixel is in a register.
//
// Holding ownership through P_SEND would mean a reply waiting behind an
// image transfer blocks every pending write for the whole 1.5 s of that
// transfer, for no benefit. Releasing early lets writes and image reads
// proceed normally while the reply waits its turn on the TX path.
//
// The controller stays busy for the whole time, so a second request is
// still rejected and flagged rather than overwriting the first.
//
// -----------------------------------------------------------------------
// ADDRESS AND BYTE LANE
// -----------------------------------------------------------------------
//   pixel_index = row * IMG_WIDTH + col        (concatenation when W = 256)
//   word_addr   = pixel_index >> 2
//   lane        = pixel_index[1:0]
//
// LANE 0 IS THE MOST SIGNIFICANT BYTE of the 32-bit word:
//
//   lane 0 -> word[31:24]     lane 2 -> word[15: 8]
//   lane 1 -> word[23:16]     lane 3 -> word[ 7: 0]
//
// This orientation is not inferred. rom_sequencer.sv does exactly the same
// thing on the image path --
//
//     pixels[0] <= {red_data[31:24], green_data[31:24], blue_data[31:24]};
//
// -- and the write path uses the same lane convention when packing bytes
// into a word. Reading with the opposite orientation would return a
// neighbouring pixel, silently.

`timescale 1ns/1ps

module pixel_rd_ctrl
    import memory_pkg::*;
#(
    // Cycles held in P_DONE so the cross-domain handshake can retract.
    // Eight 100 MHz cycles (80 ns) comfortably covers the two-to-three
    // 256 MHz clocks (~12 ns) the level takes to fall at the far side.
    parameter int HS_RECOVER = 8
)(
    input  logic        clk,              // CLK100MHZ
    input  logic        rst_n,            // sync_rst_n

    // ---- request, already validated and synchronised to this domain ----
    input  logic        req_valid,        // one-cycle strobe
    input  logic [9:0]  req_row,
    input  logic [9:0]  req_col,

    // ---- memory ownership, from mem_interlock --------------------------
    output logic        pix_rd_req,       // level, held until granted
    input  logic        pix_rd_gnt,
    output logic        pix_rd_done,      // one cycle, releases ownership

    // ---- SRAM read port (muxed against rom_sequencer in chip_top) ------
    output logic        sram_rd_en,
    output logic [SRAM_ADDR_WIDTH-1:0] sram_rd_addr,
    input  logic [31:0] red_data,
    input  logic [31:0] green_data,
    input  logic [31:0] blue_data,

    // ---- reply handover to tx_reply_ctrl (held handshake) --------------
    output logic        rpy_valid,        // LEVEL, held until rpy_accept
    input  logic        rpy_accept,       // one cycle, from the 130 MHz side
    output logic [9:0]  rpy_row,
    output logic [9:0]  rpy_col,
    output logic [23:0] rpy_pixel,

    // ---- status ---------------------------------------------------------
    output logic        busy,
    output logic        req_overrun       // sticky: request arrived while busy
);

    typedef enum logic [2:0] {
        P_IDLE, P_REQ, P_ADDR, P_WAIT, P_CAP, P_SEND, P_DONE
    } pstate_t;

    pstate_t     state;
    logic [9:0]  row_q, col_q;
    logic [15:0] pix_index;
    logic [1:0]  lane_q;
    logic [3:0]  recover_cnt;

    // busy spans the whole transaction INCLUDING the wait for the TX path,
    // which is what makes a second request an overrun rather than a queue.
    assign busy       = (state != P_IDLE);
    assign pix_rd_req = (state == P_REQ);
    assign rpy_valid  = (state == P_SEND);
    assign rpy_row    = row_q;
    assign rpy_col    = col_q;

    // pixel_index = row * IMG_WIDTH + col. For IMG_WIDTH = 256 this is a
    // concatenation and costs no hardware; written as arithmetic so a
    // non-power-of-two width still produces correct logic.
    assign pix_index    = 16'(row_q) * 16'(IMG_WIDTH) + 16'(col_q);
    assign sram_rd_addr = pix_index[SRAM_ADDR_WIDTH+1:2];
    assign sram_rd_en   = (state == P_ADDR);

    // Byte-lane select. Lane 0 is the MOST significant byte -- see header.
    function automatic logic [7:0] lane_byte(input logic [31:0] word,
                                             input logic [1:0]  lane);
        case (lane)
            2'd0: lane_byte = word[31:24];
            2'd1: lane_byte = word[23:16];
            2'd2: lane_byte = word[15: 8];
            2'd3: lane_byte = word[ 7: 0];
        endcase
    endfunction

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state       <= P_IDLE;
            row_q       <= '0;
            col_q       <= '0;
            lane_q      <= '0;
            rpy_pixel   <= '0;
            pix_rd_done <= 1'b0;
            req_overrun <= 1'b0;
            recover_cnt <= '0;
        end
        else begin
            pix_rd_done <= 1'b0;

            // A request arriving mid-service is rejected and the first one
            // kept. Overwriting would answer the newer request and silently
            // discard an answer the host is still waiting for.
            if (req_valid && (state != P_IDLE))
                req_overrun <= 1'b1;

            unique case (state)

            P_IDLE: if (req_valid) begin
                row_q <= req_row;
                col_q <= req_col;
                state <= P_REQ;
            end

            // Hold the request until the interlock grants the read port.
            // This can be a long wait -- an image readback is ~1.5 s -- and
            // that is fine: the request is deferred, never lost.
            P_REQ: if (pix_rd_gnt) begin
                lane_q <= pix_index[1:0];
                state  <= P_ADDR;
            end

            P_ADDR: state <= P_WAIT;      // rd_en asserted this cycle

            P_WAIT: state <= P_CAP;       // one-cycle registered SRAM read

            // Capture, then hand the memory back immediately. rgb_sram holds
            // rd_data when rd_en is low, so the data sampled here is the word
            // addressed in P_ADDR.
            P_CAP: begin
                rpy_pixel   <= {lane_byte(red_data,   lane_q),
                                lane_byte(green_data, lane_q),
                                lane_byte(blue_data,  lane_q)};
                pix_rd_done <= 1'b1;
                recover_cnt <= '0;
                state       <= P_SEND;
            end

            // rpy_valid is asserted combinationally from this state and the
            // payload is already stable. Wait -- for as long as it takes --
            // for tx_reply_ctrl to confirm it has taken a copy.
            P_SEND: if (rpy_accept)
                state <= P_DONE;

            // Let the far side observe rpy_valid fall before another reply
            // can possibly be offered.
            P_DONE: if (recover_cnt == 4'(HS_RECOVER - 1))
                state <= P_IDLE;
            else
                recover_cnt <= recover_cnt + 4'd1;

            default: state <= P_IDLE;
            endcase
        end
    end

`ifndef SYNTHESIS
    // The read port may only be driven while ownership is held.
    a_rd_only_when_granted: assert property (
        @(posedge clk) disable iff (!rst_n)
        sram_rd_en |-> $past(pix_rd_gnt)
    ) else $error("%m: SRAM read asserted without a grant");

    // Ownership is released exactly once, on leaving P_CAP.
    a_done_follows_cap: assert property (
        @(posedge clk) disable iff (!rst_n)
        pix_rd_done |-> $past(state == P_CAP)
    ) else $error("%m: pix_rd_done outside the capture transition");

    // The reply must not change while it is being offered.
    a_reply_stable: assert property (
        @(posedge clk) disable iff (!rst_n)
        (rpy_valid && !rpy_accept) |=> ($stable(rpy_pixel) &&
                                        $stable(rpy_row)   &&
                                        $stable(rpy_col))
    ) else $error("%m: reply payload changed while awaiting accept");

    // A reply is never withdrawn without having been accepted.
    a_no_reply_loss: assert property (
        @(posedge clk) disable iff (!rst_n)
        (rpy_valid && !rpy_accept) |=> rpy_valid
    ) else $error("%m: rpy_valid dropped without an accept");

    // An accept may only arrive while a reply is actually on offer.
    a_accept_qualified: assert property (
        @(posedge clk) disable iff (!rst_n)
        rpy_accept |-> rpy_valid
    ) else $error("%m: rpy_accept received outside P_SEND");
`endif

endmodule : pixel_rd_ctrl
