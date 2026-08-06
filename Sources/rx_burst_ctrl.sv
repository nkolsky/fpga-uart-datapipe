// rx_burst_ctrl.sv
// ================
// Owns burst MODE on the receive side. Protocol only.
//
// =======================================================================
// WHAT THIS MODULE USED TO DO, AND NO LONGER DOES
// =======================================================================
// It used to walk the rectangle: base row and column, current row and
// column, last-row and last-column limits, the row-major multiply, a
// four-slot pixel latch, and a command output that emitted one
// {address, pixel} pair per clock.
//
// All of that is gone. Address generation belongs to the memory side, which
// owns the image geometry -- pixel_word_packer does it now, and does it
// better: four consecutive pixels are exactly one 32-bit word per colour
// channel, so a burst data message becomes ONE masked word write per channel
// instead of four single-lane writes to the same word.
//
// A BRAM write activates a whole row regardless of how many byte lanes are
// enabled, so the old scheme paid four row activations for one word of data,
// on the traffic that dominates the system.
//
// This module keeps only what is genuinely protocol:
//
//   bypass_active   the next frames carry pixel data, not opcodes, so
//                   rx_msg_parser must stop looking for opcodes at bytes
//                   1, 6 and 11
//   burst_active    a burst is in progress
//   counting        how many pixels remain, to know when to drop bypass
//
// It no longer needs IMG_W or IMG_H for addressing -- only to size the pixel
// counter. RX does protocol; memory does memory.
//
// =======================================================================
// THE HEADER CHECK LIVES IN rx_classifier
// =======================================================================
// hdr_accept arrives already qualified. The dimension check -- H and W
// non-zero and within the image -- is the classifier's, so it has exactly one
// definition. This module previously duplicated it.
//
// =======================================================================
// WHY THE COUNT IS IN PIXELS, NOT FRAMES
// =======================================================================
// A burst data message always carries FOUR pixels. When H*W is not a multiple
// of four the final message is partly padding, so counting frames would need
// a ceiling division and would still not say how many pixels are real.
// Counting pixels and subtracting four per frame handles both, and the
// padding is discarded downstream by the packer, which stops accepting once
// the rectangle is complete.

`timescale 1ns/1ps

module rx_burst_ctrl
    import msg_format_pkg::*;
    import memory_pkg::*;
    import rx_burst_ctrl_pkg::*;
#(
    parameter int IMG_W = IMG_WIDTH,
    parameter int IMG_H = IMG_HEIGHT
)(
    input  logic clk,
    input  logic rst_n,

    // ---- from rx_mac / rx_msg_parser / rx_classifier --------------------
    input  logic                   msg_valid,    // rx_mac frame_done
    input  msg_kind_t              msg_kind,
    input  logic                   hdr_accept,   // classifier passed a header
    input  logic [BURST_DIM_W-1:0] height,
    input  logic [BURST_DIM_W-1:0] width,

    // Abort from the register file.
    input  logic                   burst_abort,

    // ---- outputs ---------------------------------------------------------
    output logic bypass_active,   // -> rx_msg_parser  (this clock domain)
    output logic burst_active,
    output logic burst_done,      // one pulse when the last pixel is counted
    output logic err_unexpected   // sticky: data frame with no burst armed
);

    // Widest count the geometry can produce, plus one so the full image fits.
    localparam int PIX_CNT_W = $clog2(IMG_W * IMG_H + 1);

    rx_burst_state_t state;
    logic [PIX_CNT_W-1:0] pix_left;

    logic data_seen;
    assign data_seen = msg_valid && (msg_kind == MSG_BURST_DATA);

    // bypass_active and burst_active are the same condition today. They are
    // kept separate because they mean different things: one tells the parser
    // how to decode, the other tells the rest of the design a burst is in
    // progress.
    assign bypass_active = (state == RXBURST_ACTIVE);
    assign burst_active  = (state == RXBURST_ACTIVE);

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state          <= RXBURST_IDLE;
            pix_left       <= '0;
            burst_done     <= 1'b0;
            err_unexpected <= 1'b0;
        end else begin
            burst_done <= 1'b0;

            if (burst_abort) begin
                state    <= RXBURST_IDLE;
                pix_left <= '0;
            end
            else begin
                unique case (state)

                    RXBURST_IDLE: begin
                        if (hdr_accept) begin
                            // One multiply, once per burst. H and W are at
                            // most 10 bits each and this is not in any
                            // per-cycle path.
                            pix_left <= PIX_CNT_W'(height) * PIX_CNT_W'(width);
                            state    <= RXBURST_ACTIVE;
                        end
                        else if (data_seen) begin
                            // Pixel data with nothing armed. The parser only
                            // produces MSG_BURST_DATA while bypass_active is
                            // high, so this should be unreachable; it is
                            // reported rather than assumed away.
                            err_unexpected <= 1'b1;
                        end
                    end

                    RXBURST_ACTIVE: begin
                        if (data_seen) begin
                            if (pix_left <= PIX_CNT_W'(BURST_PIX_PER_MSG)) begin
                                // Last frame. Any pixels beyond the count are
                                // padding and are dropped downstream.
                                pix_left   <= '0;
                                state      <= RXBURST_IDLE;
                                burst_done <= 1'b1;
                            end else begin
                                pix_left <= pix_left - PIX_CNT_W'(BURST_PIX_PER_MSG);
                            end
                        end
                        else if (hdr_accept) begin
                            // A header inside a burst. Cannot happen while
                            // bypass_active suppresses opcode decoding.
                            err_unexpected <= 1'b1;
                        end
                    end

                    default: state <= RXBURST_IDLE;

                endcase
            end
        end
    end

`ifndef SYNTHESIS
    // A burst never runs with nothing left to receive.
    a_active_has_pixels: assert property (
        @(posedge clk) disable iff (!rst_n || burst_abort)
        (state == RXBURST_ACTIVE) |-> (pix_left != '0)
    ) else $error("%m: burst active with no pixels remaining");

    // Bypass and burst_active track the same state.
    a_bypass_matches: assert property (
        @(posedge clk) disable iff (!rst_n)
        bypass_active == burst_active
    ) else $error("%m: bypass_active and burst_active disagree");

    // done is a single pulse.
    a_done_pulse: assert property (
        @(posedge clk) disable iff (!rst_n)
        burst_done |=> !burst_done
    ) else $error("%m: burst_done held for more than one cycle");
`endif

endmodule : rx_burst_ctrl
