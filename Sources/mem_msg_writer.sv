// mem_msg_writer.sv
// =================
// Turns received MESSAGES into rectangles and pixel streams for
// pixel_word_packer. Sits between the clock crossing and the packer.
//
//   crossing -> mem_msg_writer -> pixel_word_packer -> masked word write
//
// -----------------------------------------------------------------------
// WHAT IT DOES
// -----------------------------------------------------------------------
//   MSG_PIX_WRITE   start a 1x1 rectangle, feed one pixel
//   MSG_BURST_HDR   start an H x W rectangle, feed nothing
//   MSG_BURST_DATA  feed up to four pixels into the rectangle already open
//   anything else   accepted and ignored -- reads and register writes are
//                   handled elsewhere on the memory side
//
// A single pixel write and a burst are the same operation downstream. The
// only difference is the rectangle size and where the pixels come from.
//
// -----------------------------------------------------------------------
// THE PACKER OWNS THE RECTANGLE, NOT THIS MODULE
// -----------------------------------------------------------------------
// A burst arrives as SEVERAL data messages but is ONE rectangle. This module
// therefore calls start exactly once, on the header, and never again for the
// data messages that follow. The packer holds position, the accumulated word
// and the remaining count across every message boundary.
//
// That matters for unaligned bursts. A rectangle starting at linear 2 with
// width 6 flushes a partial word after pixels 2 and 3, then holds pixels 4
// and 5 in the accumulator when the first message ENDS, merging them with
// pixels 6 and 7 from the next message into a single full-word write.
// Restarting per message would lose them.
//
// -----------------------------------------------------------------------
// PADDING MUST BE REFUSED, NOT WRITTEN
// -----------------------------------------------------------------------
// A burst data message always carries four pixels. When H*W is not a multiple
// of four, the final message is partly padding. The packer refuses it --
// pixel_ready falls the moment the rectangle completes -- so this module
// CHECKS pack_busy BEFORE EVERY PIXEL and abandons the rest of the message
// once the rectangle is done. Pushing all four regardless would deadlock
// waiting for a ready that never arrives.
//
// -----------------------------------------------------------------------
// PAYLOAD LAYOUT
// -----------------------------------------------------------------------
// The crossing carries msg_kind plus a 96-bit payload, sized for the widest
// message. Fields are right-aligned so narrow messages ignore the top bits.
//
//   MSG_PIX_WRITE    [47:24] pixel address      [23:0] pixel {R,G,B}
//   MSG_BURST_HDR    [43:34] width   [33:24] height   [23:0] base address
//   MSG_BURST_DATA   four pixels, pixel 0 in the MOST significant bits:
//                      pixel 0 [95:72]   pixel 1 [71:48]
//                      pixel 2 [47:24]   pixel 3 [23: 0]
//
// Burst data is the DELIMITER-STRIPPED payload, so the four pixels are simply
// consecutive. The straddle in the wire format -- where one pixel spans the
// comma at byte 5 -- is resolved by rx_msg_parser before the crossing. If the
// raw frame were crossed instead, that unpacking would have to be repeated
// here.

`timescale 1ns/1ps

module mem_msg_writer
    import msg_format_pkg::*;
    import memory_pkg::*;
#(
    parameter int PAYLOAD_BITS = 96,
    parameter int PIX_BITS     = 8
)(
    input  logic clk,
    input  logic rst_n,

    // ---- from the crossing ---------------------------------------------
    input  logic                     msg_valid,
    output logic                     msg_ready,
    input  msg_kind_t                msg_kind,
    input  logic [PAYLOAD_BITS-1:0]  msg_payload,

    // ---- to pixel_word_packer ------------------------------------------
    output logic                     pack_start,
    output logic [23:0]              pack_base_addr,
    output logic [9:0]               pack_height,
    output logic [9:0]               pack_width,

    output logic                     pixel_valid,
    input  logic                     pixel_ready,
    output logic [PIX_BITS-1:0]      pixel_r,
    output logic [PIX_BITS-1:0]      pixel_g,
    output logic [PIX_BITS-1:0]      pixel_b,

    input  logic                     pack_busy,

    // Sticky: at least one message was discarded for addressing outside the
    // image. See the note below.
    output logic                     wr_rejected
);

    // -----------------------------------------------------------------
    // ADDRESS RANGE CHECK -- INHERITED FROM sram_wr_ctrl
    //
    // rx_classifier deliberately applies NO bound to a single pixel write:
    // any 24-bit address is structurally legal, and the check belongs to
    // whoever knows the image geometry. That used to be sram_wr_ctrl, which
    // this path replaces, so the check moves here. Without it an
    // out-of-range address would wrap into the SRAM and corrupt an unrelated
    // pixel.
    //
    // A burst header is bounds-checked by rx_classifier already (base inside
    // the image, extent does not overrun), so only the single pixel case
    // needs testing here.
    // -----------------------------------------------------------------
    localparam int TOTAL_PIXELS = IMG_WIDTH * IMG_HEIGHT;

    logic pix_addr_ok;
    assign pix_addr_ok = (msg_payload[47:24] < 24'(TOTAL_PIXELS));

    localparam int N_PIX = 4;               // pixels per burst data message

    typedef enum logic [1:0] {
        W_IDLE = 2'd0,      // waiting for a message
        W_WAIT = 2'd1,      // start issued, waiting for the packer to arm
        W_FEED = 2'd2       // streaming pixels out of the held payload
    } state_e;

    state_e            state;
    logic [PAYLOAD_BITS-1:0] held;
    logic [2:0]        pix_left;            // 0..4
    logic [1:0]        pix_idx;             // which pixel of the message

    // -------------------------------------------------------------------
    // Pixel extraction. Pixel 0 occupies the most significant 24 bits.
    // -------------------------------------------------------------------
    function automatic logic [23:0] pix_of(input logic [PAYLOAD_BITS-1:0] pl,
                                           input logic [1:0] i);
        return pl[PAYLOAD_BITS-1 - 24*int'(i) -: 24];
    endfunction

    logic [23:0] cur_pix;
    assign cur_pix = pix_of(held, pix_idx);

    assign pixel_r = cur_pix[23:16];
    assign pixel_g = cur_pix[15: 8];
    assign pixel_b = cur_pix[ 7: 0];

    // A message is taken only when idle, so a burst data message can never
    // overtake the pixels of the one before it.
    assign msg_ready   = (state == W_IDLE);
    assign pixel_valid = (state == W_FEED) && pack_busy;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state          <= W_IDLE;
            held           <= '0;
            pix_left       <= '0;
            pix_idx        <= '0;
            wr_rejected    <= 1'b0;
            pack_start     <= 1'b0;
            pack_base_addr <= '0;
            pack_height    <= '0;
            pack_width     <= '0;
        end else begin
            pack_start <= 1'b0;

            case (state)

                W_IDLE: begin
                    if (msg_valid) begin
                        pix_idx <= '0;

                        // Default: hold the payload as-is. Burst data has
                        // pixel 0 in the most significant 24 bits, which is
                        // what pix_of() expects.
                        held <= msg_payload;

                        // A single pixel write carries its pixel in the LOW
                        // 24 bits, so realign it to the top before feeding.
                        // Without this the writer reads payload[95:72] --
                        // zeros -- and writes a black pixel with the correct
                        // address and byte enable, which looks like a data
                        // corruption rather than a decode fault.
                        if (msg_kind == MSG_PIX_WRITE)
                            held <= {msg_payload[23:0], {(PAYLOAD_BITS-24){1'b0}}};

                        unique case (msg_kind)

                            // ---- single pixel: a 1x1 rectangle ----------
                            MSG_PIX_WRITE: if (!pix_addr_ok) begin
                                // Outside the image. Drop it and report.
                                wr_rejected <= 1'b1;
                            end else begin
                                pack_start     <= 1'b1;
                                pack_base_addr <= msg_payload[47:24];
                                pack_height    <= 10'd1;
                                pack_width     <= 10'd1;
                                pix_left       <= 3'd1;
                                // pack_start is REGISTERED, so pack_busy
                                // does not rise until the cycle after. Going
                                // straight to W_FEED would see !pack_busy and
                                // abort the rectangle before it opened.
                                state          <= W_WAIT;
                            end

                            // ---- burst header: open the rectangle -------
                            // No pixels. start is asserted ONCE here and
                            // never again for the data messages that follow.
                            MSG_BURST_HDR: begin
                                pack_start     <= 1'b1;
                                pack_base_addr <= msg_payload[23:0];
                                pack_height    <= msg_payload[33:24];
                                pack_width     <= msg_payload[43:34];
                                pix_left       <= 3'd0;
                            end

                            // ---- burst data: feed the open rectangle ----
                            MSG_BURST_DATA: begin
                                pix_left <= 3'(N_PIX);
                                state    <= W_FEED;
                            end

                            // Reads and register writes are consumed
                            // elsewhere; accept and drop them here.
                            default: ;

                        endcase
                    end
                end

                // Only reached after issuing start. A burst DATA message
                // skips this state: its rectangle is already open.
                W_WAIT: begin
                    if (pack_busy) state <= W_FEED;
                end

                W_FEED: begin
                    // The rectangle finished mid-message: the rest is
                    // padding. Abandon it rather than waiting for a ready
                    // that will never come.
                    if (!pack_busy) begin
                        state    <= W_IDLE;
                        pix_left <= '0;
                    end
                    else if (pixel_valid && pixel_ready) begin
                        pix_idx  <= pix_idx + 2'd1;
                        pix_left <= pix_left - 3'd1;
                        if (pix_left == 3'd1) state <= W_IDLE;
                    end
                end

                default: state <= W_IDLE;

            endcase
        end
    end

`ifndef SYNTHESIS
    // A pixel is never offered unless a rectangle is open.
    a_pixel_needs_rect: assert property (
        @(posedge clk) disable iff (!rst_n)
        pixel_valid |-> pack_busy
    ) else $error("%m: pixel offered with no rectangle open");

    // start is a one-cycle pulse.
    a_start_pulse: assert property (
        @(posedge clk) disable iff (!rst_n)
        pack_start |=> !pack_start
    ) else $error("%m: pack_start held for more than one cycle");

    // A new message is never taken while pixels are still outstanding.
    a_no_overtake: assert property (
        @(posedge clk) disable iff (!rst_n)
        (msg_valid && msg_ready) |-> (state == W_IDLE)
    ) else $error("%m: message accepted while feeding pixels");
`endif

endmodule : mem_msg_writer
