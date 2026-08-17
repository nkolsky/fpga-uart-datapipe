// rx_burst_ctrl.sv
// ================
// Owns burst MODE on the receive side. Protocol only.
//
// The whole job, stated completely:
//
//   1. A header was accepted  -> raise bypass_active THIS cycle, so
//                                rx_msg_parser stops decoding opcodes at
//                                bytes 1, 6 and 11.
//   2. Count data frames.
//   3. After ceil(H*W / 4) of them -> drop bypass_active.
//   4. A data frame with no burst armed -> sticky error.
//
// No addressing, no pixel unpacking, no rectangle walk. pixel_word_packer on
// the memory side owns all of that, because it owns the image geometry.
//
// =======================================================================
// H*W IS NEVER COMPUTED
// =======================================================================
// Every earlier version treated H*W as a product to be evaluated: a DSP48
// multiply, then a LUT multiply, then a shift-and-add sequencer. All three
// were the critical path, and the shift-add version was the worst -- not
// because of the adds, but because handling "a data frame arrived before
// the product landed" (mul_early) dragged a COMBINATIONAL signal out of
// rx_msg_parser into a chain of three 17-bit carry chains:
//
//   frame_buf -> parser decode -> data_seen -> add -> compare -> subtract
//                                           -> mux -> pix_left.D
//
// H*W is not a product. It is a BUDGET, and there are 5632 clocks per
// frame to accumulate it. So credit one row of W pixels per cycle and
// spend 4 per data frame:
//
//   on header    rows_left = H,  pix_left = 0
//   each cycle   frame     -> pix_left -= BURST_PIX_PER_MSG
//                rows_left -> pix_left += width_q, rows_left -= 1
//
// ONE adder, operand muxed two ways. No multiplier, no shifter, and no
// accumulate-into-itself pattern for Vivado to absorb into a DSP48.
//
// The "count has not landed yet" problem disappears rather than being
// handled: the total is never computed at an instant, so there is no
// instant before which it is wrong. A frame arriving during the refill
// simply subtracts from a partially credited budget and the running total
// stays exact at any frame spacing.
//
// Refill takes H cycles (<= 256). The earliest a data frame can follow the
// header is one complete 16-byte frame, 16 x 11 x DIV_TX = 5632 clocks.
// 22x margin. See a_no_underflow.
//
// =======================================================================
// WHY THE COUNT IS IN PIXELS, NOT FRAMES
// =======================================================================
// A burst data message always carries FOUR pixels. When H*W is not a
// multiple of four the final message is partly padding, so counting frames
// would need a ceiling division and would still not say how many pixels are
// real. Counting pixels and subtracting four per frame handles both; the
// padding is discarded downstream by the packer.
//
// =======================================================================
// THE HEADER CHECK LIVES IN rx_classifier
// =======================================================================
// hdr_accept arrives already qualified -- H and W non-zero and within the
// image. One definition, in the classifier.
//
// =======================================================================
// TIMING NOTES -- what must not be reintroduced
// =======================================================================
// Three things here exist only to keep paths short. Each one was, at some
// point, the failing path in this module. None of them is a style choice.
//
//   data_seen is REGISTERED as frame_data. msg_kind is a combinational
//   output of rx_msg_parser, so using it raw puts the opcode decode cone
//   in front of whatever it feeds. One flop buys the whole cone back,
//   against 352 clocks of slack before the next frame's byte 1 arrives.
//
//   last_frame is REGISTERED. `pix_left <= 4` is a 17-bit comparator and
//   must not sit behind frame_data. One cycle of staleness is safe for
//   free: pix_left only moves on refill or drain cycles, and the
//   "no rows remain" term self-guards the refill case -- if a refill
//   happened last cycle then rows remained then, so last_frame was false.
//
//   hdr_accept is REGISTERED as hdr_arm. It is combinational out of
//   rx_classifier -- accept && (s2_kind == MSG_BURST_HDR) -- and driving
//   it straight into this module's enables put classifier logic in front
//   of ~40 CE pins across a module boundary: -0.273ns at width_q_reg.
//   Two costs, both removed by one flop. The cross-module net drops to a
//   single load, and the endpoint moves from CE to D -- FDCE CE setup is
//   0.284ns worse than D setup (Setup_fdce_C_CE -0.205 against
//   Setup_fdce_C_D +0.079 on this part).
//
//   THE ONE-CYCLE DELAY IS SAFE, and an earlier revision of this comment
//   claimed otherwise. It asserted that arming late "leaves a window in
//   which the parser still decodes opcodes on a burst data frame". There
//   is no such window. rx_msg_parser is combinational on the live frame
//   buffer and its output is only consumed when rx_frame_done is high --
//   the NEXT frame, a full 16 bytes away. 16 x 11 x DIV_TX = 5632 clocks,
//   and even the next frame's byte 1 is ~700 clocks out. One cycle
//   against 700 is not a window.
//
//   refilling is REGISTERED, and is the reason this file exists in its
//   current form. pix_left's CLOCK ENABLE is (frame_data || rows remain).
//   Deriving "rows remain" as a live rows_left != 0 zero-detect put a
//   9-bit OR-reduction directly in that cone. Vivado built it as
//   LUT6 -> LUT4 -> LUT3 driving a fanout-17 net -- 0.828 ns of logic
//   behind 2.520 ns of route -- and it was the last failing path in this
//   module at -0.132 ns against a 3.571 ns period.
//
//   rows_left only ever decrements, so the next value is knowable a cycle
//   early: after this decrement rows remain iff rows_left != 1. The
//   compare then runs flop -> LUT -> flop in parallel with the decrement,
//   and the enable cone collapses to one LUT3 fed by three flops.
//
//   DO NOT substitute (rows_left != '0) back in for readability. That
//   single edit reintroduces the path. refilling holds an exact
//   invariant with rows_left, not an approximation -- see the assertion
//   a_refilling_exact at the bottom, which checks it every cycle.

`timescale 1ns/1ps

module rx_burst_ctrl
    import msg_format_pkg::*;
    import memory_pkg::*;
#(
    parameter int IMG_W = IMG_WIDTH,
    parameter int IMG_H = IMG_HEIGHT
)(
    input  logic clk,
    input  logic rst_n,

    // ---- from rx_mac / rx_msg_parser / rx_classifier --------------------
    input  logic                   msg_valid,    // rx_mac frame_done
    input  msg_kind_t              msg_kind,     // COMBINATIONAL from parser
    input  logic                   hdr_accept,   // classifier passed a header
    input  logic [BURST_DIM_W-1:0] height,
    input  logic [BURST_DIM_W-1:0] width,

    // ---- outputs ---------------------------------------------------------
    output logic bypass_active,   // -> rx_msg_parser (this clock domain)
    output logic burst_active,
    output logic burst_done,      // one pulse when the last frame is counted
    output logic err_unexpected   // sticky: data frame with no burst armed
);

    // Counter widths. All derived; nothing here is a literal.
    localparam int PIX_CNT_W = $clog2(IMG_W * IMG_H + 1);  // 17 @ 256x256
    localparam int ROW_CNT_W = $clog2(IMG_H + 1);          //  9
    localparam int COL_CNT_W = $clog2(IMG_W + 1);          //  9

    localparam logic [PIX_CNT_W-1:0] PIX_PER_MSG =
        PIX_CNT_W'(BURST_PIX_PER_MSG);                     // 4

    logic                 active;
    logic [PIX_CNT_W-1:0] pix_left;    // credited but not yet received
    logic [ROW_CNT_W-1:0] rows_left;   // rows not yet credited
    logic                 refilling;   // registered (rows_left != 0)
    logic [COL_CNT_W-1:0] width_q;     // latched W, one row's worth of credit
    logic                 last_frame;  // registered terminal condition

    // Dimensions are narrowed to their counter widths once, here. The
    // classifier has already range-checked both against the image, so the
    // upper bits of the 24-bit wire fields are known zero and nothing
    // downstream should pay for them.
    wire [ROW_CNT_W-1:0] height_n = ROW_CNT_W'(height);
    wire [COL_CNT_W-1:0] width_n  = COL_CNT_W'(width);

    // -----------------------------------------------------------------
    // Registered data-frame strobe. See TIMING NOTES.
    // -----------------------------------------------------------------
    logic frame_data;
    logic hdr_arm;
    logic [ROW_CNT_W-1:0] height_q;
    logic [COL_CNT_W-1:0] width_d;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            frame_data <= 1'b0;
            hdr_arm    <= 1'b0;
            height_q   <= '0;
            width_d    <= '0;
        end
        else begin
            frame_data <= msg_valid && (msg_kind == MSG_BURST_DATA);
            // hdr_accept and the dimensions it carries are captured
            // together, so they cannot drift apart. See TIMING NOTES.
            hdr_arm    <= hdr_accept;
            height_q   <= height_n;
            width_d    <= width_n;
        end
    end

    // bypass_active and burst_active are the same condition. They are kept
    // separate because they mean different things: one tells the parser how
    // to decode, the other tells the rest of the design a burst is running.
    assign bypass_active = active;
    assign burst_active  = active;

    // -----------------------------------------------------------------
    // Credit / spend counter.
    //
    // Refill is suppressed on a drain cycle so the adder keeps exactly two
    // operands. Costing the refill one cycle is free: it has 5632 clocks
    // to place at most 256 credits.
    // -----------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            active     <= 1'b0;
            pix_left   <= '0;
            rows_left  <= '0;
            refilling  <= 1'b0;
            width_q    <= '0;
            last_frame <= 1'b0;
            burst_done <= 1'b0;
        end
        else begin
            burst_done <= 1'b0;

            // Terminal condition, one cycle behind the counters. See the
            // TIMING NOTES above for why the staleness is safe.
            last_frame <= !refilling && (pix_left <= PIX_PER_MSG);

            if (hdr_arm) begin
                // Armed one cycle after hdr_accept. The parser does not
                // consume bypass_active until the next frame completes,
                // ~5632 clocks out; see TIMING NOTES.
                active     <= 1'b1;
                rows_left  <= height_q;
                refilling  <= (height_q != '0);
                width_q    <= width_d;
                pix_left   <= '0;
                last_frame <= 1'b0;
            end
            else if (active && frame_data) begin
                if (last_frame) begin
                    // Pixels beyond the count are padding, dropped
                    // downstream by pixel_word_packer.
                    active     <= 1'b0;
                    pix_left   <= '0;
                    rows_left  <= '0;
                    refilling  <= 1'b0;
                    burst_done <= 1'b1;
                    last_frame <= 1'b0;
                end
                else begin
                    pix_left <= pix_left - PIX_PER_MSG;
                end
            end
            else if (active && refilling) begin
                pix_left  <= pix_left + PIX_CNT_W'(width_q);
                rows_left <= rows_left - 1'b1;
                // Next value, computed in parallel with the decrement:
                // rows remain after this step iff rows_left != 1.
                refilling <= (rows_left != ROW_CNT_W'(1));
            end
        end
    end

    // -----------------------------------------------------------------
    // Sticky diagnostics. Off the counter path entirely.
    // -----------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            err_unexpected <= 1'b0;
        else if (frame_data && !active)
            // Pixel data with nothing armed. The parser only emits
            // MSG_BURST_DATA while bypass_active is high, so this should be
            // unreachable; it is reported rather than assumed away.
            err_unexpected <= 1'b1;
        else if (hdr_arm && active)
            // A header inside a burst. Cannot happen while bypass_active
            // suppresses opcode decoding.
            err_unexpected <= 1'b1;
    end

`ifndef SYNTHESIS
    // refilling is a TIMING transform, not a behavioural one: it must equal
    // rows_left != 0 on every cycle, with no staleness. If a future edit
    // adds a path that writes rows_left without writing refilling, this
    // fires immediately rather than showing up as a burst that ends early.
    a_refilling_exact: assert property (
        @(posedge clk) disable iff (!rst_n)
        refilling == (rows_left != '0)
    ) else $error("%m: refilling disagrees with rows_left");

    // The refill must stay ahead of the drain. Guaranteed by UART framing
    // -- 5632 clocks between data frames against at most IMG_H refill
    // cycles -- but the module does not itself enforce it, so it is
    // checked rather than assumed. A testbench driving frames faster than
    // the line can carry them will trip this.
    a_no_underflow: assert property (
        @(posedge clk) disable iff (!rst_n)
        (active && frame_data && !last_frame) |-> (pix_left >= PIX_PER_MSG)
    ) else $error("%m: drain outran refill -- pix_left would underflow");

    // A burst never completes before every row has been credited.
    a_all_rows_credited: assert property (
        @(posedge clk) disable iff (!rst_n)
        burst_done |-> (rows_left == '0)
    ) else $error("%m: burst completed with rows still uncredited");

    // Bypass and burst_active track the same flop.
    a_bypass_matches: assert property (
        @(posedge clk) disable iff (!rst_n)
        bypass_active == burst_active
    ) else $error("%m: bypass_active and burst_active disagree");

    // done is a single pulse.
    a_done_pulse: assert property (
        @(posedge clk) disable iff (!rst_n)
        burst_done |=> !burst_done
    ) else $error("%m: burst_done held for more than one cycle");

    // The refill is expected to retire long before the first data frame.
    c_data_during_refill: cover property (
        @(posedge clk) disable iff (!rst_n)
        (active && frame_data && refilling)
    );
`endif

endmodule : rx_burst_ctrl
