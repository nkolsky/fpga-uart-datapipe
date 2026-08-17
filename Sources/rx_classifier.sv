// rx_classifier.sv
// ----------------
// Validates a decoded frame and emits ONE message towards the memory domain.
//
//   rx_msg_parser -> rx_classifier -> cdc_msg_sync -> memory side
//
// =======================================================================
// ONE OUTPUT, NOT SIX
// =======================================================================
// This module used to have six output channels -- legacy RGF, single pixel
// write, register read, register write, pixel read, burst read -- each with
// its own clock crossing in chip_top. They are now one:
//
//     out_valid / out_ready / out_kind / out_payload
//
//   AREA      every separate crossing carried its own data registers on both
//             sides plus its own synchroniser chain. One crossing reuses the
//             same flops for every message kind, and the command FIFO goes
//             with them.
//   ORDERING  separate paths have no defined order between them. A burst
//             header and its data crossed on DIFFERENT paths, and only
//             similar latencies kept them in sequence. Nothing enforced it.
//             One crossing makes ordering structural.
//   COST      messages serialise rather than crossing in parallel. At about
//             2816 clocks between messages against a five clock handshake,
//             the crossing is under 0.2% utilised.
//
// =======================================================================
// IT HOLDS THE MESSAGE UNTIL IT IS TAKEN
// =======================================================================
// out_valid stays high until out_ready. Previously every output was a
// one-cycle pulse that assumed someone caught it, and a_cls_cmd_not_dropped
// asserted that the FIFO was never full rather than handling the case.
//
// That mattered because the destination CAN block for a long time. The SRAMs
// have one port and mem_interlock gives wr_allowed = !read_active, so an
// image burst read -- UART-transmitter rate limited, millions of clocks --
// blocks writes for far longer than the gap between messages. The 16-deep
// FIFO turned "lose the next command" into "lose the seventeenth", and
// cmd_ovf_sticky on LED[13] existed to report when that happened.
//
// While holding, out_busy is high. rx_mac must not deliver another frame,
// or the held message is overwritten. Holding costs ONE STATE BIT, not a
// queue: the payload is already registered, and UART with RTS/CTS is a
// stallable source, so every stage in the chain can simply wait.
//
// =======================================================================
// WHAT THIS MODULE STILL DECIDES
// =======================================================================
// Frame SHAPE is rx_mac's guarantee and opcode legality is rx_msg_parser's,
// so everything here is SEMANTIC -- questions about what a field MEANS,
// which depend on knowing which message it is:
//
//   Register read / write  address fits the RGF port, word aligned
//   Single pixel read      row < IMG_HEIGHT && col < IMG_WIDTH
//   Image burst read       H,W non-zero and within the image; base inside
//                          the image; base + extent does not overrun
//   Single pixel write     none -- any 24-bit address is structurally legal,
//                          bounds are enforced by the memory side
//   Burst data             none -- the payload is raw pixels
//
// A rejected message is not forwarded; classifier_error pulses instead.
//
// =======================================================================
// BURST DATA PASSES THROUGH
// =======================================================================
// Burst data frames now CROSS, rather than being unpacked here into pixel
// commands. rx_burst_ctrl keeps bypass_active and the frame count -- both
// protocol concerns -- and no longer generates addresses. Row-major walking
// belongs to the memory side, which owns the geometry.

`timescale 1ns/1ps

module rx_classifier
    import msg_format_pkg::*;
    import memory_pkg::*;
(
    input  logic clk,
    input  logic rst_n,

    // ---- from rx_mac / rx_msg_parser -----------------------------------
    input  logic         msg_valid,     // rx_mac frame_done, one cycle
    input  logic         frame_err,     // rx_mac framing fault, one cycle
    input  msg_kind_t    msg_kind,
    input  logic [PAYLOAD_W-1:0] field0,
    input  logic [PAYLOAD_W-1:0] field1,
    input  logic [PAYLOAD_W-1:0] field2,
    input  logic [BURST_PIX_PER_MSG-1:0][BURST_PIX_W-1:0] burst_pixels,

    // ---- message out, towards the crossing ------------------------------
    output logic         out_valid,
    input  logic         out_ready,
    output msg_kind_t    out_kind,
    output msg_payload_t out_payload,

    // High while a message is held. rx_mac must not deliver another frame.
    output logic         out_busy,

    // One cycle, when a burst header passes its dimension check. Tells
    // rx_burst_ctrl to arm. Exported rather than recomputed there so the
    // dimension check has ONE definition.
    //
    // The dimensions travel WITH it. They are registered here, so taking
    // them straight from the parser instead would give rx_burst_ctrl values
    // that had already moved on by the time hdr_accept arrived.
    output logic         hdr_accept,
    output logic [9:0]   hdr_height,
    output logic [9:0]   hdr_width,

    // ---- diagnostics -----------------------------------------------------
    output logic         classifier_error
);

    localparam int TOTAL_PIXELS = IMG_HEIGHT * IMG_WIDTH;

    // rgf_pkg is referenced only for this check, so the width used in the
    // payload struct is verified against the real port width.
    initial begin
        if (ADDR_W_RGF != rgf_pkg::ADDR_WIDTH)
            $error("%m: ADDR_W_RGF (%0d) != rgf_pkg::ADDR_WIDTH (%0d)",
                   ADDR_W_RGF, rgf_pkg::ADDR_WIDTH);
    end

    // =================================================================
    // STAGE 1 -- REGISTER THE PARSER'S OUTPUTS
    //
    // WHY THIS PIPELINE STAGE EXISTS.
    // Without it the whole decode sat between two flops: rx_mac's frame
    // buffer -> byte extraction -> opcode compare -> range checks -> the
    // payload packing mux -> this module's output register. Twelve logic
    // levels, 8.05 ns against a 7.692 ns period at 130 MHz. Eight endpoints
    // failed setup by up to 0.494 ns, all on that one path.
    //
    // The old design had the same work split across rx_msg_decode and the
    // eight parsers with msg_data registered in between. Collapsing those
    // into one parser removed a pipeline stage as a side effect, and this
    // puts it back deliberately.
    //
    // Stage 1 captures what the parser produced -- a short path, byte
    // extraction and opcode comparison only. Stage 2 does the range checks
    // and the packing on registered values. Cost is one extra cycle of
    // latency per message, against roughly 2816 cycles between messages.
    // =================================================================
    logic                 s1_valid;
    msg_kind_t            s1_kind;
    logic [PAYLOAD_W-1:0] s1_f0, s1_f1, s1_f2;
    logic [BURST_PIX_PER_MSG-1:0][BURST_PIX_W-1:0] s1_px;
    logic                 s1_frame_err;

    // THE VERDICT, computed in stage 1 and REGISTERED.
    //
    // These used to be evaluated in stage 2, on the registered s1_f* values,
    // and the result drove accept -- which is the clock enable for
    // out_payload, out_valid, out_kind and (through hdr_accept) pix_left in
    // rx_burst_ctrl. At 130 MHz that closed. At 256 MHz it did not: the path
    // s1_f1 -> br_extent_ok -> accept -> ~100 flop enables measured 5.723 ns
    // against a 3.906 ns period, 8 logic levels and 60% routing, and
    // Performance_ExplorePostRoutePhysOpt moved the worst path by 0.06 ns.
    //
    // Nothing about the pipeline changed to fix it. The checks simply moved
    // to the OTHER SIDE of the same register boundary: they are now computed
    // on the parser's combinational outputs, in the same cycle those are
    // captured, so stage 2 sees a registered verdict and accept collapses to
    // one AND gate.
    // STAGE 2 -- the range checks get their own register.
    //
    // The two-stage version computed them on the parser's combinational
    // outputs and registered the verdict into stage 1. That fixed the
    // accept cone but moved the problem in front of the stage-1 flop:
    // msg_buf -> rx_msg_parser -> range checks -> s1_ok measured
    // -1.381 ns at 256 MHz. Splitting them out gives each piece its own
    // short path:
    //
    //   stage 1  capture the parser's fields
    //   stage 2  range checks on the REGISTERED fields -> s2_ok
    //   stage 3  accept, packing mux -> out_payload
    //
    // Cost is one more cycle of latency per message, against roughly 5632
    // cycles between messages, and ~350 flops for the duplicated fields.
    logic                 s2_valid;
    msg_kind_t            s2_kind;
    logic [PAYLOAD_W-1:0] s2_f0, s2_f1, s2_f2;
    logic [BURST_PIX_PER_MSG-1:0][BURST_PIX_W-1:0] s2_px;
    logic                 s2_frame_err;
    // s2_ok is no longer a flop. The extent verdict is registered on its
    // OWN flop so its carry chain terminates there; s2_ok is recombined
    // combinationally at the point of use. See ACCEPT / REJECT below.
    logic                 s2_sem_pre;   // kind mux over the cheap terms
    logic                 s2_needs_ext; // this kind consumes the extent check
    logic                 s2_ext_row_ok; // row extent verdict
    logic                 s2_ext_col_ok; // column extent verdict
    wire                  s2_ext_ok = s2_ext_row_ok && s2_ext_col_ok;
    wire                  s2_ok = s2_sem_pre && (!s2_needs_ext || s2_ext_ok);
    logic                 s2_unknown; // kind not recognised at all

    // Each stage may only advance when the one after it is free, otherwise
    // a held message would be overwritten.
    // ONE advance condition for the WHOLE pipeline, not one per stage.
    //
    // The first version of this had stage 1 advance on s2_free and stage 2
    // latch on s3_free. Those differ whenever s2_valid is low and s3_free
    // is low: stage 1 would load a new message, stage 2 would not take it,
    // and the next cycle overwrote it. Nine messages were silently lost in
    // a 6000-cycle run.
    //
    // Moving the whole pipeline together is both correct and simpler: a
    // message can only enter when everything ahead of it will also step.
    logic s3_free;      // the output register can take a new message
    logic pipe_adv;     // the entire pipeline steps this cycle
    assign s3_free  = !out_valid || out_ready;
    assign pipe_adv = !s2_valid || s3_free;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s1_valid     <= 1'b0;
            s1_kind      <= MSG_UNKNOWN;
            s1_f0        <= '0;
            s1_f1        <= '0;
            s1_f2        <= '0;
            s1_px        <= '0;
            s1_frame_err <= 1'b0;
        end else begin
            s1_frame_err <= frame_err;

            if (pipe_adv) begin
                s1_valid <= msg_valid;
                if (msg_valid) begin
                    s1_kind <= msg_kind;
                    s1_f0   <= field0;
                    s1_f1   <= field1;
                    s1_f2   <= field2;
                    s1_px   <= burst_pixels;
                end
            end
        end
    end

    // =================================================================
    // STAGE 2 -- the verdict, on the REGISTERED fields
    //
    // The checks below read s1_f*, and their result is captured here. The
    // verdict travels WITH the fields it was computed from: same cycle,
    // same source registers, so there is no window in which s2_ok
    // describes a different message than s2_f*.
    // =================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s2_valid     <= 1'b0;
            s2_kind      <= MSG_UNKNOWN;
            s2_f0        <= '0;
            s2_f1        <= '0;
            s2_f2        <= '0;
            s2_px        <= '0;
            s2_frame_err <= 1'b0;
            s2_sem_pre   <= 1'b0;
            s2_needs_ext <= 1'b0;
            s2_ext_row_ok <= 1'b0;
            s2_ext_col_ok <= 1'b0;
            s2_unknown   <= 1'b0;
        end else if (pipe_adv) begin
            s2_valid     <= s1_valid;
            s2_frame_err <= s1_frame_err;
            if (s1_valid) begin
                s2_kind    <= s1_kind;
                s2_f0      <= s1_f0;
                s2_f1      <= s1_f1;
                s2_f2      <= s1_f2;
                s2_px      <= s1_px;
                // Three flops, not one. s2_ext_ok is the endpoint of the
                // extent carry chain -- that is the whole point.
                s2_sem_pre   <= sem_pre;
                s2_needs_ext <= needs_extent;
                s2_ext_row_ok <= br_extent_row_ok;
                s2_ext_col_ok <= br_extent_col_ok;
                s2_unknown   <= sem_unknown;
            end
        end
    end

    // =================================================================
    // SEMANTIC CHECKS -- on STAGE 1's REGISTERED fields
    //
    // s1_f0/f1/f2, with the result registered into s2_ok. This logic has a
    // flop on each side of it: nothing else shares its cycle.
    // =================================================================

    // Fits the RGF port and word aligned. Whether the address is a POPULATED
    // register is the RGF's business -- its read decode has a default arm.
    logic rgf_addr_ok;
    assign rgf_addr_ok = (s1_f0[PAYLOAD_W-1:ADDR_W_RGF] == '0) &&
                         (s1_f0[1:0] == 2'd0);

    logic pix_coord_ok;
    assign pix_coord_ok = (s1_f0 < PAYLOAD_W'(IMG_HEIGHT)) &&
                          (s1_f1 < PAYLOAD_W'(IMG_WIDTH));

    // NARROW, not PAYLOAD_W.
    //
    // These were computed at PAYLOAD_W (24 bits) so the sums could not
    // wrap. That is the right concern and the wrong width: at 256 MHz the
    // two 24-bit adders synthesised as SIX CHAINED CARRY4 blocks spanning
    // six slices, 1.34 ns of carry plus 0.72 ns of routing off the far end
    // of the chain, and s1_f0 -> s2_ok was the last failing path at
    // -0.999 ns.
    //
    // EXT_W is derived, and the bound is provable rather than assumed.
    // br_extent_ok only means anything when br_addr_ok and br_dims_ok also
    // hold -- br_ok is their conjunction -- so on any path where the result
    // is used:
    //
    //     base_row  <= IMG_HEIGHT-1      (from br_addr_ok)
    //     s1_f1     <= IMG_HEIGHT        (from br_dims_ok)
    //     sum       <= 2*IMG_HEIGHT - 1
    //
    // One bit above $clog2(2*max) therefore cannot wrap. At 256x256 that
    // is 11 bits against 24, and the carry chain drops from six CARRY4 to
    // two. The comparison result is identical for every input where it is
    // consumed; where it is not consumed, br_ok is already false.
    localparam int EXT_W = $clog2(2*(IMG_HEIGHT > IMG_WIDTH ?
                                     IMG_HEIGHT : IMG_WIDTH) + 1) + 1;

    logic [PAYLOAD_W-1:0] br_base_row_full, br_base_col_full;
    logic br_dims_ok, br_addr_ok, br_extent_ok, br_ok;

    // IMG_WIDTH is a power of two so both of these collapse to wiring;
    // written as arithmetic so a non-power-of-two image stays correct.
    assign br_base_row_full = s1_f0 / PAYLOAD_W'(IMG_WIDTH);
    assign br_base_col_full = s1_f0 % PAYLOAD_W'(IMG_WIDTH);

    assign br_dims_ok   = (s1_f1 != '0) && (s1_f1 <= PAYLOAD_W'(IMG_HEIGHT)) &&
                          (s1_f2 != '0) && (s1_f2 <= PAYLOAD_W'(IMG_WIDTH));
    assign br_addr_ok   = (s1_f0 < PAYLOAD_W'(TOTAL_PIXELS));
    // SPLIT, not ANDed here. These are two INDEPENDENT add-compares, each
    // with its own carry chain, and merging them before the flop left one
    // LUT plus its routing between the carry-out and s2_ext_ok.D -- worth
    // -0.210 ns on its own once everything else had been cleared.
    //
    // Each verdict now gets its own flop and they are ANDed a stage later,
    // inside accept, where there is room for the extra input at no depth
    // cost. br_extent_ok survives only as the reference for the assertions.
    logic br_extent_row_ok, br_extent_col_ok;
    assign br_extent_row_ok =
        ((EXT_W'(br_base_row_full) + EXT_W'(s1_f1)) <= EXT_W'(IMG_HEIGHT));
    assign br_extent_col_ok =
        ((EXT_W'(br_base_col_full) + EXT_W'(s1_f2)) <= EXT_W'(IMG_WIDTH));
    assign br_extent_ok = br_extent_row_ok && br_extent_col_ok;
    assign br_ok        = br_dims_ok && br_addr_ok && br_extent_ok;

    // The legacy {Rnnn,Cnnn,Vnnn} form is gone, and with it the ASCII
    // decimal decode that lived here: two multiplies and a three-term sum
    // per field, feeding the payload packing mux. That was the critical
    // path at 130 MHz -- s1_f2 to out_payload, twelve logic levels. The
    // message is not in the final project set, so it was removed rather
    // than pipelined.

    // =================================================================
    // THE PER-KIND VERDICT, evaluated on the INCOMING message
    //
    // Selects which of the checks above applies, keyed on s1_kind. The
    // result is registered into s2_ok, so stage 3's accept is a decode of
    // registered state rather than the end of a comparison cone.
    // =================================================================
    // -----------------------------------------------------------------
    // LATE SIGNAL LAST.
    //
    // br_extent_ok is the output of the extent carry chain and is the
    // last thing in this module to settle. Feeding it into the kind mux
    // put TWO LUT6 levels between the carry-out and s2_ok:
    //
    //   s1_f0 -> LUT2 -> 3x CARRY4 -> LUT6 (br_ok) -> LUT6 (mux) -> s2_ok.D
    //
    // measured at -0.500 ns, the WNS path in rx_classifier.
    //
    // Only MSG_BURST_READ consumes the extent check. So the mux runs on
    // the EARLY terms only -- all of which are comparisons against
    // constants and settle long before the carry chain -- and the late
    // signal joins at a single gate afterwards:
    //
    //   carry-out -> LUT3 -> s2_ok.D
    //
    // One level instead of two, the mux now runs in parallel with the
    // carry chain instead of behind it, and accept's cone is untouched
    // (it must stay one gate: it drives ~100 flop enables).
    //
    // The verdict is unchanged for every kind -- see a_burst_read_verdict.
    // -----------------------------------------------------------------
    logic sem_ok, sem_unknown, sem_pre, needs_extent;

    always_comb begin : verdict
        sem_pre      = 1'b0;
        sem_unknown  = 1'b0;
        needs_extent = 1'b0;
        unique case (s1_kind)
            MSG_PIX_WRITE  : sem_pre = 1'b1;          // no semantic constraint
            MSG_BURST_DATA : sem_pre = 1'b1;          // raw pixels
            MSG_BURST_HDR  : sem_pre = br_dims_ok;
            MSG_REG_READ   : sem_pre = rgf_addr_ok;
            MSG_REG_WRITE  : sem_pre = rgf_addr_ok;
            MSG_PIX_READ   : sem_pre = pix_coord_ok;
            MSG_BURST_READ : begin
                // br_ok minus its extent term; the extent joins below.
                sem_pre      = br_dims_ok && br_addr_ok;
                needs_extent = 1'b1;
            end
            default        : sem_unknown = 1'b1;     // MSG_UNKNOWN
        endcase
    end : verdict

    // sem_ok is retained for the assertions only -- it is NOT in the
    // datapath any more and synthesis will remove it. The real combination
    // happens one stage later, on registered bits.
    assign sem_ok = sem_pre && (!needs_extent || br_extent_ok);

    // =================================================================
    // ACCEPT / REJECT -- now a decode of REGISTERED state
    //
    // One AND gate and one inverter, against eight logic levels before.
    // accept still drives ~100 flop enables, but it is now sourced from a
    // flop rather than from the end of a comparison cone.
    // =================================================================
    // -----------------------------------------------------------------
    // WHY THE EXTENT VERDICT GETS ITS OWN FLOP.
    //
    // Registering the whole of sem_ok into a single s2_ok put the extent
    // carry chain three LUT levels away from its flop:
    //
    //   s1_f0 -> LUT2 -> CARRY4 -> LUT4 -> LUT6 -> LUT6 -> s2_ok.D
    //                            \____ 1.43 ns of logic+route ____/
    //
    // measured at -0.507 ns. Reassociating the boolean expression did not
    // help and could not have: the two forms are logically identical, so
    // synthesis re-flattens and re-factors to its own cost model. Only a
    // register is a barrier the tool will not cross.
    //
    // br_extent_ok is also not a single late signal -- it is the AND of a
    // ROW comparison and a COLUMN comparison, each with its own carry
    // chain, so there is always a merge gate after them.
    //
    // Splitting the verdict across three flops terminates the chain one
    // LUT after the CARRY4 and costs NOTHING in depth downstream: accept
    // was a 3-input gate (s2_valid, s2_ok, s2_unknown) and is now a
    // 5-input one (s2_valid, s2_sem_pre, s2_needs_ext, s2_ext_ok,
    // s2_unknown). Both are a single LUT6. Pipeline latency is unchanged.
    //
    // The two extent comparisons are split further onto their own flops,
    // so accept's input set is now exactly six:
    //
    //   s2_valid, s2_unknown, s2_sem_pre, s2_needs_ext,
    //   s2_ext_row_ok, s2_ext_col_ok
    //
    // Six is a LUT6. That is the budget fully spent -- accept drives ~100
    // flop enables and MUST stay one level. A seventh term forces it to
    // two and undoes everything above.
    // -----------------------------------------------------------------
    logic accept, reject;

    assign accept = s2_valid && s2_ok && !s2_unknown;
    assign reject = s2_valid && (!s2_ok || s2_unknown);

    // STAGE 2 values, so hdr_accept and the dimensions it carries come from
    // the same register set as accept itself. rx_burst_ctrl samples all
    // three on the same cycle.
    assign hdr_accept = accept && (s2_kind == MSG_BURST_HDR);
    assign hdr_height = s2_f1[9:0];
    assign hdr_width  = s2_f2[9:0];

    // =================================================================
    // PAYLOAD PACKING
    //
    // Named struct fields, not hand-written bit ranges. The memory side
    // unpacks with the same types from msg_format_pkg.
    // =================================================================
    msg_payload_t packed_payload;

    always_comb begin : pack
        pl_pix_write_t  pw;
        pl_burst_hdr_t  bh;
        pl_burst_data_t bd;
        pl_reg_write_t  rw;
        pl_reg_read_t   rr;
        pl_pix_read_t   pr;
        pl_burst_read_t br;

        // Every local is cleared first. Without this the case arms leave
        // the unused structs unassigned on some paths and the tool infers
        // latches for them.
        pw = '0; bh = '0; bd = '0; rw = '0;
        rr = '0; pr = '0; br = '0;

        packed_payload = '0;

        unique case (s2_kind)

            MSG_PIX_WRITE: begin
                pw.addr        = s2_f0;
                pw.pixel       = s2_f1;
                packed_payload = msg_payload_t'(pw);
            end

            MSG_BURST_HDR: begin
                bh.base_addr   = s2_f0;
                bh.height      = s2_f1[9:0];
                bh.width       = s2_f2[9:0];
                packed_payload = msg_payload_t'(bh);
            end

            MSG_BURST_DATA: begin
                bd.px0         = s2_px[0];
                bd.px1         = s2_px[1];
                bd.px2         = s2_px[2];
                bd.px3         = s2_px[3];
                packed_payload = msg_payload_t'(bd);
            end

            // Data is bytes 8,9,13,14 -- the low two payload bytes of each
            // V group. The don't-care bytes the spec shows as 0 are
            // field[23:16] and are simply not used.
            MSG_REG_WRITE: begin
                rw.addr        = s2_f0[ADDR_W_RGF-1:0];
                rw.data        = {s2_f1[15:0], s2_f2[15:0]};
                packed_payload = msg_payload_t'(rw);
            end

            MSG_REG_READ: begin
                rr.addr        = s2_f0[ADDR_W_RGF-1:0];
                packed_payload = msg_payload_t'(rr);
            end

            MSG_PIX_READ: begin
                pr.row         = s2_f0[9:0];
                pr.col         = s2_f1[9:0];
                packed_payload = msg_payload_t'(pr);
            end

            MSG_BURST_READ: begin
                br.base_addr   = s2_f0;
                br.height      = s2_f1[9:0];
                br.width       = s2_f2[9:0];
                packed_payload = msg_payload_t'(br);
            end


            default: packed_payload = '0;

        endcase
    end : pack

    // =================================================================
    // OUTPUT REGISTER -- HELD UNTIL TAKEN
    // =================================================================
    // Busy while EITHER stage holds something, so rx_mac stalls a completed
    // frame rather than overwriting one still in the pipeline.
    // Three stages deep now. rx_mac needs no change: its whole response is
    // `next_state = stall ? MAC_CHK : MAC_DONE`, which holds the frame in
    // its buffer for as long as this is high, one cycle or five.
    assign out_busy = s1_valid || s2_valid || out_valid;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            out_valid        <= 1'b0;
            out_kind         <= MSG_UNKNOWN;
            out_payload      <= '0;
            classifier_error <= 1'b0;
        end else begin
            // frame_err is NOT gated on anything: a framing fault is worth
            // reporting whatever mode the receiver is in.
            classifier_error <= reject || s2_frame_err;

            if (out_valid && out_ready) out_valid <= 1'b0;

            if (accept) begin
                out_valid   <= 1'b1;
                out_kind    <= s1_kind;   // STAGE 1 value: msg_kind has moved on
                out_payload <= packed_payload;
            end
        end
    end

`ifndef SYNTHESIS
    // A frame must never be taken into a stage that is still occupied by a
    // message the next stage has not taken. With three stages a frame MAY
    // arrive while out_valid is high -- that is the pipeline being full,
    // not an overwrite -- so the property is stated per stage instead.
    // This is what rx_mac's stall has to guarantee.
    a_no_overwrite_s1: assert property (
        @(posedge clk) disable iff (!rst_n)
        (msg_valid && pipe_adv) |-> (!s2_valid || s3_free)
    ) else $error("%m: frame taken into stage 1 while stage 2 was blocked");

    a_no_overwrite_s2: assert property (
        @(posedge clk) disable iff (!rst_n)
        (s1_valid && pipe_adv) |-> (!s2_valid || s3_free)
    ) else $error("%m: stage 1 advanced while stage 2 was blocked");

    // Nothing is ever dropped: a valid message in a stage stays there until
    // the next stage can take it.
    a_s2_holds: assert property (
        @(posedge clk) disable iff (!rst_n)
        (s2_valid && !pipe_adv) |=> s2_valid
    ) else $error("%m: stage 2 lost a message while stage 3 was blocked");

    // Held output is stable.
    a_hold_stable: assert property (
        @(posedge clk) disable iff (!rst_n)
        (out_valid && !out_ready) |=> (out_valid && $stable(out_payload) &&
                                       $stable(out_kind))
    ) else $error("%m: held message changed before it was taken");

    // Accept and reject are exclusive.
    // The restructure above must not change the verdict. br_ok is retained
    // solely as the reference for this check.
    a_burst_read_verdict: assert property (
        @(posedge clk) disable iff (!rst_n)
        (s1_kind == MSG_BURST_READ) |-> (sem_ok == br_ok)
    ) else $error("%m: burst-read verdict changed by the late-signal split");

    a_accept_xor_reject: assert property (
        @(posedge clk) disable iff (!rst_n)
        !(accept && reject)
    ) else $error("%m: message both accepted and rejected");
`endif

endmodule : rx_classifier