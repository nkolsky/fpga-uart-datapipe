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
//   legacy {Rnnn,...}      every payload byte is ASCII '0'..'9'
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

    // Stage 1 may only take a frame when stage 2 is free, otherwise the
    // held message would be overwritten.
    logic s2_free;
    assign s2_free = !out_valid || out_ready;

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

            if (s2_free) begin
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
    // SEMANTIC CHECKS -- on the REGISTERED fields
    // =================================================================

    // Fits the RGF port and word aligned. Whether the address is a POPULATED
    // register is the RGF's business -- its read decode has a default arm.
    logic rgf_addr_ok;
    assign rgf_addr_ok = (s1_f0[PAYLOAD_W-1:ADDR_W_RGF] == '0) &&
                         (s1_f0[1:0] == 2'd0);

    logic pix_coord_ok;
    assign pix_coord_ok = (s1_f0 < PAYLOAD_W'(IMG_HEIGHT)) &&
                          (s1_f1 < PAYLOAD_W'(IMG_WIDTH));

    // Decomposed at full width so the sum cannot wrap. IMG_WIDTH is a power
    // of two so both divisions collapse to wiring; written as arithmetic so
    // a non-power-of-two image stays correct.
    logic [PAYLOAD_W-1:0] br_base_row_full, br_base_col_full;
    logic br_dims_ok, br_addr_ok, br_extent_ok, br_ok;

    assign br_base_row_full = s1_f0 / PAYLOAD_W'(IMG_WIDTH);
    assign br_base_col_full = s1_f0 % PAYLOAD_W'(IMG_WIDTH);

    assign br_dims_ok   = (s1_f1 != '0) && (s1_f1 <= PAYLOAD_W'(IMG_HEIGHT)) &&
                          (s1_f2 != '0) && (s1_f2 <= PAYLOAD_W'(IMG_WIDTH));
    assign br_addr_ok   = (s1_f0 < PAYLOAD_W'(TOTAL_PIXELS));
    assign br_extent_ok = ((br_base_row_full + s1_f1) <= PAYLOAD_W'(IMG_HEIGHT)) &&
                          ((br_base_col_full + s1_f2) <= PAYLOAD_W'(IMG_WIDTH));
    assign br_ok        = br_dims_ok && br_addr_ok && br_extent_ok;

    // ---- legacy: ASCII digits ------------------------------------------
    function automatic logic is_digit(input logic [7:0] b);
        return (b >= ASCII_ZERO) && (b <= (ASCII_ZERO + 8'd9));
    endfunction

    function automatic int dec3(input logic [PAYLOAD_W-1:0] f);
        return (int'(f[23:16]) - int'(ASCII_ZERO)) * 100
             + (int'(f[15: 8]) - int'(ASCII_ZERO)) * 10
             + (int'(f[ 7: 0]) - int'(ASCII_ZERO));
    endfunction

    logic legacy_digits_ok;
    always_comb begin : check_digits
        legacy_digits_ok = 1'b1;
        for (int g = 0; g < 3; g++) begin
            logic [PAYLOAD_W-1:0] f;
            f = (g == 0) ? s1_f0 : (g == 1) ? s1_f1 : s1_f2;
            if (!is_digit(f[23:16])) legacy_digits_ok = 1'b0;
            if (!is_digit(f[15: 8])) legacy_digits_ok = 1'b0;
            if (!is_digit(f[ 7: 0])) legacy_digits_ok = 1'b0;
        end
    end : check_digits

    // =================================================================
    // ACCEPT / REJECT
    // =================================================================
    logic accept, reject;

    always_comb begin : decide
        accept = 1'b0;
        reject = 1'b0;

        if (s1_valid) begin
            unique case (s1_kind)
                MSG_LEGACY_RGF : begin accept =  legacy_digits_ok;
                                       reject = !legacy_digits_ok; end
                MSG_PIX_WRITE  : accept = 1'b1;   // no semantic constraint
                MSG_BURST_DATA : accept = 1'b1;   // raw pixels
                MSG_BURST_HDR  : begin accept =  br_dims_ok;
                                       reject = !br_dims_ok; end
                MSG_REG_READ   : begin accept =  rgf_addr_ok;
                                       reject = !rgf_addr_ok; end
                MSG_REG_WRITE  : begin accept =  rgf_addr_ok;
                                       reject = !rgf_addr_ok; end
                MSG_PIX_READ   : begin accept =  pix_coord_ok;
                                       reject = !pix_coord_ok; end
                MSG_BURST_READ : begin accept =  br_ok;
                                       reject = !br_ok; end
                default        : reject = 1'b1;   // MSG_UNKNOWN
            endcase
        end
    end : decide

    assign hdr_accept = accept && (s1_kind == MSG_BURST_HDR);
    assign hdr_height = s1_f1[9:0];
    assign hdr_width  = s1_f2[9:0];

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
        pl_legacy_t     lg;

        // Every local is cleared first. Without this the case arms leave
        // the unused structs unassigned on some paths and the tool infers
        // latches for them.
        pw = '0; bh = '0; bd = '0; rw = '0;
        rr = '0; pr = '0; br = '0; lg = '0;

        packed_payload = '0;

        unique case (s1_kind)

            MSG_PIX_WRITE: begin
                pw.addr        = s1_f0;
                pw.pixel       = s1_f1;
                packed_payload = msg_payload_t'(pw);
            end

            MSG_BURST_HDR: begin
                bh.base_addr   = s1_f0;
                bh.height      = s1_f1[9:0];
                bh.width       = s1_f2[9:0];
                packed_payload = msg_payload_t'(bh);
            end

            MSG_BURST_DATA: begin
                bd.px0         = s1_px[0];
                bd.px1         = s1_px[1];
                bd.px2         = s1_px[2];
                bd.px3         = s1_px[3];
                packed_payload = msg_payload_t'(bd);
            end

            // Data is bytes 8,9,13,14 -- the low two payload bytes of each
            // V group. The don't-care bytes the spec shows as 0 are
            // field[23:16] and are simply not used.
            MSG_REG_WRITE: begin
                rw.addr        = s1_f0[ADDR_W_RGF-1:0];
                rw.data        = {s1_f1[15:0], s1_f2[15:0]};
                packed_payload = msg_payload_t'(rw);
            end

            MSG_REG_READ: begin
                rr.addr        = s1_f0[ADDR_W_RGF-1:0];
                packed_payload = msg_payload_t'(rr);
            end

            MSG_PIX_READ: begin
                pr.row         = s1_f0[9:0];
                pr.col         = s1_f1[9:0];
                packed_payload = msg_payload_t'(pr);
            end

            MSG_BURST_READ: begin
                br.base_addr   = s1_f0;
                br.height      = s1_f1[9:0];
                br.width       = s1_f2[9:0];
                packed_payload = msg_payload_t'(br);
            end

            // Legacy payload is ASCII decimal, decoded here.
            MSG_LEGACY_RGF: begin
                lg.row         = 10'(dec3(s1_f0));
                lg.col         = 10'(dec3(s1_f1));
                lg.pixel       = 24'(dec3(s1_f2));
                packed_payload = msg_payload_t'(lg);
            end

            default: packed_payload = '0;

        endcase
    end : pack

    // =================================================================
    // OUTPUT REGISTER -- HELD UNTIL TAKEN
    // =================================================================
    // Busy while EITHER stage holds something, so rx_mac stalls a completed
    // frame rather than overwriting one still in the pipeline.
    assign out_busy = s1_valid || out_valid;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            out_valid        <= 1'b0;
            out_kind         <= MSG_UNKNOWN;
            out_payload      <= '0;
            classifier_error <= 1'b0;
        end else begin
            // frame_err is NOT gated on anything: a framing fault is worth
            // reporting whatever mode the receiver is in.
            classifier_error <= reject || s1_frame_err;

            if (out_valid && out_ready) out_valid <= 1'b0;

            if (accept) begin
                out_valid   <= 1'b1;
                out_kind    <= s1_kind;   // STAGE 1 value: msg_kind has moved on
                out_payload <= packed_payload;
            end
        end
    end

`ifndef SYNTHESIS
    // A frame must never arrive while a message is still held: the held
    // message would be silently overwritten. This is the property rx_mac's
    // stall has to guarantee.
    a_no_overwrite: assert property (
        @(posedge clk) disable iff (!rst_n)
        (msg_valid && s2_free) |-> !out_valid
    ) else $error("%m: frame accepted while a message was still held");

    // Held output is stable.
    a_hold_stable: assert property (
        @(posedge clk) disable iff (!rst_n)
        (out_valid && !out_ready) |=> (out_valid && $stable(out_payload) &&
                                       $stable(out_kind))
    ) else $error("%m: held message changed before it was taken");

    // Accept and reject are exclusive.
    a_accept_xor_reject: assert property (
        @(posedge clk) disable iff (!rst_n)
        !(accept && reject)
    ) else $error("%m: message both accepted and rejected");
`endif

endmodule : rx_classifier
