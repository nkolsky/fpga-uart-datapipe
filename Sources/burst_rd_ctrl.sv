// burst_rd_ctrl.sv
// ----------------
// Image Burst Read controller, 100 MHz memory domain.
//
// Walks an H x W rectangle in strict row-major order, reads one pixel per
// SRAM access, packs four pixels per reply message, and hands each message
// to the reply path across a held request/accept handshake.
//
//   messages emitted = ceil(H*W / 4) = (H*W + 3) >> 2
//
// -----------------------------------------------------------------------
// ONE BURST IS ONE COHERENT MEMORY TRANSACTION
// -----------------------------------------------------------------------
// Memory ownership is requested ONCE, before the first pixel, and held
// until the last real pixel has been captured and its message accepted.
// Nothing else touches memory in between: no writes, no full-image read,
// no single-pixel read.
//
// This is a deliberate choice of coherence over concurrency. The reply
// stream carries NO coordinates -- the host reconstructs position purely
// from message order -- so a region that changed underneath the walk would
// be undetectable at the far end. Holding ownership makes the returned
// rectangle a consistent snapshot by construction.
//
// THE COST, WHICH IS REAL AND MUST BE DOCUMENTED:
//
//   A 256x256 Burst Read reads 65,536 pixels and emits 16,384 messages.
//   Ownership is held across the whole of that, including every TX stall,
//   so writes and full-frame reads are blocked for roughly 0.36 s at
//   8 Mbaud. Pending writes are deferred, not lost -- mem_interlock
//   holds them -- but the host must not expect a write issued during a
//   large burst to complete promptly.
//
// Ownership is released after the final message is ACCEPTED rather than
// after the final byte leaves the UART. The SRAM is finished with well
// before that, but releasing earlier would let a write land between the
// last capture and the last handoff, which is exactly the incoherence
// this design is avoiding.
//
// There is no deadlock in holding ownership across TX backpressure. The
// reply path stalls on tx_seq_busy and mac_busy; tx_seq_busy belongs to
// the full-image path, which cannot start while this controller owns
// memory, and cannot already be running because the grant condition
// includes !img_in_flight. So the thing being waited on never depends on
// the thing being held.
//
// -----------------------------------------------------------------------
// ADDRESSING -- mirrors rx_burst_ctrl on the write side
// -----------------------------------------------------------------------
//     pixel_row  = base_row + burst_row
//     pixel_col  = base_col + burst_col
//     pixel_idx  = pixel_row * IMG_W + pixel_col
//     word_addr  = pixel_idx >> 2
//     lane       = pixel_idx[1:0]          lane 0 = MOST significant byte
//
// Row-major: H=2, W=3 from (0,0) visits (0,0)(0,1)(0,2)(1,0)(1,1)(1,2).
//
// This is a PER-PIXEL walk, not the per-word walk rom_sequencer uses. A
// sub-rectangle's rows are generally not word-aligned and W is generally
// not a multiple of four, so the four pixels of a message are not in
// general four lanes of one word. Reading one pixel at a time costs four
// SRAM accesses per message instead of one; at 8 Mbaud the UART needs
// ~22 us per message and the SRAM is idle for effectively all of it.
//
// -----------------------------------------------------------------------
// PADDING
// -----------------------------------------------------------------------
// When H*W is not a multiple of four the final message is padded to four
// slots with RGB = 0,0,0. A padded slot:
//
//   * performs NO SRAM access      -- the FSM never enters B_ADDR for it
//   * does NOT advance the region address -- burst_row/burst_col only move
//                                     in B_CAP, which only real pixels reach
//   * cannot wrap into another row -- same reason; the walk has already
//                                     terminated on pix_count == total
//
// Zero-fill happens in B_PACK by clearing the remaining slots outright,
// so padding is a property of the buffer, not of the address generator.

`timescale 1ns/1ps

module burst_rd_ctrl
    import memory_pkg::*;
    import rx_burst_pkg::*;
#(
    parameter int DIM_W = 10,
    parameter int IMG_W = IMG_WIDTH,
    // Cycles held in B_PACK before offering a message.
    //
    // chip_top feeds each message to cdc_cmd_sync, which asserts that
    // consecutive src_valid pulses are at least MIN_SPACING_SRC_CYCLES = 8
    // source cycles apart, warning that "a command may be lost" otherwise.
    //
    // Without this guard the spacing is emergent rather than guaranteed:
    // accept round trip (~5 cycles) + B_ADDR + B_WAIT + B_CAP + B_PACK
    // (4 cycles) = about 10, i.e. only two cycles of margin, and that
    // margin depends on synchroniser depths in two different modules. The
    // guard makes the minimum gap 12 by construction and stops the
    // property from being an accident of timing.
    //
    // Cost: 4 cycles per message. A full-frame burst emits 16,384
    // messages, so 65,536 cycles = 0.65 ms against a ~0.35 s transfer.
    parameter int PACK_GUARD = 4
)(
    input  logic clk,                    // CLK100MHZ
    input  logic rst_n,                  // sync_rst_n

    // ---- request, validated and already in this domain -----------------
    input  logic             req_valid,  // one-cycle strobe
    input  logic [DIM_W-1:0] req_base_row,
    input  logic [DIM_W-1:0] req_base_col,
    input  logic [DIM_W-1:0] req_height,
    input  logic [DIM_W-1:0] req_width,

    // ---- memory ownership, via the shared read client ------------------
    output logic brd_rd_req,             // level, held for the WHOLE burst
    input  logic brd_rd_gnt,
    output logic brd_rd_done,            // one cycle, releases ownership

    // ---- SRAM read port -------------------------------------------------
    output logic                        sram_rd_en,
    output logic [SRAM_ADDR_WIDTH-1:0]  sram_rd_addr,
    input  logic [31:0]                 red_data,
    input  logic [31:0]                 green_data,
    input  logic [31:0]                 blue_data,

    // ---- packed reply, held handshake -----------------------------------
    output logic                                          msg_valid,
    input  logic                                          msg_accept,
    output logic [BURST_PIX_PER_MSG-1:0][BURST_PIX_W-1:0] msg_pixels,

    // ---- status ----------------------------------------------------------
    output logic busy,
    output logic req_overrun            // sticky: request arrived while busy
);

    typedef enum logic [2:0] {
        B_IDLE, B_REQ, B_ADDR, B_WAIT, B_CAP, B_PACK, B_SEND, B_DONE
    } bstate_t;

    bstate_t state;

    logic [DIM_W-1:0] base_row_q, base_col_q;
    logic [DIM_W-1:0] h_q, w_q;
    logic [DIM_W-1:0] burst_row, burst_col;

    // Pixel accounting. IMG_HEIGHT*IMG_WIDTH = 65536 needs 17 bits to hold
    // the count inclusive of the total itself.
    localparam int CNT_W = 17;
    logic [CNT_W-1:0] pix_count;   // real pixels captured so far
    logic [CNT_W-1:0] pix_total;   // H*W

    logic [BURST_SLOT_W-1:0] slot;
    logic [1:0]              lane_q;
    logic [2:0]              guard_cnt;

    logic [15:0] pixel_idx;
    logic [DIM_W-1:0] pixel_row, pixel_col;

    assign busy       = (state != B_IDLE);
    assign brd_rd_req = (state == B_REQ);
    assign msg_valid  = (state == B_SEND);

    assign pixel_row = base_row_q + burst_row;
    assign pixel_col = base_col_q + burst_col;
    assign pixel_idx = 16'(pixel_row) * 16'(IMG_W) + 16'(pixel_col);

    assign sram_rd_addr = pixel_idx[SRAM_ADDR_WIDTH+1:2];
    assign sram_rd_en   = (state == B_ADDR);

    // Lane 0 is the MOST significant byte -- same orientation as
    // rom_sequencer (pixels[0] <= red_data[31:24]) and sram_wr_ctrl
    // (wr_be = 1 << (3 - lane)).
    function automatic logic [7:0] lane_byte(input logic [31:0] word,
                                             input logic [1:0]  lane);
        case (lane)
            2'd0: lane_byte = word[31:24];
            2'd1: lane_byte = word[23:16];
            2'd2: lane_byte = word[15: 8];
            2'd3: lane_byte = word[ 7: 0];
        endcase
    endfunction

    // True once every real pixel of the region has been captured.
    logic all_captured;
    assign all_captured = (pix_count == pix_total);

    // The message currently being assembled is full.
    logic slot_full;
    assign slot_full = (slot == BURST_SLOT_W'(BURST_PIX_PER_MSG - 1));

    integer i;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state       <= B_IDLE;
            base_row_q  <= '0;
            base_col_q  <= '0;
            h_q         <= '0;
            w_q         <= '0;
            burst_row   <= '0;
            burst_col   <= '0;
            pix_count   <= '0;
            pix_total   <= '0;
            slot        <= '0;
            lane_q      <= '0;
            guard_cnt   <= '0;
            brd_rd_done <= 1'b0;
            req_overrun <= 1'b0;
            for (i = 0; i < BURST_PIX_PER_MSG; i++) msg_pixels[i] <= '0;
        end
        else begin
            brd_rd_done <= 1'b0;

            // A request arriving mid-burst is rejected and flagged. The
            // first region is never abandoned part-way: doing so would emit
            // a truncated, uncoordinated message stream that the host has
            // no way to resynchronise.
            if (req_valid && (state != B_IDLE))
                req_overrun <= 1'b1;

            unique case (state)

            B_IDLE: if (req_valid) begin
                base_row_q <= req_base_row;
                base_col_q <= req_base_col;
                h_q        <= req_height;
                w_q        <= req_width;
                pix_total  <= CNT_W'(req_height) * CNT_W'(req_width);
                burst_row  <= '0;
                burst_col  <= '0;
                pix_count  <= '0;
                slot       <= '0;
                for (i = 0; i < BURST_PIX_PER_MSG; i++) msg_pixels[i] <= '0;
                state      <= B_REQ;
            end

            // Ownership is taken ONCE here and not released until B_DONE.
            B_REQ: if (brd_rd_gnt) begin
                lane_q <= pixel_idx[1:0];
                state  <= B_ADDR;
            end

            B_ADDR: state <= B_WAIT;      // rd_en asserted this cycle

            B_WAIT: state <= B_CAP;       // one-cycle registered SRAM read

            // Capture one real pixel, then advance the region position.
            // Padding never reaches this state, which is what guarantees
            // it performs no read and moves no address.
            B_CAP: begin
                msg_pixels[slot] <= {lane_byte(red_data,   lane_q),
                                     lane_byte(green_data, lane_q),
                                     lane_byte(blue_data,  lane_q)};
                pix_count <= pix_count + CNT_W'(1);

                // Row-major advance. Wrapping to the next row happens only
                // at the region's own right edge, not the image's.
                if (burst_col == (w_q - DIM_W'(1))) begin
                    burst_col <= '0;
                    burst_row <= burst_row + DIM_W'(1);
                end
                else begin
                    burst_col <= burst_col + DIM_W'(1);
                end

                // Finish the message when it is full, or when the region
                // has run out of real pixels -- whichever comes first.
                if (slot_full || (pix_count + CNT_W'(1) == pix_total)) begin
                    state <= B_PACK;
                end
                else begin
                    slot   <= slot + BURST_SLOT_W'(1);
                    lane_q <= next_lane();
                    state  <= B_ADDR;
                end
            end

            // Zero-fill any slots the region did not reach. On a full
            // message this clears nothing and costs one cycle.
            B_PACK: begin
                if (!slot_full) begin
                    for (i = 0; i < BURST_PIX_PER_MSG; i++) begin
                        if (BURST_SLOT_W'(i) > slot) msg_pixels[i] <= '0;
                    end
                end
                // Hold for PACK_GUARD cycles so the gap between successive
                // cdc_cmd_sync commands is guaranteed, not emergent.
                if (guard_cnt == 3'(PACK_GUARD - 1)) begin
                    guard_cnt <= '0;
                    state     <= B_SEND;
                end
                else begin
                    guard_cnt <= guard_cnt + 3'd1;
                end
            end

            // Hold the packed message until the reply path takes it. This
            // can stall for a long time; ownership is retained throughout.
            B_SEND: if (msg_accept) begin
                if (all_captured) begin
                    state <= B_DONE;
                end
                else begin
                    slot      <= '0;
                    lane_q    <= pixel_idx[1:0];
                    guard_cnt <= '0;
                    state     <= B_ADDR;
                end
            end

            // Release ownership only now: after the last real pixel was
            // captured AND its message was accepted.
            B_DONE: begin
                brd_rd_done <= 1'b1;
                state       <= B_IDLE;
            end

            default: state <= B_IDLE;
            endcase
        end
    end

    // Lane of the NEXT pixel in the walk, computed from the position the
    // B_CAP branch is about to install. Kept as a function so the two call
    // sites cannot drift.
    function automatic logic [1:0] next_lane();
        logic [DIM_W-1:0] nrow, ncol;
        logic [15:0]      nidx;
        if (burst_col == (w_q - DIM_W'(1))) begin
            ncol = '0;
            nrow = burst_row + DIM_W'(1);
        end
        else begin
            ncol = burst_col + DIM_W'(1);
            nrow = burst_row;
        end
        // Each operand is widened BEFORE the addition. Casting the sum
        // instead makes the add itself 16 bits wide against 10-bit operands,
        // which the tool warns about at every term.
        nidx = (16'(base_row_q) + 16'(nrow)) * 16'(IMG_W)
             + (16'(base_col_q) + 16'(ncol));
        return nidx[1:0];
    endfunction

`ifndef SYNTHESIS
    // Ownership must be held for every SRAM access of the burst.
    a_rd_needs_ownership: assert property (
        @(posedge clk) disable iff (!rst_n)
        sram_rd_en |-> (state == B_ADDR)
    ) else $error("%m: SRAM read outside B_ADDR");

    // Exactly one release per burst, and only from B_DONE.
    a_done_from_done: assert property (
        @(posedge clk) disable iff (!rst_n)
        brd_rd_done |-> $past(state == B_DONE)
    ) else $error("%m: ownership released outside B_DONE");

    // The region walk never exceeds the requested pixel count.
    a_no_overrun_walk: assert property (
        @(posedge clk) disable iff (!rst_n)
        (state != B_IDLE) |-> (pix_count <= pix_total)
    ) else $error("%m: walked past the end of the region");

    // A held message must not change while it is being offered.
    a_msg_stable: assert property (
        @(posedge clk) disable iff (!rst_n)
        (msg_valid && !msg_accept) |=> $stable(msg_pixels)
    ) else $error("%m: reply payload changed while awaiting accept");

    // ...and must never be withdrawn unaccepted.
    // One message offered per assembly: msg_valid must go low between
    // messages so chip_top's edge detector produces exactly one
    // cdc_cmd_sync command per packed message. This is what makes a
    // stalled TX incapable of causing a duplicate submission.
    a_msg_valid_gap: assert property (
        @(posedge clk) disable iff (!rst_n)
        $fell(msg_valid) |=> !msg_valid
    ) else $error("%m: msg_valid re-asserted without a gap");

    a_no_msg_loss: assert property (
        @(posedge clk) disable iff (!rst_n)
        (msg_valid && !msg_accept) |=> msg_valid
    ) else $error("%m: msg_valid dropped without an accept");
`endif

endmodule : burst_rd_ctrl
