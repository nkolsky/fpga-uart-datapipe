`timescale 1ps/1ps

module tx_sequencer #(
    parameter IMG_WIDTH  = 256,
    parameter IMG_HEIGHT = 256
) (
    input  logic clk,
    input  logic rst_n,

    // control inputs
    input  logic        fifo_empty,
    input  logic        cts,            // active-low: 0 = clear to send
    input  logic        mac_busy,
    // ONE ENTRY FROM EACH CHANNEL FIFO -- a 32-bit SRAM word holding four
    // consecutive pixel values of that channel. A pixel is assembled by
    // taking the SAME byte lane from all three.
    //
    // rom_sequencer used to do this unpacking and push finished pixels. It
    // moved here because the three channels now arrive in separate FIFOs,
    // and because this is where pixels are consumed -- one per message.
    input  logic [31:0] fifo_rd_data_r,
    input  logic [31:0] fifo_rd_data_g,
    input  logic [31:0] fifo_rd_data_b,

    // control outputs
    output logic        msg_valid,
    output logic        fifo_pop,
    output logic        tx_img_done,
    output logic        busy,

    // message output to TX MAC
    output logic [127:0] msg,

    // row/col counters
    output logic [9:0]  col_cnt_out,
    output logic [9:0]  row_cnt_out
    
    
);

import tx_seq_pkg::*;

// -------------------------------------------------------------------------
// Internal signals
// -------------------------------------------------------------------------
state_t current_state, next_state;

// The three channel words of the current group, and which pixel of the four
// is being sent.
logic [31:0] word_r, word_g, word_b;
logic [1:0]  pix_idx;

/* verilator lint_off ASCRANGE */
logic [$clog2(IMG_WIDTH)-1:0]  col_cnt;
logic [$clog2(IMG_HEIGHT)-1:0] row_cnt;

logic [$clog2(IMG_WIDTH)-1:0]  col_latch;
logic [$clog2(IMG_HEIGHT)-1:0] row_latch;
/* verilator lint_on ASCRANGE */
logic [23:0]                   pixel_latch;

// last pixel flag (combinatorial)
/* verilator lint_off WIDTHEXPAND */
logic last_pixel;
assign last_pixel = (row_cnt == ($bits(row_cnt))'(IMG_HEIGHT - 1)) &&
                    (col_cnt == ($bits(col_cnt))'(IMG_WIDTH  - 1));
/* verilator lint_on WIDTHEXPAND */

/* verilator lint_off WIDTHEXPAND */
assign col_cnt_out = 10'(col_latch);
assign row_cnt_out = 10'(row_latch);
/* verilator lint_on WIDTHEXPAND */

// -------------------------------------------------------------------------
// msg_composer instantiation
// -------------------------------------------------------------------------
msg_composer u_msg_composer (
    .row   (10'(row_latch)),
    .col   (10'(col_latch)),
    .pixel (pixel_latch),
    .msg   (msg)
);

// -------------------------------------------------------------------------
// FSM state register
// -------------------------------------------------------------------------
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        current_state <= IDLE;
    else
        current_state <= next_state;
end

// -------------------------------------------------------------------------
// Next-state logic (combinatorial)
// -------------------------------------------------------------------------
always_comb begin : next_state_logic
    next_state = current_state;

    case (current_state)
        IDLE: begin
            if (!fifo_empty && !cts)
                next_state = POP_FIFO;
            else
                next_state = IDLE;
        end

        POP_FIFO:  next_state = WAIT_DATA;

        WAIT_DATA: next_state = LATCH;
        
        LATCH:     next_state = WAIT_MAC;

        WAIT_MAC: begin
            if (!mac_busy)
                next_state = SEND;
            else
                next_state = WAIT_MAC;
        end

        SEND: next_state = WAIT_BUSY;

        WAIT_BUSY: begin
            if (mac_busy)
                next_state = WAIT_DONE;
            else
                next_state = WAIT_BUSY;
        end

        WAIT_DONE: begin
            if (!mac_busy)
                next_state = NEXT;
            else
                next_state = WAIT_DONE;
        end

        // Three of every four pixels come from the group already popped, so
        // NEXT returns to LATCH rather than to IDLE. Only the fourth pixel
        // ends the group and triggers another pop.
        NEXT: begin
            if (pix_idx != 2'd3)   next_state = LATCH;   // same group
            else if (last_pixel)   next_state = DONE;
            else                   next_state = IDLE;    // pop a new group
        end

        DONE:    next_state = IDLE;
        default: next_state = IDLE;
    endcase
end : next_state_logic

// -------------------------------------------------------------------------
// Output registers (separate blocks per output)
// -------------------------------------------------------------------------
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) fifo_pop <= 1'b0;
    else        fifo_pop <= (current_state == POP_FIFO);
end

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) msg_valid <= 1'b0;
    else        msg_valid <= (current_state == SEND);
end

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) tx_img_done <= 1'b0;
    else        tx_img_done <= (current_state == DONE);
end

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) busy <= 1'b0;
    else        busy <= (current_state != IDLE);
end

// -------------------------------------------------------------------------
// Datapath: latch pixel and row/col in LATCH state
// -------------------------------------------------------------------------
// WAIT_DATA captures the three channel words; LATCH selects one pixel out of
// them. LATCH is re-entered for each of the four pixels, so a single pop
// feeds four messages.
//
// BYTE-LANE ORIENTATION: within a word the MSB lane is the LEFTMOST pixel,
// so lane 0 is bits [31:24]. Same convention as rom_sequencer's read side and
// pixel_word_packer's write side -- reading it the other way round would
// mirror every group of four pixels.
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        word_r <= '0;
        word_g <= '0;
        word_b <= '0;
    end else if (current_state == WAIT_DATA) begin
        word_r <= fifo_rd_data_r;
        word_g <= fifo_rd_data_g;
        word_b <= fifo_rd_data_b;
    end
end

// Which pixel of the current word group is being sent. Cleared when a new
// group is popped, advanced in NEXT.
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)                              pix_idx <= 2'd0;
    else if (current_state == WAIT_DATA)     pix_idx <= 2'd0;
    else if (current_state == NEXT)          pix_idx <= pix_idx + 2'd1;
end

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        pixel_latch <= '0;
        row_latch   <= '0;
        col_latch   <= '0;
    end else if (current_state == LATCH) begin
        unique case (pix_idx)
            2'd0: pixel_latch <= {word_r[31:24], word_g[31:24], word_b[31:24]};
            2'd1: pixel_latch <= {word_r[23:16], word_g[23:16], word_b[23:16]};
            2'd2: pixel_latch <= {word_r[15:8],  word_g[15:8],  word_b[15:8]};
            2'd3: pixel_latch <= {word_r[7:0],   word_g[7:0],   word_b[7:0]};
        endcase
        row_latch   <= row_cnt;
        col_latch   <= col_cnt;
    end
end

// -------------------------------------------------------------------------
// Row/col counters
// -------------------------------------------------------------------------
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        col_cnt <= '0;
        row_cnt <= '0;
    end else if (current_state == NEXT) begin
        /* verilator lint_off WIDTHEXPAND */
        if (col_cnt == ($bits(col_cnt))'(IMG_WIDTH - 1)) begin
        /* verilator lint_on WIDTHEXPAND */
            col_cnt <= '0;
            row_cnt <= row_cnt + 1'b1;
        end else begin
            col_cnt <= col_cnt + 1'b1;
        end
    end
end

endmodule