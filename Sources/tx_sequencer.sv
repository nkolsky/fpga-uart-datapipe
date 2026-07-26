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
    input  logic [23:0] fifo_rd_data,   // pixel data from FIFO

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

        NEXT: begin
            if (last_pixel)
                next_state = DONE;
            else
                next_state = IDLE;
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
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        pixel_latch <= '0;
        row_latch   <= '0;
        col_latch   <= '0;
    end else if (current_state == WAIT_DATA) begin
        pixel_latch <= fifo_rd_data;
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