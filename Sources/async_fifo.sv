// async_fifo.sv
// -------------
// Dual-clock image FIFO.
//
// wr_clk = CLK100MHZ, rd_clk = pll_clk_out.
// Gray pointers and 2-FF synchronizers keep the queue coherent across domains.

`timescale 1ns/1ps

import fifo_pkg::*;

// -----------------------------------------------------------------------------
// DW is the only parameter. The image FIFO uses the default width.
//
// Pointer geometry stays fixed in fifo_pkg so the gray/bin helpers stay valid.
// Changing depth per instance would corrupt the CDC logic silently.
// -----------------------------------------------------------------------------

module async_fifo #(
    parameter int DW = fifo_pkg::FIFO_DATA_WIDTH
) (
    // -----------------------------------------------------------------
    // Write domain
    // -----------------------------------------------------------------
    input  logic                  wr_clk,
    input  logic                  wr_rst_n,     // async reset, write domain
                                                 // (synchronized internally,
                                                 //  matches diagram's "srst")

    input  logic                  wr_en,        // write enable from rom_sequencer
    input  logic [DW-1:0]         wr_data,      // payload (image FIFO: R,G,B packed)

    output logic                  full,         // hard full flag - never write when high
    output logic                  almost_full,  // soft flag - rom_sequencer backpressure

    // -----------------------------------------------------------------
    // Read domain
    // -----------------------------------------------------------------
    input  logic                  rd_clk,
    input  logic                  rd_rst_n,     // async reset, read domain
                                                 // (synchronized internally,
                                                 //  matches diagram's "drst")

    input  logic                  rd_en,        // read enable from tx_sequencer (fifo_pop)

    output logic [DW-1:0]         rd_data,      // payload output
    output logic                  empty,        // hard empty flag - never read when high
    output logic                  almost_empty, // soft flag - rom_sequencer resume signal

    // Registered copy of empty, for CROSSING TO ANOTHER CLOCK DOMAIN ONLY.
    //
    // empty above is a combinational gray-pointer comparison. Feeding a
    // comparator straight into a synchroniser is unsafe: the comparator can
    // glitch while its inputs settle, and a two-flop synchroniser resolves
    // metastability but propagates whatever value it captured -- glitch
    // included. Vivado reports that as CDC-10, "combinational logic detected
    // before a synchronizer".
    //
    // This output is one read-clock behind empty. DO NOT use it as the
    // underflow guard; uart_tx_subsystem must keep using empty, which is
    // correct in the read domain on the cycle it matters.
    //
    // almost_empty below was already registered, which is why CDC-10 fired on
    // empty alone.
    output logic                  empty_cdc
);

// Declared ahead of first use -- Vivado Synth 8-6901 otherwise.
logic [PTR_WIDTH-1:0] rd_ptr_bin, rd_ptr_gray;

// ===========================================================
// Shared: dual-port memory array
// (lives in neither domain exclusively - written by wr_clk,
//  read by rd_clk, addressed independently on each port)
// ===========================================================
logic [DW-1:0] fifo_mem [DEPTH-1:0];

// ===========================================================
// WRITE DOMAIN (wr_clk, wr_rst_n)
// ===========================================================

// -----------------------------------------------------------
// Write-domain pointer state and synchronized read pointer.
// -----------------------------------------------------------
logic [PTR_WIDTH-1:0] wr_ptr_bin, wr_ptr_gray;

// ASYNC_REG keeps the two stages in adjacent slices and stops the placer
// separating them. Without it, settling time for a metastable first stage is
// whatever routing happens to give, so MTBF becomes a placement lottery --
// works today, may not after an unrelated change moves things.
(* ASYNC_REG = "TRUE" *) logic [PTR_WIDTH-1:0] rd_ptr_gray_ff1;   // sync stage 1
(* ASYNC_REG = "TRUE" *) logic [PTR_WIDTH-1:0] rd_ptr_gray_sync;  // sync stage 2

// -----------------------------------------------------------
// Write pointer advances with the same pre-increment value used
// to generate the gray-coded pointer.
// -----------------------------------------------------------
always_ff @(posedge wr_clk or negedge wr_rst_n) begin
    if (!wr_rst_n) begin
        wr_ptr_bin  <= '0;
        wr_ptr_gray <= '0;
    end else if (wr_en && !full) begin
        wr_ptr_bin  <= wr_ptr_bin + 1;            // increment binary write pointer
        wr_ptr_gray <= bin2gray(wr_ptr_bin + 1);  // convert to gray code for synchronization
    end
end

// -----------------------------------------------------------
// Memory write - no reset (BRAM-inferable pattern); only the
// pointer/control logic needs to reset, not array contents.
// -----------------------------------------------------------
always_ff @(posedge wr_clk) begin
    if (wr_en && !full) begin
        fifo_mem[wr_ptr_bin[ADDR_WIDTH-1:0]] <= wr_data;  // write data to memory
    end
end

// -----------------------------------------------------------
// Synchronize the read pointer into the write domain.
// -----------------------------------------------------------
always_ff @(posedge wr_clk or negedge wr_rst_n) begin
    if (!wr_rst_n) begin
        rd_ptr_gray_ff1  <= '0;
        rd_ptr_gray_sync <= '0;
    end else begin
        rd_ptr_gray_ff1  <= rd_ptr_gray;       // first stage of synchronizer
        rd_ptr_gray_sync <= rd_ptr_gray_ff1;   // second stage of synchronizer
    end
end

// -----------------------------------------------------------
// Convert the synchronized read pointer back to binary.
// This is registered to keep the flag path shorter and more stable.
// -----------------------------------------------------------
logic [PTR_WIDTH-1:0] rd_bin_sync;

always_ff @(posedge wr_clk or negedge wr_rst_n) begin
    if (!wr_rst_n) rd_bin_sync <= '0;
    else           rd_bin_sync <= PTR_WIDTH'(gray2bin(rd_ptr_gray_sync));
end

// -----------------------------------------------------------
// full: direct gray comparison against the synchronized read pointer.
// -----------------------------------------------------------
assign full = (wr_ptr_gray == {~rd_ptr_gray_sync[PTR_WIDTH-1:PTR_WIDTH-2],
                                 rd_ptr_gray_sync[PTR_WIDTH-3:0]});

// -----------------------------------------------------------
// almost_full is advisory backpressure; full is the real overflow guard.
// -----------------------------------------------------------
always_ff @(posedge wr_clk or negedge wr_rst_n) begin
    if (!wr_rst_n) begin
        almost_full <= 1'b0;
    end else begin
        // Cast to the pointer width before subtracting. Without it the
        // operands widen to 32 bits against a 7-bit function result and the
        // tool warns at every occurrence.
        // wr_ptr_bin, NOT gray2bin(wr_ptr_gray). Both are written in
        // the same always_ff from the same value, so they are equal by
        // construction -- converting to gray and straight back was a
        // same-domain round trip that bought nothing and cost two LUT
        // levels in the flag cone.
        almost_full <= ((wr_ptr_bin - rd_bin_sync) >= PTR_WIDTH'(AF_THRESHOLD));
    end
end

// ===========================================================
// READ DOMAIN (rd_clk, rd_rst_n)
// ===========================================================


(* ASYNC_REG = "TRUE" *) logic [PTR_WIDTH-1:0] wr_ptr_gray_ff1;   // stage 1 (may go metastable)
(* ASYNC_REG = "TRUE" *) logic [PTR_WIDTH-1:0] wr_ptr_gray_sync;  // stage 2 (settled)

// -----------------------------------------------------------
// Read pointer advances with the same pre-increment value used
// to generate the gray-coded pointer.
// -----------------------------------------------------------
always_ff @(posedge rd_clk or negedge rd_rst_n) begin
    if (!rd_rst_n) begin
        rd_ptr_bin  <= '0;
        rd_ptr_gray <= '0;
    end else if (rd_en && !empty) begin
        rd_ptr_bin  <= rd_ptr_bin + 1;            // increment binary read pointer
        rd_ptr_gray <= bin2gray(rd_ptr_bin + 1);  // convert to gray code for synchronization
    end 
end

// -----------------------------------------------------------
// Read data is captured from the current head pointer before the
// read pointer increments. This gives a one-cycle read latency
// with the head value stable at the output.
// -----------------------------------------------------------
always_ff @(posedge rd_clk) begin
    rd_data <= fifo_mem[rd_ptr_bin[ADDR_WIDTH-1:0]];  // pre-increment address - always register head-of-queue
end

// -----------------------------------------------------------
// Synchronize the write pointer into the read domain.
// -----------------------------------------------------------
always_ff @(posedge rd_clk or negedge rd_rst_n) begin
    if (!rd_rst_n) begin
        wr_ptr_gray_ff1  <= '0;
        wr_ptr_gray_sync <= '0;
    end else begin
        wr_ptr_gray_ff1  <= wr_ptr_gray;       // first stage of synchronizer
        wr_ptr_gray_sync <= wr_ptr_gray_ff1;   // second stage of synchronizer
    end
end

// -----------------------------------------------------------
// Convert the synchronized write pointer back to binary.
// This is used for occupancy checks in the read domain.
// -----------------------------------------------------------
logic [PTR_WIDTH-1:0] wr_bin_sync;

always_ff @(posedge rd_clk or negedge rd_rst_n) begin
    if (!rd_rst_n) wr_bin_sync <= '0;
    else           wr_bin_sync <= PTR_WIDTH'(gray2bin(wr_ptr_gray_sync));
end

// -----------------------------------------------------------
// empty is a direct gray-pointer equality check.
// -----------------------------------------------------------
assign empty = (rd_ptr_gray == wr_ptr_gray_sync);

// Registered copy for the clock crossing. One flop in the read domain, so the
// synchroniser downstream sees a flop output rather than combinational logic.
// Functional use of empty is untouched.
always_ff @(posedge rd_clk or negedge rd_rst_n) begin
    if (!rd_rst_n) empty_cdc <= 1'b1;      // empty at reset, same sense as empty
    else           empty_cdc <= empty;
end

// -----------------------------------------------------------
// almost_empty is advisory; empty is the real underflow guard.
// -----------------------------------------------------------
always_ff @(posedge rd_clk or negedge rd_rst_n) begin
    if (!rd_rst_n) begin
        almost_empty <= 1'b0;
    end else begin
        // rd_ptr_bin, NOT gray2bin(rd_ptr_gray) -- same-domain round
        // trip, see almost_full.
        almost_empty <= ((wr_bin_sync - rd_ptr_bin) <= PTR_WIDTH'(AE_THRESHOLD));
    end 
end

endmodule : async_fifo