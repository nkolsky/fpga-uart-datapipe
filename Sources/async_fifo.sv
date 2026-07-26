// async_fifo.sv
// -------------
// Async FIFO - Cummings-style dual-clock gray-code pointer FIFO.
// Same clock is tied to both wr_clk and rd_clk at the chip_top level,
// but the internal structure is fully async (gray pointers + 2-FF
// synchronizers crossing each domain), per Lab 9 spec.

`timescale 1ns/1ps

import fifo_pkg::*;

module async_fifo (
    // -----------------------------------------------------------------
    // Write domain
    // -----------------------------------------------------------------
    input  logic                  wr_clk,
    input  logic                  wr_rst_n,     // async reset, write domain
                                                 // (synchronized internally,
                                                 //  matches diagram's "srst")

    input  logic                  wr_en,        // write enable from rom_sequencer
    input  logic [DATA_WIDTH-1:0] wr_data,      // pixel data (R,G,B packed)

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

    output logic [DATA_WIDTH-1:0] rd_data,      // pixel data output
    output logic                  empty,        // hard empty flag - never read when high
    output logic                  almost_empty  // soft flag - rom_sequencer resume signal
);

// ===========================================================
// Shared: dual-port memory array
// (lives in neither domain exclusively - written by wr_clk,
//  read by rd_clk, addressed independently on each port)
// ===========================================================
logic [DATA_WIDTH-1:0] fifo_mem [DEPTH-1:0];

// ===========================================================
// WRITE DOMAIN (wr_clk, wr_rst_n)
// ===========================================================

// -----------------------------------------------------------
// Signal declarations (write-domain native signals first,
// then the incoming synchronized signal from the read domain)
// -----------------------------------------------------------
logic [PTR_WIDTH-1:0] wr_ptr_bin, wr_ptr_gray;

logic [PTR_WIDTH-1:0] rd_ptr_gray_ff1;   // synchronizer stage 1 (at risk of metastability)
logic [PTR_WIDTH-1:0] rd_ptr_gray_sync;  // synchronizer stage 2 (settled, safe to use)

// -----------------------------------------------------------
// Write pointer: binary register + registered gray conversion.
// Both update together from the same pre-increment value so they
// never skew relative to each other.
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
// 2-FF synchronizer: rd_ptr_gray (read domain) -> wr_clk domain.
// Must be declared/driven before full/almost_full reference
// rd_ptr_gray_sync below.
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
// full: combinational, gray pointer comparison (top two MSBs
// inverted relative to the synchronized read pointer).
// -----------------------------------------------------------
assign full = (wr_ptr_gray == {~rd_ptr_gray_sync[PTR_WIDTH-1:PTR_WIDTH-2],
                                 rd_ptr_gray_sync[PTR_WIDTH-3:0]});

// -----------------------------------------------------------
// almost_full: registered, to shorten the combinational path.
// Safe to register because full (above) is combinational and
// unconditionally prevents overflow on its own - almost_full
// is advisory backpressure only.
// -----------------------------------------------------------
always_ff @(posedge wr_clk or negedge wr_rst_n) begin
    if (!wr_rst_n) begin
        almost_full <= 1'b0;
    end else begin
        almost_full <= (gray2bin(wr_ptr_gray) - gray2bin(rd_ptr_gray_sync)) >= AF_THRESHOLD;
    end
end

// ===========================================================
// READ DOMAIN (rd_clk, rd_rst_n)
// ===========================================================
// - rd_ptr_bin, rd_ptr_gray declared and driven here
// - synchronized wr_ptr_gray (the *incoming* synchronizer) lives here
// - empty, almost_empty computed here
// - memory read happens here

logic [PTR_WIDTH-1:0] rd_ptr_bin, rd_ptr_gray;

logic [PTR_WIDTH-1:0] wr_ptr_gray_ff1;   // synchronizer stage 1 (at risk of metastability)
logic [PTR_WIDTH-1:0] wr_ptr_gray_sync;  // synchronizer stage 2 (metastability-safe)

// -----------------------------------------------------------
// Read pointer: binary register + registered gray conversion.
// Both update together from the same pre-increment value so they
// never skew relative to each other.
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
// Memory read - no reset (BRAM-inferable pattern); only the
// pointer/control logic needs to reset, not array contents.
//
// IMPORTANT: rd_ptr_bin is incremented in the same cycle as rd_en,
// so we must capture rd_data using the CURRENT (pre-increment) address.
// We register unconditionally (no rd_en gate) so that rd_data always
// holds the stable head-of-queue entry whenever the FIFO is not empty -
// matching the one-cycle read latency tx_sequencer expects:
//   cycle 0 (POP_FIFO):  rd_en asserted, rd_ptr_bin still at current entry
//   cycle 1 (WAIT_DATA): rd_data settled with fifo_mem[old_ptr]; tx_sequencer latches it
// -----------------------------------------------------------
always_ff @(posedge rd_clk) begin
    rd_data <= fifo_mem[rd_ptr_bin[ADDR_WIDTH-1:0]];  // pre-increment address - always register head-of-queue
end

// -----------------------------------------------------------
// 2-FF synchronizer: wr_ptr_gray (write domain) -> rd_clk domain
// Must be declared/driven before empty/almost_empty reference
// wr_ptr_gray_sync below.
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
// empty: combinational, gray pointer comparison - direct
// equality (no bit inversion, unlike full). Pointers fully
// equal means no wrap-lap difference exists, i.e. genuinely empty.
// -----------------------------------------------------------
assign empty = (rd_ptr_gray == wr_ptr_gray_sync);

// -----------------------------------------------------------
// almost_empty: registered, to shorten the combinational path.
// Safe to register because empty (above) is combinational and
// unconditionally prevents underflow on its own - almost_empty
// is advisory resume signal only.
// -----------------------------------------------------------
always_ff @(posedge rd_clk or negedge rd_rst_n) begin
    if (!rd_rst_n) begin
        almost_empty <= 1'b0;
    end else begin
        almost_empty <= (gray2bin(wr_ptr_gray_sync) - gray2bin(rd_ptr_gray)) <= AE_THRESHOLD;
    end 
end

endmodule : async_fifo