// sram_rmw.sv
// ===========
// Performs masked word updates against an SRAM that has NO per-byte write
// enable. Sits between pixel_word_packer and the three channel SRAMs.
//
//   pixel_word_packer -> sram_rmw -> rgb_sram x3
//
// -----------------------------------------------------------------------
// WHY THIS EXISTS
// -----------------------------------------------------------------------
// A write covers the whole 32-bit word. Each word packs FOUR pixels of one
// colour channel, so changing a single pixel means reading the word,
// replacing one byte, and writing it back.
//
// The packer still produces a byte enable. It is no longer a hardware mask --
// it is a MERGE mask, and it also says whether the read is needed at all:
//
//     req_be == 1111   the incoming data covers every lane, so nothing of
//                      the old word survives.  WRITE DIRECTLY, NO READ.
//     anything else    some lanes must be preserved.  READ, MERGE, WRITE.
//
// -----------------------------------------------------------------------
// THIS IS WHY GROUPING BURST PIXELS MATTERS SO MUCH MORE NOW
// -----------------------------------------------------------------------
// A burst data message carries exactly four pixels, and four pixels are
// exactly one word per channel. Grouped, that is ONE plain write per channel
// with no read.
//
// The old design split each message into four single-pixel commands. Without
// byte enables every one of those is a read-modify-write, so a single burst
// data message would cost FOUR READS AND FOUR WRITES per channel -- twenty
// four SRAM accesses -- to deliver data that arrived already assembled and
// needs three.
//
//     grouped     3 accesses per message   (1 write x 3 channels)
//     ungrouped  24 accesses per message   (4 read + 4 write x 3 channels)
//
// -----------------------------------------------------------------------
// NO SAME-WORD HAZARD, BY CONSTRUCTION
// -----------------------------------------------------------------------
// Read-modify-write normally carries a hazard: two updates to the same word
// in flight both read the same stale value and the second discards the
// first. That cannot happen here, because pixel_word_packer ACCUMULATES and
// issues at most one request per word. Two consecutive requests are always
// different words, and this module handles one request at a time anyway.
//
// -----------------------------------------------------------------------
// THE READ PORT IS SHARED
// -----------------------------------------------------------------------
// The SRAMs have one address port. Writes now need the read side too, so
// this module must arbitrate with the other readers rather than assuming
// the write path is independent. port_req is raised for the whole duration
// of a request and released when it completes; mem_interlock grants it.
//
// A full-word write still needs the port, but only for the write cycle.

`timescale 1ns/1ps

module sram_rmw
    import memory_pkg::*;
#(
    parameter int ADDR_W = SRAM_ADDR_WIDTH,
    parameter int DATA_W = SRAM_DATA_WIDTH,
    parameter int NLANE  = PIXELS_PER_WORD
)(
    input  logic clk,
    input  logic rst_n,

    // ---- request in, from pixel_word_packer -----------------------------
    input  logic              req_valid,
    output logic              req_ready,
    input  logic [ADDR_W-1:0] req_addr,
    input  logic [NLANE-1:0]  req_be,      // merge mask; 1111 = no read
    input  logic [DATA_W-1:0] req_data_r,
    input  logic [DATA_W-1:0] req_data_g,
    input  logic [DATA_W-1:0] req_data_b,

    // ---- SRAM port -------------------------------------------------------
    output logic              rd_en,
    output logic [ADDR_W-1:0] rd_addr,
    input  logic [DATA_W-1:0] rd_data_r,
    input  logic [DATA_W-1:0] rd_data_g,
    input  logic [DATA_W-1:0] rd_data_b,

    output logic              wr_en,
    output logic [ADDR_W-1:0] wr_addr,
    output logic [DATA_W-1:0] wr_data_r,
    output logic [DATA_W-1:0] wr_data_g,
    output logic [DATA_W-1:0] wr_data_b,

    // ---- arbitration with the other readers ------------------------------
    output logic              port_req,
    input  logic              port_grant,

    // ---- status -----------------------------------------------------------
    output logic              busy,
    output logic              rmw_count_pulse   // one per read-modify-write
);

    typedef enum logic [2:0] {
        S_IDLE  = 3'd0,
        S_READ  = 3'd1,   // read issued
        S_WAIT  = 3'd2,   // rd_data valid at the end of this cycle
        S_MERGE = 3'd3,   // merge and write
        S_WRITE = 3'd4    // full-word write, no read needed
    } state_e;

    state_e state;

    logic [ADDR_W-1:0] addr_q;
    logic [NLANE-1:0]  be_q;
    logic [DATA_W-1:0] dr_q, dg_q, db_q;

    logic full_word;
    assign full_word = (req_be == {NLANE{1'b1}});

    assign req_ready = (state == S_IDLE) && port_grant;
    assign busy      = (state != S_IDLE);

    // Held for the whole request so the interlock does not hand the port to
    // a reader between the read and the write of a single update.
    assign port_req  = req_valid || busy;

    // Merge: a lane whose mask bit is set takes the new byte, otherwise the
    // byte just read is written back unchanged.
    function automatic logic [DATA_W-1:0] merge(input logic [DATA_W-1:0] old_w,
                                                input logic [DATA_W-1:0] new_w,
                                                input logic [NLANE-1:0]  be);
        logic [DATA_W-1:0] r;
        r = old_w;
        for (int i = 0; i < NLANE; i++)
            if (be[i]) r[i*8 +: 8] = new_w[i*8 +: 8];
        return r;
    endfunction

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state           <= S_IDLE;
            rd_en           <= 1'b0;
            rd_addr         <= '0;
            wr_en           <= 1'b0;
            wr_addr         <= '0;
            wr_data_r       <= '0;
            wr_data_g       <= '0;
            wr_data_b       <= '0;
            addr_q          <= '0;
            be_q            <= '0;
            dr_q            <= '0;
            dg_q            <= '0;
            db_q            <= '0;
            rmw_count_pulse <= 1'b0;
        end else begin
            rd_en           <= 1'b0;
            wr_en           <= 1'b0;
            rmw_count_pulse <= 1'b0;

            unique case (state)

                S_IDLE: begin
                    if (req_valid && req_ready) begin
                        addr_q <= req_addr;
                        be_q   <= req_be;
                        dr_q   <= req_data_r;
                        dg_q   <= req_data_g;
                        db_q   <= req_data_b;

                        if (full_word) begin
                            // Nothing of the old word survives.
                            wr_en     <= 1'b1;
                            wr_addr   <= req_addr;
                            wr_data_r <= req_data_r;
                            wr_data_g <= req_data_g;
                            wr_data_b <= req_data_b;
                            state     <= S_WRITE;
                        end else begin
                            rd_en   <= 1'b1;
                            rd_addr <= req_addr;
                            state   <= S_READ;
                        end
                    end
                end

                // rgb_sram registers rd_data, so it is valid one cycle after
                // rd_en was asserted.
                S_READ:  state <= S_WAIT;

                S_WAIT: begin
                    wr_en           <= 1'b1;
                    wr_addr         <= addr_q;
                    wr_data_r       <= merge(rd_data_r, dr_q, be_q);
                    wr_data_g       <= merge(rd_data_g, dg_q, be_q);
                    wr_data_b       <= merge(rd_data_b, db_q, be_q);
                    rmw_count_pulse <= 1'b1;
                    state           <= S_MERGE;
                end

                // The write is presented during this cycle.
                S_MERGE: state <= S_IDLE;
                S_WRITE: state <= S_IDLE;

                default: state <= S_IDLE;

            endcase
        end
    end

`ifndef SYNTHESIS
    // Never both at once: the SRAM has one address port.
    a_no_rd_wr_overlap: assert property (
        @(posedge clk) disable iff (!rst_n)
        !(rd_en && wr_en)
    ) else $error("%m: read and write asserted in the same cycle");

    // A full-word request must never issue a read.
    a_full_word_no_read: assert property (
        @(posedge clk) disable iff (!rst_n)
        (state == S_WRITE) |-> !rd_en
    ) else $error("%m: read issued for a full-word write");

    // Requests are only taken when the port has been granted.
    a_granted: assert property (
        @(posedge clk) disable iff (!rst_n)
        (req_valid && req_ready) |-> port_grant
    ) else $error("%m: request accepted without a port grant");
`endif

endmodule : sram_rmw
