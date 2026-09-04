`timescale 1ns/1ps
// ahb_master.sv
// =============
// AHB-Lite manager. Issues a SINGLE transfer or an INCR4 burst on request.
//
// THE PIPELINE -- an INCR4 read, zero wait states
//
//   cycle      1        2        3        4        5
//   HTRANS   NONSEQ    SEQ      SEQ      SEQ     IDLE
//   HADDR      A0       A1       A2       A3      --
//   HRDATA     --       D0       D1       D2      D3
//
// Four beats take FIVE cycles, not four: the last beat's data phase runs
// after its address phase has already gone. That trailing cycle is why the
// FSM has a drain state rather than returning straight to idle.
//
// Writes are the mirror image -- HWDATA for beat N is presented in cycle
// N+1, alongside beat N+1's address.
//
// HREADY GATES EVERYTHING
//
// A slave stretches a transfer by holding HREADY low. While it is low the
// address phase must be HELD, not advanced, and the data phase in flight
// must not be counted as complete. Every piece of state here updates only
// when HREADY is high, which is what makes wait states work without any
// special case.
//
// HTRANS SEQUENCING
//
//   NONSEQ  first beat of a burst, or a single transfer
//   SEQ     every later beat -- same burst, address one word on
//   IDLE    no transfer
//
// BUSY is not generated. It exists for a manager that needs to stall inside
// a burst; this one always has its next address ready, so a burst once
// started runs to completion. Worth knowing for the arbiter in step 5: with
// no BUSY beats, a burst occupies the bus for exactly BEATS+1 cycles.

module ahb_master
    import ahb_pkg::*;
(
    input  logic                 hclk,
    input  logic                 hresetn,

    // ---- request ---------------------------------------------------------
    input  logic                 req_valid,     // one-cycle strobe
    input  logic                 req_write,
    input  logic                 req_burst,     // 0 = SINGLE, 1 = INCR4
    input  logic [SEL_W-1:0]     req_channel,
    input  logic [WORD_W-1:0]    req_word,      // base word index
    output logic                 busy,

    // ---- read data out, one pulse per beat --------------------------------
    output logic                 rd_valid,
    output logic [DATA_W-1:0]    rd_data,
    output logic [1:0]           rd_beat,       // which beat this word is

    // ---- write data in ----------------------------------------------------
    // wr_data must hold the word for the beat currently in its DATA phase.
    // wr_ack pulses when that word has been taken; present the next one on
    // the following cycle.
    input  logic [DATA_W-1:0]    wr_data,
    output logic                 wr_ack,
    output logic [1:0]           wr_beat,

    // ---- AHB-Lite manager interface ---------------------------------------
    output logic [ADDR_W-1:0]    haddr,
    output logic                 hwrite,
    output logic [2:0]           hsize,
    output logic [2:0]           hburst,
    output logic [1:0]           htrans,
    output logic [DATA_W-1:0]    hwdata,
    input  logic                 hready,        // aggregated segment ready
    input  logic [DATA_W-1:0]    hrdata,
    input  logic                 hresp,

    // ---- diagnostics -------------------------------------------------------
    output logic                 err_sticky     // any beat returned HRESP=ERROR
);

    typedef enum logic [1:0] {
        M_IDLE  = 2'd0,
        M_ADDR  = 2'd1,   // issuing address beats
        M_DRAIN = 2'd2    // last data phase still outstanding
    } state_e;

    state_e state, next_state;

    logic                write_q;
    logic                burst_q;
    logic [SEL_W-1:0]    chan_q;
    logic [WORD_W-1:0]   base_q;
    logic [1:0]          beat_q;      // address-phase beat index

    logic [1:0] last_beat;
    assign last_beat = burst_q ? 2'd3 : 2'd0;

    // -----------------------------------------------------------------
    // Next state. Every transition is gated on hready -- a stretched
    // transfer holds the FSM where it is.
    // -----------------------------------------------------------------
    always_comb begin : next_st
        next_state = state;
        unique case (state)
            M_IDLE  : if (req_valid) next_state = M_ADDR;
            M_ADDR  : if (hready && (beat_q == last_beat)) next_state = M_DRAIN;
            M_DRAIN : if (hready) next_state = M_IDLE;
            default :                next_state = M_IDLE;
        endcase
    end : next_st

    always_ff @(posedge hclk or negedge hresetn) begin
        if (!hresetn) state <= M_IDLE;
        else          state <= next_state;
    end

    // -----------------------------------------------------------------
    // Request capture and beat counter
    // -----------------------------------------------------------------
    always_ff @(posedge hclk or negedge hresetn) begin
        if (!hresetn) begin
            write_q <= 1'b0;
            burst_q <= 1'b0;
            chan_q  <= '0;
            base_q  <= '0;
            beat_q  <= 2'd0;
        end
        else if (state == M_IDLE && req_valid) begin
            write_q <= req_write;
            burst_q <= req_burst;
            chan_q  <= req_channel;
            base_q  <= req_word;
            beat_q  <= 2'd0;
        end
        else if (state == M_ADDR && hready && (beat_q != last_beat)) begin
            beat_q <= beat_q + 2'd1;
        end
    end

    // -----------------------------------------------------------------
    // Address phase outputs
    //
    // The address steps one WORD per beat. addr_of places the word index at
    // HADDR[15:2], so consecutive words are 4 bytes apart -- which is what
    // HSIZE_WORD requires of an INCR burst.
    // -----------------------------------------------------------------
    always_comb begin : addr_out
        if (state == M_ADDR) begin
            htrans = (beat_q == 2'd0) ? HTRANS_NONSEQ : HTRANS_SEQ;
            haddr  = addr_of(chan_q, WORD_W'(base_q + WORD_W'(beat_q)));
            hburst = burst_q ? HBURST_INCR4 : HBURST_SINGLE;
        end
        else begin
            htrans = HTRANS_IDLE;
            haddr  = '0;
            hburst = HBURST_SINGLE;
        end
    end : addr_out

    assign hwrite = write_q;
    assign hsize  = HSIZE_WORD;      // this bus is word-only, see ahb_slave_sram

    // -----------------------------------------------------------------
    // Data phase tracking
    //
    // One cycle behind the address phase. Updating only on hready is what
    // makes a stretched data phase hold rather than being counted twice.
    // -----------------------------------------------------------------
    logic       dphase_valid;
    logic       dphase_write;
    logic [1:0] dphase_beat;

    always_ff @(posedge hclk or negedge hresetn) begin
        if (!hresetn) begin
            dphase_valid <= 1'b0;
            dphase_write <= 1'b0;
            dphase_beat  <= 2'd0;
        end
        else if (hready) begin
            dphase_valid <= (state == M_ADDR);
            dphase_write <= write_q;
            dphase_beat  <= beat_q;
        end
    end

    logic dphase_done;
    assign dphase_done = dphase_valid && hready;

    assign rd_valid = dphase_done && !dphase_write;
    assign rd_data  = hrdata;
    assign rd_beat  = dphase_beat;

    assign hwdata  = wr_data;
    assign wr_ack  = dphase_done && dphase_write;
    assign wr_beat = dphase_beat;

    assign busy = (state != M_IDLE) || req_valid;

    // -----------------------------------------------------------------
    // Error capture. AHB-Lite signals ERROR over two cycles; this samples it
    // where the data phase completes, which is sufficient to flag that a beat
    // failed. Sticky, so a single bad beat in a burst is still visible after.
    // -----------------------------------------------------------------
    always_ff @(posedge hclk or negedge hresetn) begin
        if (!hresetn)                                err_sticky <= 1'b0;
        else if (dphase_done && (hresp == HRESP_ERROR)) err_sticky <= 1'b1;
    end

`ifndef SYNTHESIS
    // A request while busy would be dropped -- the same silent-loss shape the
    // APB master guards against.
    a_req_when_idle: assert property (
        @(posedge hclk) disable iff (!hresetn)
        req_valid |-> (state == M_IDLE)
    ) else $error("%m: request issued while the manager was busy");

    // NONSEQ only ever opens a burst.
    a_nonseq_first: assert property (
        @(posedge hclk) disable iff (!hresetn)
        (htrans == HTRANS_NONSEQ) |-> (beat_q == 2'd0)
    ) else $error("%m: NONSEQ on a beat other than the first");

    // SEQ only ever continues one.
    a_seq_not_first: assert property (
        @(posedge hclk) disable iff (!hresetn)
        (htrans == HTRANS_SEQ) |-> (beat_q != 2'd0)
    ) else $error("%m: SEQ on the first beat");

    // A burst, once started, runs to completion: SEQ must follow NONSEQ or
    // SEQ, never IDLE.
    a_burst_unbroken: assert property (
        @(posedge hclk) disable iff (!hresetn)
        (htrans == HTRANS_SEQ) |-> $past(htrans inside {HTRANS_NONSEQ, HTRANS_SEQ})
    ) else $error("%m: burst broken -- SEQ did not follow NONSEQ/SEQ");

    // While a slave stretches, the address must be HELD.
    a_addr_held: assert property (
        @(posedge hclk) disable iff (!hresetn)
        ((state == M_ADDR) && !hready) |=> $stable(haddr)
    ) else $error("%m: address advanced while HREADY was low");

    // Consecutive beats step exactly one word.
    //
    // Qualified on $past(hready): while a slave stretches, the address is
    // HELD, so haddr equals its previous value rather than advancing. Without
    // the qualifier this fires on the second cycle of every stretched beat --
    // which is exactly how it was found.
    a_incr_by_word: assert property (
        @(posedge hclk) disable iff (!hresetn)
        ((htrans == HTRANS_SEQ) && $past(hready)) |-> (haddr == $past(haddr) + ADDR_W'(4))
    ) else $error("%m: INCR address did not step by one word");
`endif

endmodule : ahb_master
