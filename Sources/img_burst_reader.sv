`timescale 1ns/1ps
// img_burst_reader.sv
// ===================
// Full-image read engine. Drains the three RGB SRAMs into the three channel
// FIFOs using AHB-Lite INCR4 bursts, one channel at a time, round-robin.
//
// REPLACES rom_sequencer, which read all three SRAMs in parallel at one
// address. A single-manager bus cannot do that -- accesses serialise -- so a
// burst covers four words of ONE channel instead:
//
//   burst 1   R SRAM, words N..N+3   ->  4 entries into the R FIFO
//   burst 2   G SRAM, words N..N+3   ->  4 entries into the G FIFO
//   burst 3   B SRAM, words N..N+3   ->  4 entries into the B FIFO
//   then N += 4
//
// That is why there are three FIFOs. Red is up to two bursts ahead of blue,
// so the channels hold different amounts and one pointer cannot describe
// them all. Under the old parallel read they stayed in lockstep and a single
// FIFO was enough.
//
// WHY AN ARBITER AND NOT A COUNTER
//
// R, G, B could be a fixed rotation. It is an arbiter because a channel
// whose FIFO is nearly full must be able to DROP OUT while the others keep
// going -- otherwise one slow channel stalls the whole drain. A channel
// requests only when its FIFO has room for a whole burst, and
// round_robin_arbiter's lock holds the grant for the duration so a burst is
// never cut in half.
//
// The pointer only advances when the privileged channel actually takes its
// turn, so a channel that is backed up keeps its place in the rotation
// rather than losing it.
//
// BACK-PRESSURE IS PER CHANNEL NOW
//
// almost_full used to be one signal for one FIFO. Each channel has its own,
// and it gates that channel's request rather than halting the engine. There
// is no WAIT_DRAIN state any more: a full channel simply stops asking.

module img_burst_reader
    import ahb_pkg::*;
#(
    parameter int IMG_WIDTH       = 256,
    parameter int IMG_HEIGHT      = 256,
    parameter int PIXELS_PER_WORD = 4
)(
    input  logic clk,
    input  logic rst_n,

    input  logic start,          // one-cycle pulse: drain the whole image
    output logic seq_done,       // one cycle, image complete
    output logic busy,

    // ---- per-channel back-pressure ---------------------------------------
    // A channel may be granted a burst only when its FIFO can take four more
    // entries. AF_THRESHOLD leaves exactly that much headroom.
    input  logic [NUM_SLAVES-1:0] fifo_almost_full,

    // ---- FIFO write side, one enable per channel -------------------------
    output logic [NUM_SLAVES-1:0] fifo_wr_en,
    output logic [DATA_W-1:0]     fifo_wr_data,   // shared: one beat lands in
                                                  // exactly one channel

    // ---- AHB master request ----------------------------------------------
    output logic                  req_valid,
    output logic                  req_write,
    output logic                  req_burst,
    output logic [SEL_W-1:0]      req_channel,
    output logic [WORD_W-1:0]     req_word,
    input  logic                  ahb_busy,

    // ---- AHB master read data --------------------------------------------
    input  logic                  rd_valid,
    input  logic [DATA_W-1:0]     rd_data,

    // ---- arbiter ----------------------------------------------------------
    output logic [NUM_SLAVES-1:0] arb_req,
    input  logic [NUM_SLAVES-1:0] arb_gnt
);

    localparam int WORDS_PER_CH = (IMG_WIDTH * IMG_HEIGHT) / PIXELS_PER_WORD;
    localparam int LAST_BASE    = WORDS_PER_CH - BEATS_PER_BURST;

    typedef enum logic [2:0] {
        S_IDLE    = 3'd0,
        S_ARB     = 3'd1,   // ask for a channel
        S_REQ     = 3'd2,   // issue the INCR4
        S_COLLECT = 3'd3,   // take four beats into that channel's FIFO
        S_ADVANCE = 3'd4,   // all three channels done at this base
        S_DONE    = 3'd5
    } state_e;

    state_e state, next_state;

    logic [WORD_W-1:0]     base_word;              // start of the current group of 4
    logic [NUM_SLAVES-1:0] ch_done;                // channels finished at this base
    logic [SEL_W-1:0]      cur_ch;                 // channel of the burst in flight
    logic [2:0]            beats_left;

    // -----------------------------------------------------------------
    // Request vector: a channel asks when it still has work at this base
    // AND its FIFO can take a whole burst.
    // -----------------------------------------------------------------
    for (genvar i = 0; i < NUM_SLAVES; i++) begin : g_req
        assign arb_req[i] = (state == S_ARB) && !ch_done[i] && !fifo_almost_full[i];
    end : g_req

    logic any_grant;
    assign any_grant = |arb_gnt;

    // Granted channel index, from the one-hot grant.
    logic [SEL_W-1:0] gnt_idx;
    always_comb begin : g_idx
        gnt_idx = '0;
        for (int i = 0; i < NUM_SLAVES; i++)
            if (arb_gnt[i]) gnt_idx = SEL_W'(i);
    end : g_idx

    // -----------------------------------------------------------------
    // Next state
    // -----------------------------------------------------------------
    always_comb begin : next_st
        next_state = state;
        unique case (state)
            S_IDLE    : if (start) next_state = S_ARB;

            // Wait here until some channel both wants a burst and has room.
            // If every channel is backed up, arb_req is zero and this simply
            // spins -- the FIFOs are draining, so it always resolves.
            // ahb_busy is tested HERE, not in S_REQ. Gating req_valid on it
            // would be a combinational loop: ahb_master's busy includes
            // req_valid, so req_valid <- !busy <- req_valid. Checking before
            // entry means req_valid depends only on state.
            S_ARB     : if (&ch_done)                     next_state = S_ADVANCE;
                        else if (any_grant && !ahb_busy)  next_state = S_REQ;

            // One cycle. The master was idle on entry and nothing else drives
            // it, so the request is always taken.
            S_REQ     : next_state = S_COLLECT;

            S_COLLECT : if (rd_valid && (beats_left == 3'd1))
                                             next_state = S_ARB;

            S_ADVANCE : next_state = (base_word == WORD_W'(LAST_BASE))
                                     ? S_DONE : S_ARB;

            S_DONE    : next_state = S_IDLE;
            default   : next_state = S_IDLE;
        endcase
    end : next_st

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) state <= S_IDLE;
        else        state <= next_state;
    end

    // -----------------------------------------------------------------
    // Datapath
    // -----------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            base_word  <= '0;
            ch_done    <= '0;
            cur_ch     <= '0;
            beats_left <= 3'd0;
        end
        else begin
            unique case (state)
                S_IDLE: if (start) begin
                    base_word <= '0;
                    ch_done   <= '0;
                end

                S_ARB: if (any_grant && !ahb_busy) begin
                    cur_ch     <= gnt_idx;
                    beats_left <= 3'(BEATS_PER_BURST);
                end

                S_COLLECT: if (rd_valid) begin
                    beats_left <= beats_left - 3'd1;
                    // Mark the channel done as its last beat lands, not when
                    // the request was issued -- a burst that never completed
                    // must not count.
                    if (beats_left == 3'd1) ch_done[cur_ch] <= 1'b1;
                end

                S_ADVANCE: begin
                    base_word <= base_word + WORD_W'(BEATS_PER_BURST);
                    ch_done   <= '0;
                end

                default: ;
            endcase
        end
    end

    // -----------------------------------------------------------------
    // Outputs
    // -----------------------------------------------------------------
    assign req_valid   = (state == S_REQ);
    assign req_write   = 1'b0;
    assign req_burst   = 1'b1;                 // always INCR4
    assign req_channel = cur_ch;
    assign req_word    = base_word;

    // Each returned beat goes to exactly one channel FIFO -- the one whose
    // burst is in flight. One shared data bus, three enables.
    for (genvar i = 0; i < NUM_SLAVES; i++) begin : g_wr
        assign fifo_wr_en[i] = (state == S_COLLECT) && rd_valid &&
                               (cur_ch == SEL_W'(i));
    end : g_wr
    assign fifo_wr_data = rd_data;

    assign seq_done = (state == S_DONE);
    assign busy     = (state != S_IDLE);

`ifndef SYNTHESIS
    initial begin
        if (WORDS_PER_CH % BEATS_PER_BURST != 0)
            $error("%m: %0d words per channel is not a whole number of INCR4 bursts",
                   WORDS_PER_CH);
    end

    // A beat may only ever land in one channel.
    a_wr_onehot: assert property (
        @(posedge clk) disable iff (!rst_n)
        $onehot0(fifo_wr_en)
    ) else $error("%m: more than one channel FIFO written in a cycle");

    // Never write a channel that already finished this base -- that would be
    // a duplicated burst and would shift the image.
    a_no_double_burst: assert property (
        @(posedge clk) disable iff (!rst_n)
        (state == S_COLLECT) |-> !ch_done[cur_ch]
    ) else $error("%m: collecting a burst for a channel already marked done");

    // The arbiter must not grant a channel that is backed up.
    a_gnt_has_room: assert property (
        @(posedge clk) disable iff (!rst_n)
        ((state == S_ARB) && any_grant && !ahb_busy) |-> !fifo_almost_full[gnt_idx]
    ) else $error("%m: granted a channel whose FIFO cannot take a burst");

    // Exactly four beats per burst.
    a_beats_bounded: assert property (
        @(posedge clk) disable iff (!rst_n)
        (state == S_COLLECT) |-> (beats_left != 3'd0)
    ) else $error("%m: data beat arrived after the burst should have ended");
`endif

endmodule : img_burst_reader
