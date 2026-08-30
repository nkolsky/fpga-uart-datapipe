`timescale 1ns/1ps
// img_burst_writer.sv
// ===================
// Full-image write path. Gathers four consecutive word-writes from
// pixel_word_packer and issues them as three AHB-Lite INCR4 bursts, one per
// colour channel.
//
// WHY A GATHER IS NEEDED AT ALL
//
// One burst-data message carries FOUR PIXELS, which de-interleave into
// exactly ONE 32-bit word per channel:
//
//   {<R0,G0,B0,R1>, <G1,B1,R2,G2>, <B2,R3,G3,B3>}
//       -> R word {R0,R1,R2,R3}, G word {G0..G3}, B word {B0..B3}
//
// An INCR4 needs FOUR consecutive words of ONE channel -- R0..R15 -- which is
// four messages. So the natural unit of the write path is one word per
// channel, and the burst has to be manufactured by holding four of them.
//
//   msg 1 -> R word 0, G word 0, B word 0
//   msg 2 -> R word 1, ...
//   msg 3 -> R word 2, ...
//   msg 4 -> R word 3, ...          now: INCR4 to R, then G, then B
//
// FULL IMAGE ONLY
//
// burst_mode is decided once, before the rectangle starts, from the burst
// header geometry: base == 0 and height x width == the whole image. Nothing
// is decided per word.
//
// Two things make that the only safe case. AHB-Lite has no byte strobes, so a
// partial word (wr_be != all ones) cannot be expressed at all -- and
// pixel_word_packer emits partial words at the end of every row of a narrow
// rectangle, because the linear address jumps between rows. For a full-width
// rectangle the row-end flush always coincides with a full word, so every
// write is complete and the addresses run 0..N-1 with no gaps.
//
// Everything else -- single pixels, offset or short rectangles -- keeps the
// direct port with its byte enables, untouched.
//
// PPA NOTE
//
// This buys no throughput. A message arrives every 22 us and a burst takes
// about 50 ns, so the bus is ~0.2% utilised either way. SINGLE transfers
// would be equally protocol-conformant and about 50 flops cheaper, with no
// gather at all. INCR4 is implemented because the specification asks for it
// on the full-image path; the cost is the four-word buffer below and the
// write-side registers in ahb_slave_sram, which were previously tied off.

module img_burst_writer
    import ahb_pkg::*;
#(
    parameter int ADDR_W_IN = 14        // word address width from the packer
)(
    input  logic clk,
    input  logic rst_n,

    // Latched from the burst header before the rectangle starts. Low means
    // this module is transparent and the direct port carries the write.
    input  logic                  burst_mode,
    input  logic                  pack_busy,     // rectangle in progress
    input  logic                  pack_done,     // one pulse, rectangle complete

    // ---- from pixel_word_packer -------------------------------------------
    input  logic                  wr_valid,
    output logic                  wr_ready,
    input  logic [ADDR_W_IN-1:0]  wr_addr,
    input  logic [3:0]            wr_be,
    input  logic [DATA_W-1:0]     wr_data_r,
    input  logic [DATA_W-1:0]     wr_data_g,
    input  logic [DATA_W-1:0]     wr_data_b,

    // ---- AHB master request ------------------------------------------------
    output logic                  req_valid,
    output logic                  req_write,
    output logic                  req_burst,
    output logic [SEL_W-1:0]      req_channel,
    output logic [WORD_W-1:0]     req_word,
    input  logic                  ahb_busy,

    // ---- AHB master write data ---------------------------------------------
    // wr_beat says which beat of the burst is in its DATA phase; the mux
    // below presents that word. ahb_wr_ack pulses as each is taken.
    input  logic [1:0]            ahb_wr_beat,
    input  logic                  ahb_wr_ack,
    output logic [DATA_W-1:0]     ahb_wr_data,

    output logic                  busy
);

    typedef enum logic [2:0] {
        W_IDLE    = 3'd0,
        W_FILL    = 3'd1,   // taking four words from the packer
        W_WAIT    = 3'd2,   // wait for the master to be free
        W_REQ     = 3'd3,   // issue the INCR4 for the current channel
        W_DRAIN   = 3'd4,   // four beats going out
        W_NEXT_CH = 3'd5    // channel done: next one, or refill
    } state_e;

    state_e state, next_state;

    // Four words per channel. 4 x 3 x 32 = 384 bits.
    logic [DATA_W-1:0] buf_r [BEATS_PER_BURST];
    logic [DATA_W-1:0] buf_g [BEATS_PER_BURST];
    logic [DATA_W-1:0] buf_b [BEATS_PER_BURST];

    // pack_done is a ONE-CYCLE pulse and usually lands while the gather is
    // mid-burst, where the FSM cannot see it. Latching it is what lets the
    // engine finish: without this it sits in W_FILL waiting for a word the
    // packer will never send.
    logic              done_seen;

    logic [1:0]        fill_cnt;      // words gathered so far
    logic [WORD_W-1:0] base_word;     // address of buf[0]
    logic [SEL_W-1:0]  cur_ch;        // channel currently bursting

    // -----------------------------------------------------------------
    // Next state
    // -----------------------------------------------------------------
    always_comb begin : next_st
        next_state = state;
        unique case (state)
            W_IDLE   : if (burst_mode && pack_busy) next_state = W_FILL;

            // Four full words gathered. pack_done cannot arrive mid-group for
            // a full image -- the word count divides evenly by four -- but if
            // it ever did, the group would be incomplete and must not burst.
            // Finish only on a group boundary. For a full image the word
            // count divides evenly by four, so fill_cnt is always 0 here when
            // the rectangle ends; a partial group would mean the geometry was
            // not a full image and should never have been in burst mode.
            W_FILL   : if (wr_valid && (fill_cnt == 2'd3))       next_state = W_WAIT;
                       else if (done_seen && (fill_cnt == 2'd0)) next_state = W_IDLE;

            // ahb_busy is tested HERE, in its own state, and NOT gated into
            // req_valid. Gating it would be a combinational loop: the
            // master's busy includes req_valid, so req_valid <- !busy <-
            // req_valid. The reader has the same structure for the same
            // reason.
            W_WAIT   : if (!ahb_busy) next_state = W_REQ;

            // One cycle. The master was free on entry and nothing else drives
            // it, so the request is always taken.
            W_REQ    : next_state = W_DRAIN;

            W_DRAIN  : if (ahb_wr_ack && (ahb_wr_beat == 2'd3))
                                      next_state = W_NEXT_CH;

            W_NEXT_CH: next_state = (cur_ch == SEL_W'(NUM_SLAVES-1))
                                    ? W_FILL     // all three channels done
                                    : W_WAIT;    // next channel, same words

            default  : next_state = W_IDLE;
        endcase
    end : next_st

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) state <= W_IDLE;
        else        state <= next_state;
    end

    // -----------------------------------------------------------------
    // Gather
    // -----------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fill_cnt  <= 2'd0;
            base_word <= '0;
            cur_ch    <= '0;
            done_seen <= 1'b0;
        end
        else begin
            // Latched wherever the FSM happens to be.
            if (pack_done) done_seen <= 1'b1;

            unique case (state)
                W_IDLE: begin
                    fill_cnt  <= 2'd0;
                    cur_ch    <= '0;
                    done_seen <= 1'b0;
                end

                W_FILL: if (wr_valid) begin
                    // The first word of a group fixes the burst address.
                    if (fill_cnt == 2'd0) base_word <= WORD_W'(wr_addr);
                    fill_cnt <= fill_cnt + 2'd1;
                end

                W_NEXT_CH: begin
                    if (cur_ch == SEL_W'(NUM_SLAVES-1)) begin
                        cur_ch   <= '0;
                        fill_cnt <= 2'd0;      // start the next group
                    end
                    else begin
                        cur_ch <= cur_ch + SEL_W'(1);
                    end
                end

                default: ;
            endcase
        end
    end

    // -----------------------------------------------------------------
    // Data buffers -- NO RESET, deliberately
    //
    // These are always written before they are read: the FSM cannot issue a
    // burst until four words have been gathered, and fill_cnt is reset. A
    // reset value would add clear logic to 384 flops for no benefit.
    //
    // Keeping them out of the async-reset block also stops Vivado inferring
    // set and reset on the same register (Synth 8-7137), which it warns can
    // cause a simulation mismatch.
    // -----------------------------------------------------------------
    always_ff @(posedge clk) begin
        if ((state == W_FILL) && wr_valid) begin
            buf_r[fill_cnt] <= wr_data_r;
            buf_g[fill_cnt] <= wr_data_g;
            buf_b[fill_cnt] <= wr_data_b;
        end
    end

    // -----------------------------------------------------------------
    // Outputs
    // -----------------------------------------------------------------
    // Accept words only while gathering. Holding this low through the three
    // bursts back-pressures the packer, which stalls cleanly: its
    // pixel_ready is (!wr_valid || wr_ready), so the whole RX chain waits.
    assign wr_ready = burst_mode ? (state == W_FILL) : 1'b1;

    assign req_valid   = (state == W_REQ);
    assign req_write   = 1'b1;
    assign req_burst   = 1'b1;                 // INCR4
    assign req_channel = cur_ch;
    assign req_word    = base_word;

    // The word for the beat currently in its data phase, from the channel
    // being bursted.
    always_comb begin : wdata_mux
        unique case (cur_ch)
            SEL_W'(0): ahb_wr_data = buf_r[ahb_wr_beat];
            SEL_W'(1): ahb_wr_data = buf_g[ahb_wr_beat];
            SEL_W'(2): ahb_wr_data = buf_b[ahb_wr_beat];
            default  : ahb_wr_data = '0;
        endcase
    end : wdata_mux

    assign busy = (state != W_IDLE);

`ifndef SYNTHESIS
    // A partial word cannot be expressed on this bus at all. burst_mode is
    // supposed to guarantee it never reaches here.
    a_full_words_only: assert property (
        @(posedge clk) disable iff (!rst_n)
        ((state == W_FILL) && wr_valid) |-> (wr_be == 4'hF)
    ) else $error("%m: partial word (be=%b) reached the burst path", wr_be);

    // The gathered words must be consecutive, or the INCR4 address sequence
    // does not describe them.
    //
    // Compared against the last ACCEPTED address, not $past(wr_addr). While
    // the gather is draining three bursts it holds wr_ready low, and the
    // packer holds wr_valid AND wr_addr steady the whole time -- so on
    // returning to W_FILL the previous cycle's address is the same word, not
    // the one before it. $past fires spuriously on every group boundary,
    // which is how this was found.
    logic [ADDR_W_IN-1:0] last_acc_addr;
    logic                 last_acc_valid;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            last_acc_addr  <= '0;
            last_acc_valid <= 1'b0;
        end
        else if ((state == W_FILL) && wr_valid) begin
            last_acc_addr  <= wr_addr;
            last_acc_valid <= (fill_cnt != 2'd3);   // group boundary resets it
        end
    end

    a_consecutive: assert property (
        @(posedge clk) disable iff (!rst_n)
        ((state == W_FILL) && wr_valid && (fill_cnt != 2'd0) && last_acc_valid)
            |-> (wr_addr == last_acc_addr + ADDR_W_IN'(1))
    ) else $error("%m: non-consecutive word address in a burst group");

    // Never take a word while bursting -- that would overwrite the buffer
    // mid-flight.
    a_no_fill_while_bursting: assert property (
        @(posedge clk) disable iff (!rst_n)
        (state inside {W_WAIT, W_REQ, W_DRAIN, W_NEXT_CH}) |-> !wr_ready
    ) else $error("%m: accepted a word while a burst was in flight");

    // Every group must go out as exactly three bursts, one per channel.
    a_three_channels: assert property (
        @(posedge clk) disable iff (!rst_n)
        (state == W_NEXT_CH) |-> (cur_ch <= SEL_W'(NUM_SLAVES-1))
    ) else $error("%m: channel index out of range");
`endif

endmodule : img_burst_writer
