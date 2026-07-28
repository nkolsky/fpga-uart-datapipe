// tx_reply_ctrl.sv
// ----------------
// Builds and schedules the 6-byte Register Read reply.
//
//   byte 0  0x7B  '{'
//   byte 1  pc_rdata[31:24]   V3      big-endian, MSB first
//   byte 2  pc_rdata[23:16]   V2
//   byte 3  pc_rdata[15: 8]   V1
//   byte 4  pc_rdata[ 7: 0]   V0
//   byte 5  0x7D  '}'
//
// tx_mac packs byte 0 in msg_data[7:0], so the frame is assembled LSB-first
// below -- the same convention msg_composer already uses.
//
// -----------------------------------------------------------------------
// ARBITRATION -- why the reply waits
// -----------------------------------------------------------------------
// A reply is held until BOTH tx_seq_busy and mac_busy are low.
//
// An image transfer is 65536 fixed-size packets and the host reads it as one
// uninterrupted stream, counting bytes. A 6-byte reply inserted between two
// pixel messages would desynchronise every packet after it -- the capture
// would fail with a framing error thousands of packets later, far from the
// cause. Deferring costs nothing: at UART rates a full image takes ~1.5 s
// and the host is not waiting on a register value during a capture.
//
// -----------------------------------------------------------------------
// NO PULSE IS LOST
// -----------------------------------------------------------------------
// reply_pending is a level, set by the incoming reply and cleared only when
// the MAC has actually taken the message (mac_busy observed high while this
// module is driving it). Backpressure of any duration simply extends the
// wait; nothing is dropped and nothing is re-sent.
//
// -----------------------------------------------------------------------
// A SECOND REQUEST WHILE ONE IS PENDING
// -----------------------------------------------------------------------
// The second reply is REJECTED and the first is preserved, with
// reply_overrun raised as a sticky diagnostic. Overwriting would answer the
// newer request with a value the host never asked for at that point in the
// exchange, and silently discard an answer it is still waiting for. The
// protocol is strictly request/response, so a well-behaved host cannot
// create this condition -- it exists to make a misbehaving one visible.

`timescale 1ns/1ps

module tx_reply_ctrl
    import msg_pkg::*;
(
    input  logic         clk,            // pll_clk_out, 130 MHz
    input  logic         rst_n,          // sync_pll_rst_n

    // ---- captured register value, already in this clock domain --------
    input  logic         rd_valid,       // one-cycle strobe from cdc_cmd_sync
    input  logic [31:0]  rd_data,

    // ---- transmit path status -----------------------------------------
    input  logic         tx_seq_busy,    // image transfer in progress
    input  logic         mac_busy,       // tx_mac occupied

    // ---- request to drive tx_mac (muxed in chip_top) ------------------
    output logic         reply_req,      // hold high until accepted
    output logic [127:0] reply_msg,
    output logic [4:0]   reply_len,

    // ---- status --------------------------------------------------------
    output logic         reply_pending,  // a reply is queued or in flight
    output logic         reply_sent,     // one cycle, reply handed to the MAC
    output logic         reply_overrun   // sticky: a reply arrived while busy
);

    localparam logic [4:0] REPLY_BYTES = 5'd6;

    logic [31:0] value_q;
    logic        pending;
    logic        driving;      // reply_req asserted, waiting for the MAC

    assign reply_pending = pending;
    assign reply_len     = REPLY_BYTES;

    // Frame assembled LSB-first: byte 0 occupies bits [7:0].
    assign reply_msg = {80'd0,
                        CHAR_CLOSE_BRACE,     // byte 5  [47:40]
                        value_q[ 7: 0],       // byte 4  [39:32]  V0
                        value_q[15: 8],       // byte 3  [31:24]  V1
                        value_q[23:16],       // byte 2  [23:16]  V2
                        value_q[31:24],       // byte 1  [15: 8]  V3
                        CHAR_OPEN_BRACE};     // byte 0  [ 7: 0]

    // Drive the MAC only when the transmit path is completely idle.
    assign reply_req = pending && !tx_seq_busy && !mac_busy;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            value_q       <= '0;
            pending       <= 1'b0;
            driving       <= 1'b0;
            reply_sent    <= 1'b0;
            reply_overrun <= 1'b0;
        end
        else begin
            reply_sent <= 1'b0;

            // ---- accept a new value ----------------------------------
            if (rd_valid) begin
                if (pending) begin
                    // Keep the first answer; flag the collision.
                    reply_overrun <= 1'b1;
                end
                else begin
                    value_q <= rd_data;
                    pending <= 1'b1;
                end
            end

            // ---- hand the frame to the MAC ---------------------------
            // reply_req is combinational, so it deasserts the moment
            // mac_busy rises. Latching `driving` on the request cycle and
            // clearing `pending` when the MAC actually goes busy is what
            // makes the handover survive any amount of backpressure.
            if (reply_req)
                driving <= 1'b1;

            if (driving && mac_busy) begin
                driving    <= 1'b0;
                pending    <= 1'b0;
                reply_sent <= 1'b1;
            end
        end
    end

`ifndef SYNTHESIS
    // A reply must never be offered while the image path is active.
    a_no_overlap: assert property (
        @(posedge clk) disable iff (!rst_n)
        reply_req |-> (!tx_seq_busy && !mac_busy)
    ) else $error("%m: reply offered during image transmission");

    // Once queued, a reply is never silently dropped. The condition is the
    // ACTUAL handoff -- (driving && mac_busy) -- not reply_sent, which is a
    // nonblocking output updated in the very cycle pending is legitimately
    // cleared and so cannot be used to qualify its own clearing.
    a_no_loss: assert property (
        @(posedge clk) disable iff (!rst_n)
        (pending && !(driving && mac_busy)) |=> pending
    ) else $error("%m: pending reply disappeared without being sent");

    // ...and pending may fall ONLY on a genuine handoff. Expressed against
    // reply_sent, which is set by the SAME nonblocking assignment that
    // clears pending, so the two are sampled consistently. $past(driving &&
    // mac_busy) does not work here: `driving` is cleared in that same NBA,
    // so the sampled history does not describe the handoff cycle.
    a_fall_only_on_handoff: assert property (
        @(posedge clk) disable iff (!rst_n)
        $fell(pending) |-> reply_sent
    ) else $error("%m: pending cleared without a MAC handoff");

    // The held value is stable for the whole wait.
    a_stable: assert property (
        @(posedge clk) disable iff (!rst_n)
        (pending && !(driving && mac_busy)) |=> $stable(value_q)
    ) else $error("%m: pending reply value changed");
`endif

endmodule : tx_reply_ctrl
