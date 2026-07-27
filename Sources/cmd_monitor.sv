// cmd_monitor.sv
// --------------
// TEMPORARY STAGE 2B VERIFICATION AID -- DELETE IN STAGE 2C.
//
// Stage 2B builds the pixel-command clock-domain crossing only. There is no
// SRAM write controller, no address mapping and no arbitration yet, so the
// 100 MHz side of the command FIFO has no real consumer. This module is that
// consumer: it drains the FIFO and provides a single hardware-observable
// indication that a command survived the crossing.
//
// It is deliberately NOT a proto write controller. When Stage 2C adds the
// real one, this module and its LED are removed outright.
//
// -----------------------------------------------------------------------
// READ TIMING -- CAPTURE ONE CYCLE AFTER THE POP
// -----------------------------------------------------------------------
// async_fifo registers its output unconditionally from the PRE-INCREMENT
// pointer:
//
//     always_ff @(posedge rd_clk) rd_data <= fifo_mem[rd_ptr_bin];
//     ... else if (rd_en && !empty) rd_ptr_bin <= rd_ptr_bin + 1;
//
// Both blocks fire on the same edge and both read the pre-edge pointer, so
// asserting rd_en in cycle N advances the pointer AND loads rd_data with the
// entry that rd_en just consumed. That entry is therefore visible in cycle
// N+1, and back-to-back pops deliver distinct entries in order:
//
//   cycle  rd_ptr  rd_data  rd_en  ->  next rd_data  next rd_ptr  capture
//     1      0        A       1          A               1          -
//     2      1        A       1          B               2          A
//     3      2        B       1          C               3          B
//     4      3        C       0          -               3          C
//
// NO GAP IS REQUIRED between pops. A gap is only needed by a consumer that
// captures in the SAME cycle as rd_en -- which is what tx_sequencer does, and
// is safe there only because its FSM never pops on consecutive cycles.
//
// This module uses the one-cycle-later discipline, so it drains at one entry
// per clock.
//
// -----------------------------------------------------------------------
// WHY pop_q IS REGISTERED FROM (cmd_rd_en && !cmd_empty)
// -----------------------------------------------------------------------
// The capture qualifier must track the DATA, not the current FIFO state.
// Popping the last entry in cycle N makes cmd_empty assert in cycle N+1 --
// exactly the cycle the final word is on rd_data. Gating the capture on live
// cmd_empty would discard it. Registering the ACCEPTED pop condition instead
// keeps the qualifier aligned with the data it refers to.
//
// The redundant "&& !cmd_empty" is intentional even though cmd_rd_en is
// currently driven from !cmd_empty: it states the accept condition
// explicitly and keeps this timing contract independent of the drain policy
// above, which Stage 2C will replace.

`timescale 1ns/1ps

module cmd_monitor #(
    parameter int CMD_W = 48
)(
    input  logic             clk,
    input  logic             rst_n,

    // ---- command FIFO read side ---------------------------------------
    input  logic             cmd_empty,
    input  logic [CMD_W-1:0] cmd_rd_data,
    output logic             cmd_rd_en,

    // ---- observability -------------------------------------------------
    output logic             cmd_seen,   // sticky: a command was captured
    output logic [CMD_W-1:0] cmd_last    // most recently captured command
);

    // -----------------------------------------------------------------
    // Drain policy for Stage 2B: pop whenever anything is queued.
    //
    // The producer is UART-rate limited to roughly one command per 15 us
    // (an 11-byte Single Pixel Write at 8.125 Mbaud), while this drains at
    // one per 10 ns. Occupancy therefore never exceeds one and the FIFO
    // cannot fill -- which is why Stage 2B needs no almost_full term in
    // UART_CTS. The sticky overflow flag in chip_top exists to prove that,
    // not to mitigate it.
    // -----------------------------------------------------------------
    assign cmd_rd_en = !cmd_empty;

    logic pop_q;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pop_q    <= 1'b0;
            cmd_last <= '0;
            cmd_seen <= 1'b0;
        end else begin
            pop_q <= cmd_rd_en && !cmd_empty;

            if (pop_q) begin
                cmd_last <= cmd_rd_data;
                cmd_seen <= 1'b1;
            end
        end
    end

    // -----------------------------------------------------------------
    // Note on cmd_last: nothing consumes it in Stage 2B, so synthesis will
    // optimise the register away. That is expected and harmless -- cmd_seen
    // depends only on pop_q and survives. cmd_last exists so the simulation
    // testbenches can check the exact 48-bit payload, which is where
    // payload-level verification belongs at this milestone. If hardware
    // payload inspection is ever needed, add mark_debug / an ILA rather
    // than a DONT_TOUCH attribute.
    // -----------------------------------------------------------------

endmodule : cmd_monitor
