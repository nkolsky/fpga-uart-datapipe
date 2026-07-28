// cdc_cmd_sync.sv
// ---------------
// Atomic single-entry command clock-domain crossing.
//
// Transfers a complete bus transaction -- {is_write, addr, wdata} -- from
// a source clock domain to a destination clock domain, delivering it as
// exactly one destination-domain strobe with all fields valid together.
//
// Built on cdc_pulse_sync, which is already hardware-validated. No new
// synchroniser logic is written here: this module adds the payload
// register and the qualified destination outputs around that primitive.
//
// -----------------------------------------------------------------------
// WHY A ONE-ENTRY HANDSHAKE AND NOT A FIFO
// -----------------------------------------------------------------------
// A FIFO earns its place when the source can burst faster than the
// destination drains. Here the source is rate-limited by the UART: one
// legacy RGF command per 16-byte message, ~21.7 us at 8.125 Mbaud, which
// is ~2820 source cycles at 130 MHz. Synchroniser latency is ~3
// destination cycles. Occupancy therefore never exceeds one.
//
// -----------------------------------------------------------------------
// ATOMICITY -- the whole point of this module
// -----------------------------------------------------------------------
// The payload register is written on the SAME source edge that flips the
// toggle inside cdc_pulse_sync. The toggle then needs at least two
// destination edges to traverse the synchroniser before dst_valid can
// assert, so by the time the destination samples the payload it has been
// stable for >= 2 destination clocks.
//
// The data therefore crosses WITHOUT per-bit synchronisation, and that is
// correct rather than a shortcut: synchronising the bits individually
// would actively BREAK atomicity, since each bit could resolve on a
// different edge and the destination could observe a mixture of two
// different transactions. One toggle covers all ADDR_W + DATA_W + 1 bits.
//
// -----------------------------------------------------------------------
// IDLE_ADDR -- structural protection for level-sensitive decodes
// -----------------------------------------------------------------------
// dst_addr presents IDLE_ADDR on every cycle where dst_valid is low.
//
// This is inside the module deliberately, rather than left as a mux at
// the instantiation site. The RGF this feeds implements read-to-clear as
// a LEVEL-SENSITIVE decode:
//
//     if (!pc_wen && (pc_addr == IMG_TX_MON_ADDR)) ...   // rgf.sv:134
//
// If dst_addr held the last command address continuously, IMG_TX_MON
// would be cleared on every single cycle, so the completion flag could
// never latch and the start interlock would never engage. That failure
// is silent -- no error, no assertion, just an interlock that quietly
// stops working. Making the idle value structural means the module
// cannot be instantiated in a way that reintroduces it.
//
// IDLE_ADDR must be an address the destination decodes to nothing. For
// the RGF it is 8'hFF, which matches none of 0x00/0x04/0x08/0x0C/0x10/0x14.
//
// -----------------------------------------------------------------------
// MINIMUM SPACING
// -----------------------------------------------------------------------
// This is a two-phase (toggle-only) handshake with no acknowledge, so the
// source must not issue a second command before the previous toggle has
// propagated: about 3 destination clocks, i.e. ~4 source cycles at
// 130 -> 100 MHz. The UART rate guarantees ~2820, a margin of ~700x.
//
// A four-phase request/acknowledge scheme would remove that assumption,
// but it would also need a busy signal fed back to stall the source --
// real plumbing for a case that cannot occur here. Instead the assumption
// is policed by the simulation-only assertion at the bottom of this file,
// which turns a violation into a loud failure rather than a silent
// dropped command.

`timescale 1ns/1ps

module cdc_cmd_sync #(
    parameter int                ADDR_W                 = 8,
    parameter int                DATA_W                 = 32,
    // Address presented whenever dst_valid is low. Must decode to nothing
    // in the destination address map.
    parameter logic [ADDR_W-1:0] IDLE_ADDR              = '1,
    // Destination synchroniser depth, passed through to cdc_pulse_sync.
    parameter int                SYNC_STAGES            = 2,
    // Simulation-only spacing check, in source clocks.
    parameter int                MIN_SPACING_SRC_CYCLES = 8
)(
    // ---- source domain ----------------------------------------------
    input  logic              src_clk,
    input  logic              src_rst_n,
    input  logic              src_valid,     // one source-clock pulse
    input  logic              src_is_write,  // 1 = write, 0 = read
    input  logic [ADDR_W-1:0] src_addr,
    input  logic [DATA_W-1:0] src_wdata,

    // ---- destination domain -----------------------------------------
    output logic              dst_valid,     // one destination-clock pulse
    output logic              dst_is_write,
    output logic [ADDR_W-1:0] dst_addr,      // IDLE_ADDR while !dst_valid
    output logic [DATA_W-1:0] dst_wdata,     // held between transfers
    input  logic              dst_clk,
    input  logic              dst_rst_n
);

    // -----------------------------------------------------------------
    // Payload capture, source domain.
    //
    // Written on the same edge that flips the toggle below. That
    // simultaneity is what makes the transfer atomic -- see the header.
    // -----------------------------------------------------------------
    logic              cap_is_write;
    logic [ADDR_W-1:0] cap_addr;
    logic [DATA_W-1:0] cap_wdata;

    always_ff @(posedge src_clk or negedge src_rst_n) begin
        if (!src_rst_n) begin
            cap_is_write <= 1'b0;
            cap_addr     <= '0;
            cap_wdata    <= '0;
        end else if (src_valid) begin
            cap_is_write <= src_is_write;
            cap_addr     <= src_addr;
            cap_wdata    <= src_wdata;
        end
    end

    // -----------------------------------------------------------------
    // Event path.
    //
    // Reuses the hardware-validated toggle synchroniser from Change B.
    // One src_valid in, exactly one dst_valid out -- never zero, never
    // two -- regardless of the clock ratio or phase relationship.
    // -----------------------------------------------------------------
    cdc_pulse_sync #(
        .SYNC_STAGES (SYNC_STAGES)
    ) u_event_sync (
        .src_clk   (src_clk),
        .src_rst_n (src_rst_n),
        .src_pulse (src_valid),
        .dst_clk   (dst_clk),
        .dst_rst_n (dst_rst_n),
        .dst_pulse (dst_valid)
    );

    // -----------------------------------------------------------------
    // Destination outputs.
    //
    // dst_addr / dst_is_write are qualified by dst_valid so the
    // transaction is presented for exactly one destination cycle. This is
    // what makes a level-sensitive read decode fire once and only once.
    //
    // dst_wdata is deliberately NOT qualified: it simply holds the last
    // captured value. The destination only samples write data while its
    // write enable is asserted, which by construction is only during the
    // dst_valid cycle, so gating it would add a mux for no benefit.
    // -----------------------------------------------------------------
    assign dst_is_write = dst_valid ? cap_is_write : 1'b0;
    assign dst_addr     = dst_valid ? cap_addr     : IDLE_ADDR;
    assign dst_wdata    = cap_wdata;

    // -----------------------------------------------------------------
    // Simulation-only minimum-spacing check.
    //
    // Purely source-domain -- no cross-clock assertion, no sampling of
    // destination signals. gap_cnt saturates rather than wrapping, so a
    // long idle period cannot alias back into a false failure, and it
    // starts saturated so the first command after reset always passes.
    //
    // Zero synthesis footprint: the counter itself is inside the guard.
    // -----------------------------------------------------------------
`ifndef SYNTHESIS
    localparam int GAP_W = 16;
    logic [GAP_W-1:0] gap_cnt;

    always_ff @(posedge src_clk or negedge src_rst_n) begin
        if (!src_rst_n)                gap_cnt <= '1;
        else if (src_valid)            gap_cnt <= '0;
        else if (gap_cnt != '1)        gap_cnt <= gap_cnt + 1'b1;
    end

    a_min_spacing: assert property (
        @(posedge src_clk) disable iff (!src_rst_n)
        src_valid |-> (gap_cnt >= GAP_W'(MIN_SPACING_SRC_CYCLES))
    )
    else $error("%m: src_valid pulses only %0d source cycles apart, minimum is %0d. A command may be lost.",
                gap_cnt, MIN_SPACING_SRC_CYCLES);
`endif

endmodule : cdc_cmd_sync
