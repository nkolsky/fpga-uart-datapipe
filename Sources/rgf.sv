// rgf.sv
// ------
// Config RGF - Lab 9/10 register file. SIX registers; see rgf_pkg.sv for
// the full map, addresses and access types.
//
// Robustness additions (invalid-address latching, reserved-bit write
// masking) remain a deliberate follow-up and are not implemented here.
// An unmapped read returns 0 and an unmapped write is silently dropped;
// malformed addresses are already refused upstream by the register
// parsers, so they do not reach this module.
//
// -----------------------------------------------------------------------
// HOW EACH REGISTER IS WRITTEN -- THREE MECHANISMS, NO ARBITRATION NEEDED
// -----------------------------------------------------------------------
// There is no single "write port". Registers are updated by three
// separate, non-overlapping mechanisms, which is why no arbitration
// exists anywhere in this file:
//
//   1. PC port (pc_wen / pc_addr / pc_wdata)
//        writes IMG_CTRL and CLK_CTRL -- and nothing else.
//        Only these two are PC-writable.
//
//   2. Status port (status_wen / status_addr / status_wdata)
//        writes IMG_TX_MON -- and nothing else.
//        An earlier version of this header also claimed FIFO_STATUS was
//        written through this bus. It is not, and never was in this
//        implementation: see mechanism 3.
//
//   3. Dedicated hardware inputs, bypassing both buses entirely
//        IMG_STATUS       <- img_height_in / img_width_in / img_ready_in
//        FIFO_STATUS      <- fifo_full / empty / almost_full / almost_empty
//                            (combinational passthrough, no storage at all)
//        PARITY_FAULT_CNT <- parity_fault_incr (single-bit increment pulse,
//                            not a value write -- the status_* bus has
//                            "write this whole value" semantics and cannot
//                            express "add one")
//
// No register is targeted by more than one mechanism. The ONLY place two
// mechanisms can touch the same register in the same cycle is IMG_TX_MON,
// where a status write can coincide with a PC read-to-clear -- resolved by
// explicit priority below.
//
// IMG_CTRL interlock (see rgf_pkg.sv header for the full reasoning):
//   A '1' write to start_img_read is accepted only if, at the moment of
//   the write, IMG_TX_MON.img_send_complete==0 AND
//   IMG_TX_MON.img_send_error==0 ("both cleared", per the literal spec
//   text). A '0' write always succeeds (clearing the bit).
//
// IMG_TX_MON read-to-clear:
//   A PC READ of IMG_TX_MON (pc_addr selects it, pc_wen low) clears
//   img_send_complete and img_send_error back to 0 on that same edge --
//   this is what re-arms the interlock above for the next transfer.
//   row_cnt/col_cnt are untouched by a read; they're live progress
//   counters, not one-shot event flags.
//   Priority: if a status-port write and a PC read land on IMG_TX_MON in
//   the same cycle, the status write wins (a fresh hardware event should
//   never be silently erased by a same-cycle read; the PC will see it on
//   its next read instead).

`timescale 1ns/1ps

import rgf_pkg::*;

// ADDR_WIDTH IS QUALIFIED EVERYWHERE IN THIS FILE, DELIBERATELY.
//
// The name is defined in BOTH fifo_pkg (6, from $clog2(DEPTH)) and rgf_pkg
// (8). Both are wildcard-imported into the shared compilation unit, so an
// unqualified ADDR_WIDTH resolves by COMPILE ORDER -- and fifo_pkg comes
// first, which silently made these address ports 6 bits instead of 8.
//
// The symptom was a width warning at every address comparison in this file,
// since the register addresses in rgf_pkg are declared 8 bits wide. The
// effect on hardware would be truncation of any address at or above 64.

module rgf (
    input  logic clk,
    input  logic rst_n,

    // -----------------------------------------------------------------
    // PC-facing port - driven by Sequencer on behalf of parsed PC commands
    // -----------------------------------------------------------------
    input  logic                  pc_wen,
    input  logic [rgf_pkg::ADDR_WIDTH-1:0] pc_addr,
    input  logic [DATA_WIDTH-1:0] pc_wdata,
    output logic [DATA_WIDTH-1:0] pc_rdata,

    // -----------------------------------------------------------------
    // Internal status-update port - driven by the Sequencer during image
    // drain to keep IMG_TX_MON current. IMG_TX_MON is the ONLY register
    // reachable through this bus; FIFO_STATUS arrives on its own dedicated
    // inputs below. Write-only; the Sequencer never needs to read back
    // through this port.
    // -----------------------------------------------------------------
    input  logic                  status_wen,
    input  logic [rgf_pkg::ADDR_WIDTH-1:0] status_addr,
    input  logic [DATA_WIDTH-1:0] status_wdata,

    // -----------------------------------------------------------------
    // Static image dimension inputs - IMG_STATUS.img_height/img_width
    // are fixed at build time (from rgb_rom parameters), not written
    // through either write port; img_ready is asserted once and held.
    // -----------------------------------------------------------------
    input  logic [9:0]            img_height_in,
    input  logic [9:0]            img_width_in,
    input  logic                  img_ready_in,

    // -----------------------------------------------------------------
    // Direct output to Sequencer - pulses high the cycle a successful
    // (interlock-passing) write to IMG_CTRL.start_img_read happens. Lets
    // the Sequencer react immediately without polling pc_rdata.
    // -----------------------------------------------------------------
    output logic                  start_img_read_out,

    // Direct output to the clock mux (Lab 10) - continuous level (not a
    // pulse like start_img_read_out), since clk_sel needs to persist
    // until explicitly changed, not fire-and-clear. Drives the counter
    // system's clock select -- see chip_top.sv's PLL/clock mux status
    // block for the full picture.
    output logic                  clk_sel_out,

    // Dedicated increment pulse for PARITY_FAULT_CNT (Lab 10) - driven
    // by rx_phy.parity_err_pulse via chip_top.sv. Single-bit, not the
    // shared status_* bus -- see rgf_pkg.sv for why.
    input  logic                  parity_fault_incr,

    // Dedicated FIFO_STATUS inputs (Lab 10) - continuous passthrough
    // from async_fifo, wired directly in chip_top.sv. Not the status_*
    // bus -- see rgf_pkg.sv/FIFO_STATUS section for why.
    input  logic                  fifo_full,
    input  logic                  fifo_empty,
    input  logic                  fifo_almost_full,
    input  logic                  fifo_almost_empty
);

// -------------------------------------------------------------------------
// IMG_STATUS (RO) - driven directly from static inputs, no write-port
// decode needed at all. Simple registered pass-through.
// -------------------------------------------------------------------------
img_status_t img_status_reg;

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        img_status_reg <= '0;
    end else begin
        img_status_reg.img_height <= img_height_in;
        img_status_reg.img_width  <= img_width_in;
        img_status_reg.img_ready  <= img_ready_in;
    end
end

// -------------------------------------------------------------------------
// IMG_TX_MON (IW / IW+RC) - written via status_* port when status_addr
// selects it; img_send_complete/img_send_error additionally clear on a
// PC read (see module header). Reset is plain '0 -- both complete and
// error start cleared, which already satisfies the IMG_CTRL interlock
// with no special-case reset value needed.
// -------------------------------------------------------------------------
img_tx_mon_t img_tx_mon_reg;

logic status_sel_img_tx_mon;
assign status_sel_img_tx_mon = (status_addr == IMG_TX_MON_ADDR);

logic pc_sel_img_tx_mon;
assign pc_sel_img_tx_mon = (pc_addr == IMG_TX_MON_ADDR);

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        img_tx_mon_reg <= '0;
    end else if (status_wen && status_sel_img_tx_mon) begin
        // Fresh status update from the Sequencer takes priority over a
        // same-cycle read-clear (see module header priority note).
        img_tx_mon_reg.row_cnt           <= status_wdata[IMG_TX_MON_row_STOP:IMG_TX_MON_row_START];
        img_tx_mon_reg.col_cnt           <= status_wdata[IMG_TX_MON_col_STOP:IMG_TX_MON_col_START];
        img_tx_mon_reg.img_send_complete <= status_wdata[IMG_TX_MON_complete_START];
        img_tx_mon_reg.img_send_error    <= status_wdata[IMG_TX_MON_error_START];
    end else if (!pc_wen && pc_sel_img_tx_mon) begin
        // PC read: read-to-clear on complete/error only. row_cnt/col_cnt
        // hold their live value -- pc_rdata below still returns the
        // pre-clear value combinationally on this same cycle.
        img_tx_mon_reg.img_send_complete <= 1'b0;
        img_tx_mon_reg.img_send_error    <= 1'b0;
    end
end

// -------------------------------------------------------------------------
// IMG_CTRL (RW) - written via pc_* port when pc_addr selects it.
// Interlock enforced here: start_img_read is only accepted (and only
// pulses start_img_read_out) if img_send_complete and img_send_error are
// BOTH clear at the moment of the write, per the literal spec wording.
// -------------------------------------------------------------------------
img_ctrl_t img_ctrl_reg;

logic pc_sel_img_ctrl;
assign pc_sel_img_ctrl = (pc_addr == IMG_CTRL_ADDR);

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        img_ctrl_reg        <= '0;
        start_img_read_out  <= 1'b0;
    end else begin
        // default: pulse low unless this cycle's write succeeds
        start_img_read_out <= 1'b0;

        if (pc_wen && pc_sel_img_ctrl) begin
            if (pc_wdata[IMG_CTRL_start_START] &&
                !img_tx_mon_reg.img_send_complete &&
                !img_tx_mon_reg.img_send_error) begin
                // interlock passed (both cleared): accept the start bit,
                // pulse the output
                img_ctrl_reg.start_img_read <= 1'b1;
                start_img_read_out          <= 1'b1;
            end else if (!pc_wdata[IMG_CTRL_start_START]) begin
                // PC writing 0 to the start bit always succeeds (clear)
                img_ctrl_reg.start_img_read <= 1'b0;
            end
            // else: interlock failed on an attempted '1' write - silently
            // dropped, img_ctrl_reg unchanged, no pulse generated
        end
    end
end

// -------------------------------------------------------------------------
// FIFO_STATUS (RO) - dedicated ports, continuous passthrough. NOT the
// status_* bus -- these are already-live hardware levels from
// async_fifo, not a one-shot value that needs capturing at an event
// (see rgf_pkg.sv for the full reasoning). No clock/reset needed here
// at all -- it's a plain wire-through, same pattern as IMG_STATUS.
// -------------------------------------------------------------------------
fifo_status_t fifo_status_reg;

always_comb begin
    fifo_status_reg.full         = fifo_full;
    fifo_status_reg.empty        = fifo_empty;
    fifo_status_reg.almost_full  = fifo_almost_full;
    fifo_status_reg.almost_empty = fifo_almost_empty;
    fifo_status_reg.pad          = '0;
end

// -------------------------------------------------------------------------
// CLK_CTRL (RW) - written via pc_* port when pc_addr selects it.
// No interlock (see rgf_pkg.sv) -- any write of bit 0 is accepted
// unconditionally. clk_sel_out is a continuous level, not a pulse.
// Drives the clock mux selecting the counter system's clock speed.
// -------------------------------------------------------------------------
clk_ctrl_t clk_ctrl_reg;

logic pc_sel_clk_ctrl;
assign pc_sel_clk_ctrl = (pc_addr == CLK_CTRL_ADDR);

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        clk_ctrl_reg <= '0;   // default: clk_sel=0, clk0/CLK100MHZ fallback
    end else if (pc_wen && pc_sel_clk_ctrl) begin
        clk_ctrl_reg.clk_sel <= pc_wdata[CLK_CTRL_sel_START];
    end
end

assign clk_sel_out = clk_ctrl_reg.clk_sel;

// -------------------------------------------------------------------------
// PARITY_FAULT_CNT (RO, Lab 10) - plain monotonic counter, no bitfields.
// Increments by 1 each time parity_fault_incr pulses. Not saturating --
// 32 bits is large enough that wraparound is not a realistic concern for
// this lab. Not read-to-clear/write-to-clear in this pass (see
// rgf_pkg.sv) -- only chip reset clears it.
// -------------------------------------------------------------------------
logic [31:0] parity_fault_cnt_reg;

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        parity_fault_cnt_reg <= '0;
    end else if (parity_fault_incr) begin
        parity_fault_cnt_reg <= parity_fault_cnt_reg + 32'd1;
    end
end

// -------------------------------------------------------------------------
// Read mux - pc_rdata selects the addressed register, combinational
// (zero-latency read; only active when pc_wen is low so a single-cycle
// read doesn't collide with a same-cycle write to the same address).
// -------------------------------------------------------------------------
always_comb begin
    if (pc_wen) begin
        pc_rdata = '0; // don't read while writing
    end else begin
        case (pc_addr)
            IMG_STATUS_ADDR:       pc_rdata = img_status_reg;
            IMG_TX_MON_ADDR:       pc_rdata = img_tx_mon_reg;
            IMG_CTRL_ADDR:         pc_rdata = img_ctrl_reg;
            FIFO_STATUS_ADDR:      pc_rdata = fifo_status_reg;
            CLK_CTRL_ADDR:         pc_rdata = clk_ctrl_reg;
            PARITY_FAULT_CNT_ADDR: pc_rdata = parity_fault_cnt_reg;
            default:               pc_rdata = '0; // unimplemented address returns 0
        endcase
    end
end

endmodule : rgf