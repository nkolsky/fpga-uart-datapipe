`timescale 1ns/1ps
// apb_slave_rgf.sv
// ================
// APB slave front end for the register file.
//
//   apb_bar --APB--> apb_slave_rgf --pc_*--> rgf
//
// rgf keeps its existing mem-style port; this module is the translation and
// nothing else. Address offset, write strobe, read strobe, and the
// PREADY/PRDATA/PSLVERR response. No state of its own unless wait states are
// enabled.
//
// WHY pc_ren EXISTS
// rgf's IMG_TX_MON read-to-clear used to be a LEVEL decode with no access
// qualifier:
//
//     else if (!pc_wen && pc_sel_img_tx_mon)      // before
//
// Nothing in that expression says "a read is happening". Any cycle in which
// pc_addr sat at IMG_TX_MON_ADDR with no write in progress cleared
// img_send_complete and img_send_error -- the two bits the IMG_CTRL start
// interlock tests. A driver that simply held its last address disabled the
// interlock silently.
//
// The mitigation was parking pc_addr at rgf_pkg::IDLE_ADDR whenever no command
// was in flight, an obligation that has already moved from cdc_cmd_sync to
// mem_msg_router and left an explanatory comment in three files. pc_ren is a
// real one-cycle access strobe, so rgf becomes:
//
//     else if (pc_ren && pc_sel_img_tx_mon)       // after
//
// and pc_addr may sit wherever it likes between transfers. The hazard is
// deleted rather than relocated.
//
// TIMING
// rgf's read mux is combinational on pc_addr, and APB holds address and
// control stable from SETUP through ACCESS, so PRDATA is valid the moment
// PENABLE rises. WAIT_STATES defaults to 0 and a transfer is two cycles.
//
// The parameter is not decoration: a non-zero value is the only way to
// exercise the master's wait-state path, and a waveform showing PENABLE held
// with PREADY low is worth more in a report than one where every transfer
// looks identical.

module apb_slave_rgf
    import apb_pkg::*;
#(
    // Cycles PREADY is held low inside ACCESS. 0 = zero-wait, two-cycle
    // transfer, which is what the RGF can actually do.
    parameter int WAIT_STATES = 0
)(
    input  logic clk,
    input  logic rst_n,

    // ---- APB ---------------------------------------------------------
    input  logic                  psel,
    input  logic                  penable,
    input  logic                  pwrite,
    input  logic [ADDR_W-1:0]     paddr,
    input  logic [DATA_W-1:0]     pwdata,
    output logic                  pready,
    output logic [DATA_W-1:0]     prdata,
    output logic                  pslverr,

    // ---- rgf mem-style port ------------------------------------------
    output logic [OFFSET_W-1:0]   pc_addr,
    output logic                  pc_wen,
    output logic                  pc_ren,
    output logic [DATA_W-1:0]     pc_wdata,
    input  logic [DATA_W-1:0]     pc_rdata
);

    // -----------------------------------------------------------------
    // Wait-state counter. Disappears entirely when WAIT_STATES == 0.
    // -----------------------------------------------------------------
    generate
        if (WAIT_STATES == 0) begin : g_zero_wait
            assign pready = 1'b1;
        end
        else begin : g_wait
            localparam int CNT_W = $clog2(WAIT_STATES + 1);
            logic [CNT_W-1:0] cnt;

            always_ff @(posedge clk or negedge rst_n) begin
                if (!rst_n)                            cnt <= '0;
                else if (!(psel && penable))           cnt <= '0;
                else if (cnt != CNT_W'(WAIT_STATES))   cnt <= cnt + 1'b1;
            end

            assign pready = (psel && penable) && (cnt == CNT_W'(WAIT_STATES));
        end
    endgenerate

    // -----------------------------------------------------------------
    // Access strobes
    //
    // Both qualified by pready, so each is exactly one cycle wide no matter
    // how many wait states were inserted. rgf sees one write or one read per
    // transfer, never a level held across the whole ACCESS phase.
    // -----------------------------------------------------------------
    logic access;
    assign access = psel && penable && pready;

    assign pc_wen   = access &&  pwrite;
    assign pc_ren   = access && !pwrite;
    assign pc_addr  = addr_offset(paddr);
    assign pc_wdata = pwdata;

    // rgf's read mux is combinational and already returns 0 for an
    // unimplemented offset, so PRDATA needs no qualification here.
    assign prdata   = pc_rdata;

    // The RGF cannot fail an access that reached it: every offset either
    // decodes to a register or reads as zero. Unmapped APERTURES are
    // apb_bar's business, not this slave's.
    assign pslverr  = 1'b0;

`ifndef SYNTHESIS
    // This is the check apb_pkg deliberately does not make: the package
    // declares OFFSET_W and DATA_W independently so the bus does not inherit
    // its shape from one slave, and the coupling is asserted HERE, where it
    // actually exists. Same pattern rx_classifier uses for ADDR_W_RGF.
    initial begin
        if (OFFSET_W != rgf_pkg::ADDR_WIDTH)
            $error("%m: apb_pkg::OFFSET_W (%0d) != rgf_pkg::ADDR_WIDTH (%0d)",
                   OFFSET_W, rgf_pkg::ADDR_WIDTH);
        if (DATA_W != rgf_pkg::DATA_WIDTH)
            $error("%m: apb_pkg::DATA_W (%0d) != rgf_pkg::DATA_WIDTH (%0d)",
                   DATA_W, rgf_pkg::DATA_WIDTH);
    end

    // A write and a read must never be signalled together: rgf's read mux
    // returns 0 while pc_wen is high, so this would silently corrupt a reply.
    a_not_both: assert property (
        @(posedge clk) disable iff (!rst_n)
        !(pc_wen && pc_ren)
    ) else $error("%m: write and read strobes asserted together");

    // One strobe per transfer. If this fires, pready has come loose from the
    // strobe qualification and rgf is seeing a held level again -- the exact
    // failure mode this module exists to remove.
    a_single_cycle: assert property (
        @(posedge clk) disable iff (!rst_n)
        (pc_wen || pc_ren) |=> !(pc_wen || pc_ren)
    ) else $error("%m: access strobe held for more than one cycle");
`endif

endmodule : apb_slave_rgf
