`timescale 1ns/1ps
// ahb_decoder.sv
// ==============
// Address decode, response multiplexing and the default slave for the
// AHB-Lite segment.
//
//   ahb_master --HADDR/HTRANS--> ahb_decoder --HSEL[2:0]--> R / G / B slaves
//              <--HREADY/HRDATA/HRESP--        <--HREADYOUT/HRDATA/HRESP--
//
// THE RESPONSE MUX FOLLOWS THE DATA PHASE, NOT THE ADDRESS PHASE
//
// AHB overlaps phases: while beat N's data comes back, beat N+1's address is
// already on the bus. So HADDR no longer points at the slave whose response
// is arriving. Selecting the response mux from the live decode returns the
// WRONG slave's data the moment two consecutive beats target different
// slaves -- which is exactly what round-robin across R/G/B does.
//
// dsel below is the decode REGISTERED one phase behind, and the response mux
// uses it. dsel updates only when HREADY is high, so a stretched data phase
// keeps pointing at the right slave.
//
// THE ERROR RESPONSE IS TWO CYCLES
//
// AHB-Lite defines it as: HRESP=ERROR with HREADYOUT low, then HRESP=ERROR
// with HREADYOUT high. A one-cycle error is not protocol-legal, and a
// manager that samples HRESP only when HREADY is high would miss it
// entirely. err_second below is that second cycle.
//
// Without a default slave an unmapped address is far worse than an error:
// nothing drives HREADY, the manager waits forever, and the arbiter never
// gets its grant back. One bad address locks the bus permanently.
//
// HREADY OUT OF HERE IS THE SEGMENT READY
//
// m_hready goes to the manager AND back into every slave as their hready
// input. A slave needs it to know whether the address phase it can see is
// real or is being stretched by somebody else.

module ahb_decoder
    import ahb_pkg::*;
(
    input  logic                    hclk,
    input  logic                    hresetn,

    // ---- from the manager -------------------------------------------------
    input  logic [ADDR_W-1:0]       haddr,
    input  logic [1:0]              htrans,

    // ---- to the manager ---------------------------------------------------
    output logic                    m_hready,
    output logic [DATA_W-1:0]       m_hrdata,
    output logic                    m_hresp,

    // ---- to the slaves ----------------------------------------------------
    output logic [NUM_SLAVES-1:0]   hsel,

    // ---- from the slaves --------------------------------------------------
    input  logic [NUM_SLAVES-1:0]   s_hreadyout,
    input  logic [NUM_SLAVES-1:0]   s_hresp,
    input  logic [DATA_W-1:0]       s_hrdata [NUM_SLAVES],

    // ---- diagnostics -------------------------------------------------------
    output logic                    decode_err   // one cycle, on an unmapped access
);

    // -----------------------------------------------------------------
    // Address-phase decode
    //
    // HSEL is a pure address decode, as AHB defines it. Each slave qualifies
    // it with HTRANS itself, so a select asserted during IDLE is harmless.
    // -----------------------------------------------------------------
    logic [SEL_W-1:0] sel;
    assign sel = addr_sel(haddr);

    logic [NUM_SLAVES-1:0] hit;
    for (genvar i = 0; i < NUM_SLAVES; i++) begin : g_decode
        assign hit[i]  = (sel == SLAVE_BASE[i]);
        assign hsel[i] = hit[i];
    end : g_decode

    // A real transfer that matches nothing. IDLE and BUSY move no data, so
    // they are not unmapped accesses even at an unmapped address.
    logic xfer_active, no_match;
    assign xfer_active = (htrans == HTRANS_NONSEQ) || (htrans == HTRANS_SEQ);
    assign no_match    = xfer_active && !(|hit);

    // -----------------------------------------------------------------
    // Carry the decode into the data phase
    // -----------------------------------------------------------------
    logic [NUM_SLAVES-1:0] dsel;
    logic                  dsel_none;

    always_ff @(posedge hclk or negedge hresetn) begin
        if (!hresetn) begin
            dsel      <= '0;
            dsel_none <= 1'b0;
        end
        else if (m_hready) begin
            dsel      <= hit & {NUM_SLAVES{xfer_active}};
            dsel_none <= no_match;
        end
    end

    // -----------------------------------------------------------------
    // Default slave: two-cycle ERROR
    // -----------------------------------------------------------------
    // dsel_none IS the data phase of an unmapped access -- named for what it
    // means at the point of use.
    logic no_match_dphase;
    assign no_match_dphase = dsel_none;

    logic err_second;

    always_ff @(posedge hclk or negedge hresetn) begin
        if (!hresetn) err_second <= 1'b0;
        else          err_second <= no_match_dphase && !err_second;
    end

    assign decode_err = no_match_dphase && err_second;   // completing cycle

    // -----------------------------------------------------------------
    // Response mux
    //
    // Unselected slaves are ignored rather than trusted to drive zero.
    // Default when nothing is selected is READY -- an idle bus must not stall
    // the manager.
    // -----------------------------------------------------------------
    logic              mux_hready;
    logic              mux_hresp;
    logic [DATA_W-1:0] mux_hrdata;

    always_comb begin : rsp_mux
        mux_hready = 1'b1;
        mux_hresp  = HRESP_OKAY;
        mux_hrdata = '0;
        for (int i = 0; i < NUM_SLAVES; i++) begin
            if (dsel[i]) begin
                mux_hready = s_hreadyout[i];
                mux_hresp  = s_hresp[i];
                mux_hrdata = s_hrdata[i];
            end
        end
    end : rsp_mux

    assign m_hready = no_match_dphase ? err_second   : mux_hready;
    assign m_hresp  = no_match_dphase ? HRESP_ERROR  : mux_hresp;
    assign m_hrdata = no_match_dphase ? '0           : mux_hrdata;

`ifndef SYNTHESIS
    // At most one aperture may claim an address. Overlapping SLAVE_BASE
    // entries would silently merge two slaves' responses.
    always_comb begin : a_onehot
        assert ($onehot0(hit))
        else $error("%m: overlapping apertures -- more than one slave decoded");
    end : a_onehot

    // The data-phase select must never point at two slaves.
    a_dsel_onehot: assert property (
        @(posedge hclk) disable iff (!hresetn)
        $onehot0(dsel)
    ) else $error("%m: data-phase select is not one-hot");

    // An unmapped access must never leave HREADY low forever. Two cycles,
    // then done.
    a_err_terminates: assert property (
        @(posedge hclk) disable iff (!hresetn)
        (no_match_dphase && !err_second) |=> err_second
    ) else $error("%m: error response did not complete in two cycles");

    // A mapped and an unmapped response must never both be driving.
    a_not_both: assert property (
        @(posedge hclk) disable iff (!hresetn)
        !(no_match_dphase && (|dsel))
    ) else $error("%m: default slave active while a real slave is selected");
`endif

endmodule : ahb_decoder
