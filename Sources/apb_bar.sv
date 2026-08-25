`timescale 1ns/1ps
// apb_bar.sv
// ==========
// Address decode and response mux for the APB bus -- the BAR.
//
//   apb_master --> apb_bar --> psel[0]  apb_slave_rgf
//                          --> default responder, for everything unmapped
//
// Two jobs:
//   DECODE   PADDR[15:8] picks one aperture. Exactly one psel bit is set
//            while a transfer is active, or none if nothing matches.
//   MUX      PREADY / PRDATA / PSLVERR come back from whichever slave was
//            selected, or from the default responder if none was.
//
// Purely combinational -- no clock port, no state. It adds no cycle to a
// transfer, so apb_master's two-cycle figure holds with the decoder in place.
//
// THE DEFAULT RESPONDER IS A DEADLOCK GUARD
// If no aperture matches, no slave drives PREADY. Left alone that is a HANG:
// apb_master sits in M_ACCESS forever, busy stays high, mem_msg_router
// stalls, cdc_msg_sync stalls, UART_CTS never releases, and the host waits on
// a link that will never move again. One bad address bricks the design, and
// the symptom looks like a dead UART rather than an address bug.
//
// So an unmatched access is completed HERE, in one cycle, with PSLVERR set.
// The transfer finishes, the master returns to idle, the error is reported,
// and the link keeps running.
//
// This also closes a gap rgf_pkg records as deferred: a write to an unmapped
// register used to vanish and a read used to return zero, with nothing to
// distinguish either from success.
//
// WHAT DOES NOT REACH HERE
// rx_classifier already rejects a register address that does not fit the
// 8-bit port or is not word aligned, so a malformed HOST address is a
// classifier error and never becomes a transfer. The default responder is for
// addresses that are structurally legal but unpopulated -- aperture 1 today.

module apb_bar
    import apb_pkg::*;
(
    // ---- from apb_master ---------------------------------------------
    input  logic                  m_active,
    input  logic                  m_penable,
    input  logic [ADDR_W-1:0]     m_paddr,

    // ---- to apb_master -----------------------------------------------
    output logic                  m_pready,
    output logic [DATA_W-1:0]     m_prdata,
    output logic                  m_pslverr,

    // ---- to the slaves -------------------------------------------------
    output logic [NUM_SLAVES-1:0] psel,

    // ---- from the slaves -----------------------------------------------
    input  logic [NUM_SLAVES-1:0] s_pready,
    input  logic [NUM_SLAVES-1:0] s_pslverr,
    input  logic [DATA_W-1:0]     s_prdata [NUM_SLAVES],

    // ---- diagnostics ----------------------------------------------------
    // One cycle, on completion of an access that matched no aperture.
    // chip_top latches this into a sticky bit for an LED.
    output logic                  decode_err
);

    logic [SEL_W-1:0] sel;
    assign sel = addr_sel(m_paddr);

    // -----------------------------------------------------------------
    // Decode. One equality compare per aperture, generated from the table in
    // apb_pkg -- adding a slave there adds a psel bit here with no edit.
    // -----------------------------------------------------------------
    logic [NUM_SLAVES-1:0] hit;

    for (genvar i = 0; i < NUM_SLAVES; i++) begin : g_decode
        assign hit[i]  = (sel == SLAVE_BASE[i]);
        assign psel[i] = m_active && hit[i];
    end : g_decode

    logic no_match;
    assign no_match = m_active && !(|hit);

    // -----------------------------------------------------------------
    // Response mux
    //
    // Unselected slaves are IGNORED rather than trusted to drive zero, so a
    // slave holding PREADY high while idle cannot complete somebody else's
    // transfer.
    // -----------------------------------------------------------------
    logic              sel_pready;
    logic              sel_pslverr;
    logic [DATA_W-1:0] sel_prdata;

    always_comb begin : rsp_mux
        sel_pready  = 1'b0;
        sel_pslverr = 1'b0;
        sel_prdata  = '0;
        for (int i = 0; i < NUM_SLAVES; i++) begin
            if (psel[i]) begin
                sel_pready  = s_pready[i];
                sel_pslverr = s_pslverr[i];
                sel_prdata  = s_prdata[i];
            end
        end
    end : rsp_mux

    // The default responder completes an unmatched access immediately. PREADY
    // is asserted only in the ACCESS phase: asserting it during SETUP would
    // violate the protocol and mislead a waveform reader, even though
    // apb_master does not sample it there.
    assign m_pready  = no_match ? m_penable : sel_pready;
    assign m_pslverr = no_match ? m_penable : sel_pslverr;
    assign m_prdata  = no_match ? '0        : sel_prdata;

    assign decode_err = no_match && m_penable;

`ifndef SYNTHESIS
    // At most one aperture may claim an address. Overlapping SLAVE_BASE
    // entries would otherwise produce a silently merged response. Written as
    // an immediate assertion because this module has no clock to sample on.
    always_comb begin : a_onehot
        if (m_active)
            assert ($onehot0(psel))
            else $error("%m: more than one slave selected -- overlapping apertures");
    end : a_onehot
`endif

endmodule : apb_bar
