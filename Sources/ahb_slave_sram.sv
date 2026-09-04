`timescale 1ns/1ps
// ahb_slave_sram.sv
// =================
// AHB-Lite subordinate front end for one rgb_sram. One instance per colour
// channel; rgb_sram itself is unchanged.
//
// TWO-PHASE, AND WHY THE ADDRESS MUST BE REGISTERED
//
//   cycle N    address phase   HSEL, HTRANS, HADDR, HWRITE valid
//   cycle N+1  data phase      HWDATA valid (write) / HRDATA expected (read)
//
// HWDATA arrives a cycle AFTER the address it belongs to. So a write cannot
// be issued from the address phase -- the data is not there yet. The address
// and direction are captured in cycle N and the SRAM write is driven in
// cycle N+1 alongside HWDATA.
//
// Reads go the other way. rgb_sram registers rd_data, so rd_en asserted in
// the address phase makes the data appear exactly when the data phase needs
// it. Zero wait states, no extra pipeline stage.
//
// HREADY IS AN INPUT, HREADYOUT IS THE OUTPUT
//
// hreadyout is this slave's own "I am done". hready is the AGGREGATED ready
// of the bus segment, fed back in by the decoder. The address phase is only
// valid when hready is high -- otherwise a previous transfer is still
// stretching and this address must not be captured yet. Wiring a slave's own
// hreadyout back into its hready appears to work with one slave and fails
// silently the moment a second is added.
//
// WHAT THIS SLAVE DOES NOT DO
//
// Partial-word writes. AHB-Lite has no byte-strobe signal (HWSTRB is AHB5),
// so a sub-word write can only be expressed through HSIZE plus address
// alignment, which cannot produce an arbitrary lane mask. Single-pixel writes
// therefore stay on rgb_sram's direct write port, driven by
// pixel_word_packer -- the "Direct Connect Single Write" path in the spec.
// This bus carries whole 32-bit words: burst image writes and all reads.

module ahb_slave_sram
    import ahb_pkg::*;
#(
    parameter int SRAM_ADDR_W = 14
)(
    input  logic                    hclk,
    input  logic                    hresetn,

    // ---- AHB-Lite subordinate interface --------------------------------
    input  logic                    hsel,
    input  logic                    hready,     // segment ready, from the decoder
    input  logic [ADDR_W-1:0]       haddr,
    input  logic                    hwrite,
    input  logic [2:0]              hsize,
    // Present because the AHB-Lite subordinate interface defines it, but a
    // subordinate does not act on it: burst type matters to the manager and
    // to an arbiter deciding when a burst may be interrupted, not to the
    // target. Each beat is just an address here.
    /* verilator lint_off UNUSEDSIGNAL */
    input  logic [2:0]              hburst,
    /* verilator lint_on UNUSEDSIGNAL */
    input  logic [1:0]              htrans,
    input  logic [DATA_W-1:0]       hwdata,

    output logic                    hreadyout,
    output logic [DATA_W-1:0]       hrdata,
    output logic                    hresp,

    // ---- rgb_sram ports -------------------------------------------------
    output logic                    sram_rd_en,
    output logic [SRAM_ADDR_W-1:0]  sram_rd_addr,
    input  logic [DATA_W-1:0]       sram_rd_data,
    output logic                    sram_wr_en,
    output logic [DATA_W/8-1:0]     sram_wr_be,
    output logic [SRAM_ADDR_W-1:0]  sram_wr_addr,
    output logic [DATA_W-1:0]       sram_wr_data
);

    // -----------------------------------------------------------------
    // Address phase
    //
    // A beat is live when this slave is selected, the segment is ready, and
    // HTRANS is a real transfer. IDLE and BUSY are not: BUSY in particular
    // means the manager is stalling INSIDE a burst, so the burst is still in
    // progress but no data moves this beat.
    // -----------------------------------------------------------------
    logic addr_phase;
    assign addr_phase = hsel && hready &&
                        ((htrans == HTRANS_NONSEQ) || (htrans == HTRANS_SEQ));

    // -----------------------------------------------------------------
    // Read: driven straight from the address phase
    // -----------------------------------------------------------------
    assign sram_rd_en   = addr_phase && !hwrite;
    assign sram_rd_addr = SRAM_ADDR_W'(addr_word(haddr));

    // rgb_sram holds rd_data until the next rd_en, so this is stable for the
    // whole data phase without a capture register here.
    assign hrdata = sram_rd_data;

    // -----------------------------------------------------------------
    // Write: address captured now, SRAM driven next cycle with HWDATA
    // -----------------------------------------------------------------
    logic                   wr_pending;
    logic [SRAM_ADDR_W-1:0] wr_addr_q;

    always_ff @(posedge hclk or negedge hresetn) begin
        if (!hresetn) begin
            wr_pending <= 1'b0;
            wr_addr_q  <= '0;
        end
        else begin
            wr_pending <= addr_phase && hwrite;
            if (addr_phase && hwrite)
                wr_addr_q <= SRAM_ADDR_W'(addr_word(haddr));
        end
    end

    assign sram_wr_en   = wr_pending;
    assign sram_wr_addr = wr_addr_q;
    assign sram_wr_data = hwdata;

    // All four lanes. See the note in the header: this bus is word-only, and
    // partial-word writes stay on the direct port.
    assign sram_wr_be   = {(DATA_W/8){1'b1}};

    // -----------------------------------------------------------------
    // Response
    //
    // Zero wait states: the SRAM is always ready. HREADYOUT must be high when
    // idle as well as when completing, so it is simply tied high.
    //
    // This slave cannot fail an access that reached it -- every address in
    // its aperture is a real word. Unmapped addresses are the decoder's
    // default slave, not this module's business.
    // -----------------------------------------------------------------
    assign hreadyout = 1'b1;
    assign hresp     = HRESP_OKAY;

`ifndef SYNTHESIS
    initial begin
        if (SRAM_ADDR_W > WORD_W)
            $error("%m: SRAM_ADDR_W (%0d) exceeds the address field WORD_W (%0d)",
                   SRAM_ADDR_W, WORD_W);
    end

    // This bus only ever issues word transfers. A byte or halfword would be
    // silently widened to a full word by sram_wr_be above, corrupting three
    // neighbouring pixels.
    a_word_only: assert property (
        @(posedge hclk) disable iff (!hresetn)
        addr_phase |-> (hsize == HSIZE_WORD)
    ) else $error("%m: non-word HSIZE %0d -- this slave cannot express sub-word writes",
                  hsize);

    // Word transfers must be word aligned.
    a_aligned: assert property (
        @(posedge hclk) disable iff (!hresetn)
        addr_phase |-> (haddr[1:0] == 2'b00)
    ) else $error("%m: unaligned address 0x%08h", haddr);

    // Read and write to the same SRAM address in the same cycle trips
    // rgb_sram's own collision assertion. Catch it here, where the AHB
    // context is still visible.
    a_no_collision: assert property (
        @(posedge hclk) disable iff (!hresetn)
        !(sram_rd_en && sram_wr_en && (sram_rd_addr == sram_wr_addr))
    ) else $error("%m: read/write collision at word %0d", sram_rd_addr);

    // Guards against the HREADY/HREADYOUT mistake: if hready were wired from
    // this slave's own hreadyout, addr_phase would ignore a stretched
    // transfer elsewhere on the segment.
    a_hready_when_selected: assert property (
        @(posedge hclk) disable iff (!hresetn)
        (hsel && (htrans == HTRANS_SEQ)) |-> $past(hsel)
    ) else $error("%m: SEQ beat without a preceding selected beat -- burst broken");
`endif

endmodule : ahb_slave_sram
