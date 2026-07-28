// -----------------------------------------------------------------------------
// chip_top.sv
// Top-level wrapper. Lab 9 (ROM-to-PC image transfer over UART, RX+RGF
// wired in) is complete. Lab 10 (PLL + glitchless clock mux, UART parity)
// is in progress -- see the PLL/clock mux status block below for exactly
// what's done vs. still open in that effort.
//
// RGF status: instantiated and wired to a real dispatcher (see "Config RGF
// + dispatcher" section below). row_q selects a register by index,
// col_q's parity selects read/write, pixel_q is write data. IMG_CTRL's
// start bit (via the interlock) is what actually triggers an image send
// now -- NOT "any successfully parsed RX command" like the pre-RGF
// version of this file. See RGF_SIGNAL_REFERENCE.md for the full
// register map and HANDOFF.md for what's still open (read replies,
// FIFO_STATUS wiring, RGF robustness pass).
// -----------------------------------------------------------------------------

import uart_pkg::*;
import rgf_pkg::*;

module chip_top (
    input  logic        CLK100MHZ,      // 100 MHz clock
    input  logic        CPU_RESETN,     // active-low, async, from push button

    // UART interface
    input  logic        UART_TXD_IN,    // RX serial line from PC (board naming: bridge TX -> FPGA in)
    input  logic        UART_RTS,       // RTS flow control (active-low). Per Nexys manual: signal
                                         // names are from the PC's (DTE) point of view, so "RTS" is
                                         // the PC's own RTS output -- an INPUT here at the FPGA.
    output logic        UART_RXD_OUT,   // TX serial line to PC
    output logic        UART_CTS,       // CTS flow control (active-low). Same convention: "CTS" is
                                         // the PC's own CTS input, which the FPGA must drive -- an
                                         // OUTPUT here. (Confirmed empirically on hardware: an
                                         // earlier version had these two directions swapped, which
                                         // is why CTS-gated TX never left IDLE until this was fixed.)

    // LEDs
    output logic [15:0] LED             // LED[0]: TX toggle indicator
);

// -----------------------------------------------------------------------------
// Reset synchronization
// -----------------------------------------------------------------------------
logic sync_rst_n;

reset_synch u_reset_synch (
    .clk        (CLK100MHZ),
    .async_rst_n(CPU_RESETN),
    .sync_rst_n (sync_rst_n)
);

// -----------------------------------------------------------------------------
// PLL / clock mux (Lab 10)
// -----------------------------------------------------------------------------
// Per the spec's literal wording: "a counter system that can operate at
// two different clock speeds... selected through a control register and
// a glitchless clock multiplexer." One mux, register-selected, driving a
// counter -- that's the whole deliverable. (An earlier version of this
// file also had a second, auto-switching mux for an "operational system
// clock" domain -- that was scope I added, reasoning from another team's
// example, not something the assignment actually asks for. Removed.)
//
// STATUS:
//   [DONE] PLL:    clk_wiz_0 generated and instantiated below (real IP)
//                   -- 100 MHz in -> 130.000 MHz out (exact, VCO 1007.5
//                   MHz via MULT_F=50.375/DIVCLK_DIVIDE=5, clk_out1
//                   divide=7.750). resetn is active-low per the wizard
//                   config, wired directly to sync_rst_n.
//   [DONE] Mux +   glitchless_clk_mux, clk0=CLK100MHZ fallback,
//          counter: clk1=pll_clk_out, sel=clk_sel driven by
//                   rgf.clk_sel_out (CLK_CTRL register, address 0x10,
//                   bit[0]) -- {R004,C000,V001} selects the PLL clock,
//                   {R004,C000,V000} selects CLK100MHZ. The heartbeat
//                   counter below runs off the mux output, giving a
//                   visible LED confirmation of the two-speed switching.
//   [OPEN] UART on PLL clock: the spec separately asks to "adjust the
//                   PLL/MMCM output accordingly with respect to the
//                   baudx16" -- i.e. wire tx_phy/rx_phy directly onto
//                   pll_clk_out, permanently, no switching, no register.
//                   This is UNRELATED to the mux above (no mux involved
//                   in this piece at all) and not done in this pass --
//                   uart_pkg.sv's DIV_TX/DIV_RX still assume
//                   CLK100MHZ/1Mbps. Also needs its own reset
//                   synchronizer once it happens (a block on a new clock
//                   domain needs its own synchronized reset, same
//                   reasoning as always).
// -----------------------------------------------------------------------------
logic pll_clk_out;
logic pll_locked;

clk_wiz_0 u_clk_wiz_0 (
    .clk_out1 (pll_clk_out),
    .resetn   (sync_rst_n),
    .locked   (pll_locked),
    .clk_in1  (CLK100MHZ)
);

// Clock mux: PC-controlled via RGF CLK_CTRL, drives the counter below.
//
// SYNTHESIS uses the BUFGCTRL hard primitive instead of the RTL
// glitchless_clk_mux module. This isn't stylistic -- a signal that drives
// OTHER flip-flops' clock inputs must be routed through the FPGA's
// dedicated global clock network, which plain fabric AND/OR gates (what
// glitchless_clk_mux's clk_out = clk0_gated | clk1_gated actually
// synthesizes to) cannot reach. Left as RTL-only, this either fails
// Vivado's implementation DRC outright or gets placed on ordinary fabric
// routing with unpredictable skew -- something that would never show up
// in simulation (which doesn't model physical clock-tree routing at all)
// but is a real risk now that this clock drives a real flip-flop (the
// heartbeat counter below). IGNORE0/IGNORE1=0 uses BUFGCTRL's own
// built-in synchronizer for the switch itself, equivalent to
// glitchless_clk_mux's 2-FF chains.
//
// clk_sel comes from a PC-writable register that could be set to 1
// before pll_locked ever asserts -- glitchless_clk_mux's own sel_qual
// (= sel & pll_lock) handles that internally in the RTL branch, but
// BUFGCTRL has no equivalent lock-gating concept, so sel_qual below
// replicates it explicitly before feeding S0/S1.
logic clk_sel;   // driven by rgf.clk_sel_out below
logic sys_clk;
logic sel_qual;
assign sel_qual = clk_sel & pll_locked;

`ifdef SYNTHESIS
    BUFGCTRL u_glitchless_clk_mux (
        .O       (sys_clk),
        .I0      (CLK100MHZ),
        .I1      (pll_clk_out),
        .S0      (~sel_qual),
        .S1      (sel_qual),
        .CE0     (1'b1),
        .CE1     (1'b1),
        .IGNORE0 (1'b0),
        .IGNORE1 (1'b0)
    );
`else
    glitchless_clk_mux u_glitchless_clk_mux (
        .clk0     (CLK100MHZ),
        .clk1     (pll_clk_out),
        .sel      (clk_sel),
        .pll_lock (pll_locked),
        .clk_out  (sys_clk)
    );
`endif

// Heartbeat: a raw ~100-130 MHz toggle is invisible to the eye on an LED,
// so divide sys_clk down to a human-visible blink rate instead.
// Free-running, no reset -- this is the visible half of "a counter
// system that can operate at two different clock speeds."
// Blink period scales directly with whichever clock is currently
// selected -- ~1.3x faster blinking when clk_sel=1 (130MHz) vs
// clk_sel=0 (100MHz) is a real, physical confirmation of the two-speed
// switching, not just the register bit changing.
logic [26:0] counter_heartbeat_cnt;
always_ff @(posedge sys_clk) begin
    counter_heartbeat_cnt <= counter_heartbeat_cnt + 1'b1;
end

// -----------------------------------------------------------------------------
// Reset synchronization for PLL clk
// -----------------------------------------------------------------------------
logic sync_pll_rst_n;

reset_synch u_pll_reset_synch (
    .clk        (pll_clk_out),   // was: sys_clk
    .async_rst_n(CPU_RESETN),
    .sync_rst_n (sync_pll_rst_n)
);

// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
// Start pulse: now driven by rgf.start_img_read_out (see RGF + dispatcher
// section below, placed after the RX chain since it depends on
// rx_classifier's outputs). Declared here since rom_sequencer below
// references it -- SystemVerilog module-level nets are visible throughout
// the module regardless of textual declare/use order, but the actual
// driving assign lives with the logic that computes it, for readability.
// -----------------------------------------------------------------------------
logic start_pulse;

// -----------------------------------------------------------------------------
// STAGE 2C: shared SRAM write bus
// -----------------------------------------------------------------------------
// One address, one byte enable and one write enable drive all three channel
// SRAMs on the same 100 MHz cycle; only the data differs per channel. Declared
// here so the instantiations below can reference them.
// -----------------------------------------------------------------------------
logic                                    sram_wr_en;
logic [memory_pkg::SRAM_DATA_WIDTH/8-1:0] sram_wr_be;
logic [memory_pkg::SRAM_ADDR_WIDTH-1:0]   sram_wr_addr;
logic [memory_pkg::SRAM_DATA_WIDTH-1:0]   sram_wr_data_r;
logic [memory_pkg::SRAM_DATA_WIDTH-1:0]   sram_wr_data_g;
logic [memory_pkg::SRAM_DATA_WIDTH-1:0]   sram_wr_data_b;

// Arbitration between the image-read path and the write path.
logic sram_wr_allowed;
logic sram_wr_busy;
logic read_go;

// tx_img_done recovered onto CLK100MHZ by the Change B toggle synchroniser.
// Declared here rather than beside its cdc_pulse_sync instance because
// mem_interlock (below) consumes it, and that instantiation comes first.
logic tx_img_done_100;

// -----------------------------------------------------------------------------
// RGB BRAM SRAMs (read/write as of Stage 2C)
// -----------------------------------------------------------------------------
// Stage 1 replaced the three `rom` instances with rgb_sram. The read
// interface is bit- and cycle-identical to the ROM's -- same 1-cycle
// registered latency, same hold-last-value behaviour when the enable is
// low, same widths, depth and contents -- so rom_sequencer.sv below is
// UNCHANGED and sees no difference at all.
//
// STAGE 2C: the write ports are now LIVE. All three instances share one
// write enable, address and byte enable driven by sram_wr_ctrl; only the
// data bus differs per channel. mem_interlock keeps the write port and
// rom_sequencer's read port mutually exclusive, so wr_en and rom_rd_en are
// never asserted in the same cycle -- which is what makes the same-address
// collision assertion inside rgb_sram unreachable.
//
// Geometry comes from memory_pkg (SRAM_DATA_WIDTH / SRAM_DEPTH). The
// per-channel initialisation filenames are deliberately NOT in that
// package -- they are per-instance configuration, passed in below.
//
// The rom_addr / rom_rd_en / *_data signal names below are unchanged on
// purpose: they connect straight to rom_sequencer.sv, which this stage
// does not touch. The names are now slightly inaccurate; that is
// carried as documented debt rather than widening the Stage 1 diff.
// -----------------------------------------------------------------------------
logic [13:0] rom_addr;      // 14-bit address for 16384 words (= SRAM_ADDR_WIDTH)
logic        rom_rd_en;
logic [31:0] red_data;
logic [31:0] green_data;
logic [31:0] blue_data;

// -----------------------------------------------------------------------------
// SRAM READ-PORT MUX
// -----------------------------------------------------------------------------
// The read port now has TWO clients:
//
//   rom_sequencer   full-image readback, drives rom_rd_en / rom_addr
//   pixel_rd_ctrl   Single Pixel Read, drives pix_rd_en / pix_rd_addr
//
// Selection is NOT made here on any local condition. It follows
// mem_interlock's pix_rd_owner, which is the same registered pix_rd_active
// that gates read_go and wr_allowed inside the arbiter. Deriving the mux
// select from anything else -- pixel_rd_ctrl's own state, say -- would let
// the datapath and the arbiter disagree about who owns the port, which is
// precisely the class of bug the interlock exists to make impossible.
//
// Because pix_rd_owner blocks read_go, rom_rd_en is guaranteed inactive for
// the whole window in which the mux points at the pixel reader, so nothing
// is lost by switching the address bus wholesale.
// -----------------------------------------------------------------------------
logic        pix_sram_rd_en;
logic [13:0] pix_sram_rd_addr;
logic        pix_rd_owner;      // from mem_interlock

logic        sram_rd_en_mux;
logic [13:0] sram_rd_addr_mux;

// -----------------------------------------------------------------------------
// SINGLE PIXEL READ -- ALL signal declarations, gathered here on purpose
// -----------------------------------------------------------------------------
// These are declared HERE, ahead of every instantiation that touches them,
// rather than beside the blocks that drive them.
//
// That is not a style preference. u_tx_reply_ctrl is instantiated around line
// 500, roughly 600 lines above the pixel-read datapath, and connects
// pix_rpy_valid_130, pix_reply_msg and pix_rpy_accept_130. When those were
// declared next to their drivers -- i.e. AFTER that instantiation -- the port
// connections referenced identifiers that did not yet exist, and Verilog's
// implicit-net rule silently created ONE-BIT WIRES for them. A 128-bit reply
// frame connected through a 1-bit implicit net delivers bit 0 and zeroes the
// other 127, which is exactly how a perfectly framed reply ends up carrying an
// all-zero payload.
//
// Nothing warns about this: implicit nets are legal Verilog, and the later
// explicit declaration is a separate object. Declaring every cross-block
// signal in one place, above all users, makes the failure unreachable.
// -----------------------------------------------------------------------------

// ---- 130 MHz: request out of rx_classifier ----------------------------------
logic        rx_pr_cmd_valid;
logic [9:0]  rx_pr_cmd_row, rx_pr_cmd_col;

// ---- 100 MHz: request delivered to pixel_rd_ctrl ----------------------------
logic        pix_req_valid_100;
logic [19:0] pix_req_data_100;      // {row[9:0], col[9:0]}, atomic

// ---- 100 MHz: memory arbitration --------------------------------------------
logic        pix_rd_req, pix_rd_gnt, pix_rd_done;

// ---- 100 MHz: reply payload, held by pixel_rd_ctrl until acknowledged -------
logic        pix_rpy_valid_100, pix_rpy_accept_100;
logic [9:0]  pix_rpy_row, pix_rpy_col;
logic [23:0] pix_rpy_pixel;
logic        pix_rd_busy, pix_rd_overrun;

// ---- 100 MHz: one-cycle send event, derived from the held valid -------------
logic        pix_rpy_valid_100_d;
logic        pix_rpy_send;

// ---- 130 MHz: reply delivered atomically by cdc_cmd_sync --------------------
logic        pix_rpy_valid_130;     // one-cycle strobe
logic [43:0] pix_rpy_data_130;      // {row, col, pixel}, coherent
logic        pix_rpy_held;          // destination-side pending flag
logic [43:0] pix_rpy_payload_130;   // captured, stable for the composer
logic        pix_rpy_accept_130;
logic [127:0] pix_reply_msg;

assign sram_rd_en_mux   = pix_rd_owner ? pix_sram_rd_en   : rom_rd_en;
assign sram_rd_addr_mux = pix_rd_owner ? pix_sram_rd_addr : rom_addr;

`ifndef SYNTHESIS
    // The two read clients can never both drive the port.
    a_read_client_exclusive: assert property (
        @(posedge CLK100MHZ) disable iff (!sync_rst_n)
        !(rom_rd_en && pix_sram_rd_en)
    ) else $error("chip_top: image and pixel readers drove the SRAM together");

    // A pixel read only ever reaches the memory while it owns it.
    a_pix_rd_owns: assert property (
        @(posedge CLK100MHZ) disable iff (!sync_rst_n)
        pix_sram_rd_en |-> pix_rd_owner
    ) else $error("chip_top: pixel read drove the SRAM without ownership");
`endif

// Red channel SRAM
rgb_sram #(
    .DATA_WIDTH (memory_pkg::SRAM_DATA_WIDTH),
    .DEPTH      (memory_pkg::SRAM_DEPTH),
    .INIT_FILE  ("red_hex.mem")
) u_sram_red (
    .clk     (CLK100MHZ),
    .rd_en   (sram_rd_en_mux),
    .rd_addr (sram_rd_addr_mux),
    .rd_data (red_data),
    .wr_en   (sram_wr_en),
    .wr_be   (sram_wr_be),
    .wr_addr (sram_wr_addr),
    .wr_data (sram_wr_data_r)
);

// Green channel SRAM
rgb_sram #(
    .DATA_WIDTH (memory_pkg::SRAM_DATA_WIDTH),
    .DEPTH      (memory_pkg::SRAM_DEPTH),
    .INIT_FILE  ("green_hex.mem")
) u_sram_green (
    .clk     (CLK100MHZ),
    .rd_en   (sram_rd_en_mux),
    .rd_addr (sram_rd_addr_mux),
    .rd_data (green_data),
    .wr_en   (sram_wr_en),
    .wr_be   (sram_wr_be),
    .wr_addr (sram_wr_addr),
    .wr_data (sram_wr_data_g)
);

// Blue channel SRAM
rgb_sram #(
    .DATA_WIDTH (memory_pkg::SRAM_DATA_WIDTH),
    .DEPTH      (memory_pkg::SRAM_DEPTH),
    .INIT_FILE  ("blue_hex.mem")
) u_sram_blue (
    .clk     (CLK100MHZ),
    .rd_en   (sram_rd_en_mux),
    .rd_addr (sram_rd_addr_mux),
    .rd_data (blue_data),
    .wr_en   (sram_wr_en),
    .wr_be   (sram_wr_be),
    .wr_addr (sram_wr_addr),
    .wr_data (sram_wr_data_b)
);

// -----------------------------------------------------------------------------
// Inter-block signals (declared once; shared across rom_sequencer, async_fifo,
// and tx_sequencer instantiations below)
// -----------------------------------------------------------------------------
logic        fifo_wr_en;        // rom_sequencer → async_fifo write enable
logic [23:0] fifo_wr_data;      // rom_sequencer → async_fifo pixel data
logic        almost_full;       // async_fifo → rom_sequencer backpressure
logic        almost_empty;      // async_fifo → rom_sequencer resume signal
logic        fifo_rd_en;        // tx_sequencer → async_fifo read enable
logic [23:0] fifo_rd_data;      // async_fifo → tx_sequencer pixel data (24-bit RGB)
logic        fifo_empty;        // async_fifo → tx_sequencer empty flag
logic        fifo_full;         // async_fifo full flag (monitored, not used for control)
logic        seq_done;          // rom_sequencer → chip_top: one-cycle done pulse
logic        rom_seq_busy;      // rom_sequencer → LEDs

// -----------------------------------------------------------------------------
// IMAGE FIFO STATUS FLAGS: 130 MHz -> 100 MHz
// -----------------------------------------------------------------------------
// The two occupancy flags are NOT generated in the same clock domain, and the
// asymmetry is easy to miss because they are declared side by side above:
//
//   almost_full   registered on wr_clk  (CLK100MHZ)  -- same domain as its
//                                                       consumer, safe as-is
//   almost_empty  registered on rd_clk  (pll_clk_out) -- a DIFFERENT domain
//                                                       from its consumer
//
// almost_empty was wired straight from the 130 MHz read domain into
// rom_sequencer, which runs on CLK100MHZ, with no synchroniser. It is the only
// control signal in the full-image read path that crosses domains unprotected.
//
// WHY THIS CORRUPTS PIXEL DATA RATHER THAN JUST DELAYING A RESUME
// --------------------------------------------------------------
// rom_sequencer consumes it as a next-state condition:
//
//     WAIT_DRAIN: if (almost_empty) next_state = READ_ROM;
//
// so an asynchronous signal feeds combinational logic whose result is captured
// by current_state. A setup/hold violation there does not merely delay the
// transition by a cycle -- which would be harmless -- it can leave the state
// register resolving inconsistently across its bits, so current_state can land
// on a value that is not a legal successor of WAIT_DRAIN.
//
// Landing on LATCH is the damaging case. LATCH re-captures pixels[] from
// red/green/blue_data without a preceding READ_ROM, so it latches the PREVIOUS
// word still standing on the SRAM outputs. The four pixels of that word are
// then pushed again, and NEXT_ADDR still advances addr_counter, so the FIFO
// receives the correct NUMBER of pixels and the frame stays aligned -- four
// values are simply wrong. That is exactly the observed signature: scattered
// wrong pixels, correct coordinates, the rest of the frame bit-exact.
//
// WHY IT IS RARE BUT NOT RARE ENOUGH
// ----------------------------------
// With AF_THRESHOLD = 52 and AE_THRESHOLD = 8 the sequencer drains 44 pixels
// per cycle of backpressure, so a 65,536-pixel frame contains roughly
// 65536/44 = ~1,490 WAIT_DRAIN exits. Each one samples almost_empty
// asynchronously. Nearly all resolve cleanly; a handful per frame do not.
// A few corrupted pixels per frame is the expected order of magnitude, and it
// moves with temperature, placement and reset phase -- which is why repeated
// readbacks disagree and why re-running implementation changes the pattern.
//
// THE FIX
// -------
// A two-flop level synchroniser, using the same cdc_level_sync already carrying
// burst_active across the same boundary. async_fifo is NOT touched: its
// interface, timing, pointer arithmetic and flag generation are all unchanged,
// and the previously rejected read-gating experiment is not revived.
//
// Cost to throughput: rom_sequencer observes almost_empty two to three
// CLK100MHZ cycles later, so it resumes ~20-30 ns later than before. The
// consumer drains one pixel per ~2 us, so occupancy at the resume point is
// unchanged for all practical purposes.
//
// fifo_empty is synchronised alongside it. That one only feeds rgf status bits,
// so it cannot corrupt image data, but it is the same 130 -> 100 crossing and
// costs nothing to close while the synchronisers are being added.
// -----------------------------------------------------------------------------
logic almost_empty_100;
logic fifo_empty_100;

cdc_level_sync u_cdc_almost_empty (
    .src_level (almost_empty),      // pll_clk_out domain
    .dst_clk   (CLK100MHZ),
    .dst_rst_n (sync_rst_n),
    .dst_level (almost_empty_100)
);

cdc_level_sync u_cdc_fifo_empty (
    .src_level (fifo_empty),        // pll_clk_out domain
    .dst_clk   (CLK100MHZ),
    .dst_rst_n (sync_rst_n),
    .dst_level (fifo_empty_100)
);

// -----------------------------------------------------------------------------
// ROM Sequencer
// -----------------------------------------------------------------------------
rom_sequencer u_rom_sequencer (
    .clk         (CLK100MHZ),
    .rst_n       (sync_rst_n),
    .start       (read_go),        // Stage 2C: gated by mem_interlock
    .almost_full (almost_full),    // already CLK100MHZ, unchanged
    .almost_empty(almost_empty_100),
    .red_data    (red_data),
    .green_data  (green_data),
    .blue_data   (blue_data),
    .rom_addr    (rom_addr),
    .rom_rd_en   (rom_rd_en),
    .wr_en       (fifo_wr_en),
    .wr_data     (fifo_wr_data),
    .seq_done    (seq_done),
    .busy        (rom_seq_busy)
);

// -----------------------------------------------------------------------------
// Asynchronous FIFO
// -----------------------------------------------------------------------------
async_fifo u_async_fifo (
    // Write domain (rom_sequencer side)
    .wr_clk      (CLK100MHZ),
    .wr_rst_n    (sync_rst_n),
    .wr_en       (fifo_wr_en),
    .wr_data     (fifo_wr_data),
    .full        (fifo_full),
    .almost_full (almost_full),
    // Read domain (tx_sequencer side)
    .rd_clk      (pll_clk_out),   // was: sys_clk
    .rd_rst_n    (sync_pll_rst_n),
    .rd_en       (fifo_rd_en),
    .rd_data     (fifo_rd_data),
    .empty       (fifo_empty),
    .almost_empty(almost_empty)
);

// -----------------------------------------------------------------------------
// TEMPORARY DIAGNOSTIC -- image FIFO overflow detector. REMOVE AFTER ANALYSIS.
// -----------------------------------------------------------------------------
// async_fifo drops a write silently when full:
//
//     if (wr_en && !full) fifo_mem[...] <= wr_data;   // async_fifo.sv
//
// so a lost pixel leaves no trace anywhere. This flag is the only way to
// observe it.
//
// It answers one question: did rom_sequencer ever attempt a push that the
// FIFO refused? If it lights, pixels were dropped, tx_sequencer never
// received its 65536th pixel, never reached DONE, and tx_img_done never
// fired -- which is exactly the stall the clear-then-start test implied.
//
// Purely observational: it reads fifo_wr_en and fifo_full and drives nothing
// back into the datapath, so it cannot perturb FIFO behaviour. Both signals
// are native to the write domain (CLK100MHZ / sync_rst_n), so there is no
// clock crossing here.
// -----------------------------------------------------------------------------
logic img_fifo_ovf_sticky;

always_ff @(posedge CLK100MHZ or negedge sync_rst_n) begin
    if (!sync_rst_n)                 img_fifo_ovf_sticky <= 1'b0;
    else if (fifo_wr_en && fifo_full) img_fifo_ovf_sticky <= 1'b1;
end
// -----------------------------------------------------------------------------
// TX Sequencer (includes msg_composer internally)
// -----------------------------------------------------------------------------
logic        msg_valid;
logic        mac_busy;
logic        tx_img_done;
logic        tx_seq_busy;
logic [9:0]  tx_row;
logic [9:0]  tx_col;
logic [127:0] msg_data;

tx_sequencer u_tx_sequencer (
    .clk         (pll_clk_out),
    .rst_n       (sync_pll_rst_n),
    .fifo_empty  (fifo_empty),
    .cts         (UART_RTS),
    .mac_busy    (mac_busy),
    .fifo_rd_data(fifo_rd_data),
    .msg_valid   (msg_valid),
    .fifo_pop    (fifo_rd_en),
    .tx_img_done (tx_img_done),
    .busy        (tx_seq_busy),
    .msg         (msg_data),
    .row_cnt_out (tx_row),
    .col_cnt_out (tx_col)
);

// -----------------------------------------------------------------------------
// TEMPORARY DIAGNOSTIC -- did tx_sequencer reach DONE? REMOVE AFTER ANALYSIS.
// -----------------------------------------------------------------------------
// tx_img_done is a one-cycle pulse registered off tx_sequencer's DONE state.
// Reaching DONE requires NEXT to have been entered 65536 times, and NEXT is
// reachable only from WAIT_DONE on !mac_busy -- i.e. after a full mac_busy
// rise-and-fall for that message. mac_busy in turn only falls once the MAC has
// reached MAC_DONE, which requires byte_idx == 15 and phy_ready, and phy_ready
// only asserts after the PHY has returned to PHY_IDLE from PHY_STOP_BIT on
// baud_tick.
//
// So one counter increment corresponds to one COMPLETELY TRANSMITTED 16-byte
// message, stop bit included. The counters cannot run ahead of the wire, and
// this flag therefore answers the outstanding question directly:
//
//   LIT  after a short capture -> all 65536 messages physically left
//                                 UART_RXD_OUT; the loss is downstream of the
//                                 FPGA pin.
//   DARK after a short capture -> tx_sequencer never reached DONE; the stall
//                                 is on the FPGA side and the rest of the LED
//                                 map localises it.
//
// Unlike the protocol-level retry tests, this cannot be confounded by cts, by
// img_in_flight, or by anything on the host.
//
// Clocked on pll_clk_out / sync_pll_rst_n to match tx_sequencer's domain --
// tx_img_done is sampled where it is generated, with no crossing.
// -----------------------------------------------------------------------------
logic tx_done_sticky;

always_ff @(posedge pll_clk_out or negedge sync_pll_rst_n) begin
    if (!sync_pll_rst_n)  tx_done_sticky <= 1'b0;
    else if (tx_img_done) tx_done_sticky <= 1'b1;
end

// -----------------------------------------------------------------------------
// TX MAC
// -----------------------------------------------------------------------------
logic        phy_valid;
logic [7:0]  phy_data;
logic        phy_ready;

// -----------------------------------------------------------------------------
// REGISTER READ: reply scheduling and TX mux
// -----------------------------------------------------------------------------
// The reply waits until the transmit path is completely idle. An image
// transfer is 65536 fixed-size packets that the host reads as one stream
// counting bytes; a 6-byte reply inserted between two pixel messages would
// desynchronise everything after it.
//
// tx_reply_ctrl only asserts reply_req when !tx_seq_busy && !mac_busy, and
// tx_sequencer only asserts msg_valid from a non-IDLE state, so the two can
// never drive the MAC in the same cycle.
// -----------------------------------------------------------------------------
logic         rr_reply_req, rr_reply_pending, rr_reply_sent, rr_reply_overrun;
logic [127:0] rr_reply_msg;
logic [4:0]   rr_reply_len;

tx_reply_ctrl u_tx_reply_ctrl (
    .clk           (pll_clk_out),
    .rst_n         (sync_pll_rst_n),
    .rd_valid      (rx_rd_reply_valid),
    .rd_data       (rx_rd_reply_data),
    .pix_valid     (pix_rpy_held),      // destination-side held request
    .pix_msg       (pix_reply_msg),
    .pix_accept    (pix_rpy_accept_130),
    .tx_seq_busy   (tx_seq_busy),
    .mac_busy      (mac_busy),
    .reply_req     (rr_reply_req),
    .reply_msg     (rr_reply_msg),
    .reply_len     (rr_reply_len),
    .reply_pending (rr_reply_pending),
    .reply_sent    (rr_reply_sent),
    .reply_overrun (rr_reply_overrun)
);

logic         tx_mac_msg_valid;
logic [127:0] tx_mac_msg_data;
logic [4:0]   tx_mac_msg_len;

// -----------------------------------------------------------------------------
// TX MAC SOURCE MUX -- THE SELECT MUST OUTLIVE THE REQUEST
// -----------------------------------------------------------------------------
// tx_mac does NOT capture the message in the cycle it sees msg_valid. It
// samples msg_valid in MAC_IDLE, moves to MAC_LOAD, and captures on the NEXT
// clock:
//
//     if (cur_state == MAC_LOAD) begin
//         msg_buf  <= msg_data;          // tx_mac.sv
//         last_idx <= len_m1[3:0];
//     end
//
// mac_busy is registered from next_state, so it is already HIGH in that
// MAC_LOAD cycle. And rr_reply_req is combinational:
//
//     reply_req = pending && !tx_seq_busy && !mac_busy;   // tx_reply_ctrl.sv
//
// so rr_reply_req falls in exactly the cycle tx_mac performs the capture.
//
// Selecting the mux on rr_reply_req alone therefore switched the data and
// length buses back to the IMAGE path one cycle too early -- tx_mac latched
// tx_sequencer's msg output instead of the reply. With the image path idle,
// tx_sequencer drives msg_composer with row_latch = col_latch = pixel_latch =
// 0, which composes a perfectly well-formed 16-byte frame whose row, column
// and RGB fields are all zero. That is precisely the observed failure: correct
// braces, correct 'R'/'C'/'P' markers, zero payload. The reply data was never
// wrong -- it was never sampled.
//
// The same defect truncated nothing on the Register Read path but silently
// promoted it to 16 bytes, because last_idx is latched in MAC_LOAD too and saw
// the image path's 5'd16.
//
// reply_owns_mac extends the select across the capture. It is set the cycle the
// request is made and released on reply_sent, which tx_reply_ctrl raises once
// the MAC has genuinely taken the message -- so the buses are guaranteed stable
// for the whole of MAC_LOAD regardless of how many cycles the MAC takes to get
// there.
// -----------------------------------------------------------------------------
logic reply_owns_mac;

always_ff @(posedge pll_clk_out or negedge sync_pll_rst_n) begin
    if (!sync_pll_rst_n)      reply_owns_mac <= 1'b0;
    else if (rr_reply_req)    reply_owns_mac <= 1'b1;
    else if (rr_reply_sent)   reply_owns_mac <= 1'b0;
end

logic reply_drives_mac;
assign reply_drives_mac = rr_reply_req || reply_owns_mac;

// msg_valid stays qualified on rr_reply_req -- tx_mac only samples it in
// MAC_IDLE and must see exactly one assertion -- while data and length follow
// the extended select so they are still correct at MAC_LOAD.
assign tx_mac_msg_valid = reply_drives_mac ? rr_reply_req  : msg_valid;
assign tx_mac_msg_data  = reply_drives_mac ? rr_reply_msg  : msg_data;
assign tx_mac_msg_len   = reply_drives_mac ? rr_reply_len  : 5'd16;

`ifndef SYNTHESIS
    a_tx_producers_exclusive: assert property (
        @(posedge pll_clk_out) disable iff (!sync_pll_rst_n)
        !(rr_reply_req && msg_valid)
    ) else $error("chip_top: reply and image message offered to tx_mac together");

    // Image traffic must always see the legacy 16-byte length. Qualified on
    // the EXTENDED select: while the reply owns the MAC the length bus
    // legitimately carries 6 or 16, and tx_sequencer is idle by construction
    // because reply_req is gated on !tx_seq_busy.
    a_image_len_16: assert property (
        @(posedge pll_clk_out) disable iff (!sync_pll_rst_n)
        (msg_valid && !reply_drives_mac) |-> (tx_mac_msg_len == 5'd16)
    ) else $error("chip_top: image message sent with a non-16 byte length");

    // The reply buses must still be selected when tx_mac actually captures.
    // mac_busy rising is the MAC_LOAD cycle.
    a_reply_held_through_load: assert property (
        @(posedge pll_clk_out) disable iff (!sync_pll_rst_n)
        ($rose(mac_busy) && $past(reply_drives_mac)) |-> reply_drives_mac
    ) else $error("chip_top: reply mux released before tx_mac captured");
`endif

tx_mac u_tx_mac (
    .clk      (pll_clk_out),
    .rst_n    (sync_pll_rst_n),
    .rx_mode  (1'b0),           // always TX mode
    .msg_valid(tx_mac_msg_valid),
    .msg_data (tx_mac_msg_data),
    .msg_len  (tx_mac_msg_len),
    .mac_busy (mac_busy),
    .phy_ready(phy_ready),
    .phy_data (phy_data),
    .phy_valid(phy_valid)
);

// -----------------------------------------------------------------------------
// TX PHY
// -----------------------------------------------------------------------------
tx_phy u_tx_phy (
    .clk      (pll_clk_out),
    .rst_n    (sync_pll_rst_n),
    .phy_valid(phy_valid),
    .phy_data (phy_data),
    .phy_ready(phy_ready),
    .tx_out   (UART_RXD_OUT),
    .led      (LED[0])
);

// -----------------------------------------------------------------------------
// RX chain: PHY -> MAC -> parser -> classifier
// Verified standalone (SVA + waveform) against rx_phy/rx_mac/rx_parser/
// rx_classifier before integration -- see rx pipeline testbench.
// -----------------------------------------------------------------------------
logic        rx_byte_valid;
logic [7:0]  rx_byte_val;
logic        rx_phy_busy;

logic         rx_mac_msg_valid;
logic [127:0] rx_mac_msg_data;
logic         rx_mac_busy;

logic        rx_parse_valid;
logic        rx_parse_error;
logic [9:0]  rx_row;
logic [9:0]  rx_col;
logic [23:0] rx_pixel;

logic        rx_classifier_valid;
logic        rx_classifier_error;
logic [9:0]  rx_row_q, rx_col_q;
logic [23:0] rx_pixel_q;

logic        rx_parity_err_pulse;

// -----------------------------------------------------------------------------
// Stage 2A: variable-length framing signals
// -----------------------------------------------------------------------------
// The receive path is now count-based rather than fixed-16. rx_msg_decode
// is combinational and is the single source of opcode truth; rx_mac holds
// no protocol knowledge and simply counts to expected_len.
//
// The loop rx_mac -> rx_msg_decode -> rx_mac closes through registers
// (msg_buf / byte_idx), so it is a combinational PATH, not a combinational
// loop. Depth is a handful of byte comparators feeding a 5-bit compare.
// -----------------------------------------------------------------------------
logic [rx_msg_pkg::BYTE_CNT_W-1:0] rx_expected_len;
rx_msg_pkg::msg_kind_t             rx_msg_kind_prov;   // provisional - rx_mac only
logic                              rx_kind_known;      // observability / TB use
logic [127:0]                      rx_frame_buf;       // LIVE buffer tap
logic [rx_msg_pkg::BYTE_CNT_W-1:0] rx_byte_cnt;        // LIVE byte count
rx_msg_pkg::msg_kind_t             rx_msg_kind_q;      // FINAL, latched

logic        rx_pw_parse_valid;
logic        rx_pw_parse_error;
logic [23:0] rx_pw_addr;
logic [23:0] rx_pw_pixel;

logic        rx_cmd_valid;
logic [23:0] rx_cmd_addr;
logic [23:0] rx_cmd_pixel;
logic        rx_bypass_active;

rx_phy u_rx_phy (
    .clk              (pll_clk_out),
    .rst_n            (sync_pll_rst_n),
    .rx_in            (UART_TXD_IN),
    .byte_valid       (rx_byte_valid),
    .rx_byte          (rx_byte_val),
    .parity_err_pulse (rx_parity_err_pulse),
    .rx_busy          (rx_phy_busy)
);

// Combinational frame-length / kind decoder. bypass_active comes from
// rx_classifier and is hardwired inactive for the whole of Stage 2A.
rx_msg_decode u_rx_msg_decode (
    .frame_buf     (rx_frame_buf),
    .byte_cnt      (rx_byte_cnt),
    .bypass_active (rx_bypass_active),
    .expected_len  (rx_expected_len),
    .msg_kind_prov (rx_msg_kind_prov),
    .kind_known    (rx_kind_known)
);

rx_mac u_rx_mac (
    .clk           (pll_clk_out),
    .rst_n         (sync_pll_rst_n),
    .byte_valid    (rx_byte_valid),
    .rx_byte       (rx_byte_val),
    .par_val_rst   (rx_parity_err_pulse),
    .expected_len  (rx_expected_len),
    .msg_kind_prov (rx_msg_kind_prov),
    .frame_buf     (rx_frame_buf),
    .byte_cnt      (rx_byte_cnt),
    .msg_valid     (rx_mac_msg_valid),
    .msg_data      (rx_mac_msg_data),
    .msg_kind_q    (rx_msg_kind_q),
    .mac_busy      (rx_mac_busy)
);

// Legacy {Rnnn,Cnnn,Vnnn} parser -- UNCHANGED. Still the active
// register-control path for the whole of Stage 2A.
rx_parser u_rx_parser (
    .msg_in      (rx_mac_msg_data),
    .parse_valid (rx_parse_valid),
    .parse_error (rx_parse_error),
    .row         (rx_row),
    .col         (rx_col),
    .pixel       (rx_pixel)
);

// New Single Pixel Write parser, running in parallel on the same frame.
rx_pixel_wr_parser u_rx_pixel_wr_parser (
    .msg_in         (rx_mac_msg_data),
    .pw_parse_valid (rx_pw_parse_valid),
    .pw_parse_error (rx_pw_parse_error),
    .pw_addr        (rx_pw_addr),
    .pw_pixel       (rx_pw_pixel)
);

// -----------------------------------------------------------------------------
// STAGE 3 / M5: Image Burst Write
// -----------------------------------------------------------------------------
// Burst write is entirely a RECEIVE-SIDE feature. rx_burst_ctrl unpacks each
// Burst Data frame into ordinary Stage 2C {address, pixel} commands and feeds
// them to the SAME command FIFO, so sram_wr_ctrl, mem_interlock, rgb_sram and
// the whole hardware-validated Stage 2C write path are reused untouched.
//
// Both burst parsers are combinational and run on every frame; msg_kind_q
// decides which one is authoritative, exactly as rx_parser and
// rx_pixel_wr_parser already do.
// -----------------------------------------------------------------------------
logic                              rx_hdr_frame_ok, rx_hdr_dims_ok, rx_hdr_valid;
logic [rx_burst_pkg::BURST_DIM_W-1:0] rx_burst_height, rx_burst_width;

logic                              rx_bdata_frame_ok, rx_bdata_error;
logic [rx_burst_pkg::BURST_PIX_PER_MSG-1:0]
      [rx_burst_pkg::BURST_PIX_W-1:0] rx_burst_pixels;

logic                               rx_burst_active;
logic                               rx_burst_done;
logic                               rx_burst_cmd_valid;
logic [rx_burst_pkg::BURST_ADDR_W-1:0] rx_burst_cmd_addr;
logic [rx_burst_pkg::BURST_PIX_W-1:0]  rx_burst_cmd_pixel;
logic                               rx_burst_err_hdr, rx_burst_err_data,
                                    rx_burst_err_unexp;

rx_burst_hdr_parser u_rx_burst_hdr_parser (
    .msg_in       (rx_mac_msg_data),
    .hdr_frame_ok (rx_hdr_frame_ok),
    .hdr_dims_ok  (rx_hdr_dims_ok),
    .hdr_valid    (rx_hdr_valid),
    .height       (rx_burst_height),
    .width        (rx_burst_width)
);

rx_burst_data_parser u_rx_burst_data_parser (
    .msg_in        (rx_mac_msg_data),
    .data_frame_ok (rx_bdata_frame_ok),
    .data_error    (rx_bdata_error),
    .pixels        (rx_burst_pixels)
);

// SOLE DRIVER of bypass_active. rx_classifier surrendered the port in M4;
// two drivers would be a real multi-driver conflict.
//
// cmd_ready is !cmd_fifo_full and must NOT be tied high: the controller
// samples it to decide whether to advance, and a command presented into a
// full FIFO is dropped silently by async_fifo's "if (wr_en && !full)" gate.
rx_burst_ctrl u_rx_burst_ctrl (
    .clk              (pll_clk_out),
    .rst_n            (sync_pll_rst_n),
    .msg_valid        (rx_mac_msg_valid),
    .msg_kind         (rx_msg_kind_q),
    .hdr_valid        (rx_hdr_valid),
    .height           (rx_burst_height),
    .width            (rx_burst_width),
    .data_frame_ok    (rx_bdata_frame_ok),
    .pixels           (rx_burst_pixels),
    .burst_abort      (1'b0),              // reset is the only recovery for now
    .cmd_ready        (!cmd_fifo_full),
    .cmd_valid        (rx_burst_cmd_valid),
    .cmd_addr         (rx_burst_cmd_addr),
    .cmd_pixel        (rx_burst_cmd_pixel),
    .bypass_active    (rx_bypass_active),  // -> rx_msg_decode
    .burst_active     (rx_burst_active),   // -> rx_classifier and, synchronised,
    .burst_done       (rx_burst_done),     //    to mem_interlock
    .err_hdr_invalid  (rx_burst_err_hdr),
    .err_data_invalid (rx_burst_err_data),
    .err_unexpected   (rx_burst_err_unexp)
);

// -----------------------------------------------------------------------------
// REGISTER READ: request parser
// -----------------------------------------------------------------------------
logic        rx_rr_frame_ok, rx_rr_addr_ok, rx_rr_valid, rx_rr_addr_err;
logic [23:0] rx_rr_addr_raw;
logic [5:0]  rx_rr_rgf_addr;

rx_reg_read_parser u_rx_reg_read_parser (
    .msg_in      (rx_mac_msg_data),
    .rr_frame_ok (rx_rr_frame_ok),
    .rr_addr_ok  (rx_rr_addr_ok),
    .rr_valid    (rx_rr_valid),
    .rr_addr_err (rx_rr_addr_err),
    .rr_addr     (rx_rr_addr_raw),
    .rr_rgf_addr (rx_rr_rgf_addr)
);

// -----------------------------------------------------------------------------
// SINGLE PIXEL READ: request parser
// -----------------------------------------------------------------------------
// Combinational, stable while rx_mac_msg_valid is high, exactly like the
// Register Read and Single Pixel Write parsers beside it. It validates the
// COMPLETE 24-bit row and column fields against the image geometry, so a
// coordinate with rubbish in its high bytes is rejected here and never
// reaches the memory domain at all.
// -----------------------------------------------------------------------------
logic        rx_pr_frame_ok, rx_pr_coord_ok, rx_pr_valid, rx_pr_coord_err;
logic [23:0] rx_pr_row_raw, rx_pr_col_raw;
logic [9:0]  rx_pr_row, rx_pr_col;

rx_pixel_rd_parser u_rx_pixel_rd_parser (
    .msg_in       (rx_mac_msg_data),
    .pr_frame_ok  (rx_pr_frame_ok),
    .pr_coord_ok  (rx_pr_coord_ok),
    .pr_valid     (rx_pr_valid),
    .pr_coord_err (rx_pr_coord_err),
    .pr_row_raw   (rx_pr_row_raw),
    .pr_col_raw   (rx_pr_col_raw),
    .pr_row       (rx_pr_row),
    .pr_col       (rx_pr_col)
);

logic       rx_rr_cmd_valid;
logic [5:0] rx_rr_cmd_addr;

rx_classifier u_rx_classifier (
    .clk              (pll_clk_out),
    .rst_n            (sync_pll_rst_n),
    .msg_valid        (rx_mac_msg_valid),
    .msg_kind         (rx_msg_kind_q),
    .parse_valid      (rx_parse_valid),
    .parse_error      (rx_parse_error),
    .row              (rx_row),
    .col              (rx_col),
    .pixel            (rx_pixel),
    .pw_parse_valid   (rx_pw_parse_valid),
    .pw_parse_error   (rx_pw_parse_error),
    .pw_addr          (rx_pw_addr),
    .pw_pixel         (rx_pw_pixel),
    .classifier_valid (rx_classifier_valid),
    .classifier_error (rx_classifier_error),
    .row_q            (rx_row_q),
    .col_q            (rx_col_q),
    .pixel_q          (rx_pixel_q),
    .cmd_valid        (rx_cmd_valid),
    .cmd_addr         (rx_cmd_addr),
    .cmd_pixel        (rx_cmd_pixel),
    .burst_active     (rx_burst_active),  // M4: input, from rx_burst_ctrl
    .rr_valid         (rx_rr_valid),
    .rr_addr_err      (rx_rr_addr_err),
    .rr_rgf_addr      (rx_rr_rgf_addr),
    .rr_cmd_valid     (rx_rr_cmd_valid),
    .rr_cmd_addr      (rx_rr_cmd_addr),
    .pr_valid         (rx_pr_valid),
    .pr_coord_err     (rx_pr_coord_err),
    .pr_row           (rx_pr_row),
    .pr_col           (rx_pr_col),
    .pr_cmd_valid     (rx_pr_cmd_valid),
    .pr_cmd_row       (rx_pr_cmd_row),
    .pr_cmd_col       (rx_pr_cmd_col)
);

// -----------------------------------------------------------------------------
// TEMPORARY STAGE 2A OBSERVABILITY -- REMOVE IN STAGE 2B
// -----------------------------------------------------------------------------
// Stage 2A stops at the classifier: there is no command FIFO, no write
// controller and no SRAM write logic yet, so rx_cmd_addr / rx_cmd_pixel
// have no consumer. This sticky flag gives the milestone a single
// hardware-observable endpoint: it sets on the first correctly framed
// Single Pixel Write and stays set until reset.
//
// Once the command FIFO exists in Stage 2B this flag and its LED
// assignment below should be deleted.
// -----------------------------------------------------------------------------
logic pix_wr_seen_sticky;

always_ff @(posedge pll_clk_out or negedge sync_pll_rst_n) begin
    if (!sync_pll_rst_n) pix_wr_seen_sticky <= 1'b0;
    else if (rx_cmd_valid) pix_wr_seen_sticky <= 1'b1;
end

// -----------------------------------------------------------------------------
// STAGE 2B: pixel-command clock-domain crossing
// -----------------------------------------------------------------------------
// Carries the RAW protocol values across from the 130 MHz RX domain to the
// 100 MHz memory domain, unchanged:
//
//     cmd_fifo_wr_data[47:24] = cmd_addr [23:0]
//     cmd_fifo_wr_data[23: 0] = cmd_pixel[23:0] = {R,G,B}
//
// No SRAM word address or byte lane is derived here -- that mapping belongs
// to Stage 2C, together with the write controller and arbitration.
//
// A FIFO rather than a cdc_cmd_sync handshake because Stage 2C's consumer can
// be stalled for the whole duration of an image transmission by the arbiter,
// during which commands must queue rather than be dropped. In Stage 2B the
// monitor drains at one command per clock against a producer limited to about
// one per 15 us, so occupancy never exceeds one.
//
// FULL HANDLING: async_fifo's own "if (wr_en && !full)" gate prevents pointer
// corruption but drops the write silently, so cmd_ovf_sticky below detects
// exactly that. It must never light in Stage 2B -- the FIFO cannot fill at
// these rates. The almost_full -> UART_CTS backpressure path is deliberately
// deferred to Stage 2C, where the arbiter creates a real stall condition;
// adding it now would put a gray2bin XOR cascade on the 130 MHz domain for no
// benefit and risk the timing closure just achieved.
// -----------------------------------------------------------------------------
localparam int CMD_FIFO_W = 48;   // {cmd_addr[23:0], cmd_pixel[23:0]}

logic                  cmd_fifo_wr_en;
logic [CMD_FIFO_W-1:0] cmd_fifo_wr_data;
logic                  cmd_fifo_full;
logic                  cmd_fifo_rd_en;
logic [CMD_FIFO_W-1:0] cmd_fifo_rd_data;
logic                  cmd_fifo_empty;

logic                  sram_wr_seen;      // sticky: a pixel was written
logic                  sram_wr_rejected;  // sticky: a command was discarded

// -----------------------------------------------------------------------------
// Command mux -- VALID-DRIVEN, not mode-driven.
//
// Selecting on rx_burst_cmd_valid rather than on rx_bypass_active matters.
// A mode signal can drop while a command is still pending on the interface,
// which would withdraw it mid-handshake -- twice during M3 that was a real
// defect. Selecting on the valid itself cannot: the mux follows exactly the
// signal that says a command is present.
//
// MUTUAL EXCLUSION (established in M4, two independent layers):
//   structural -- while bypass_active is high rx_msg_decode classifies every
//                 frame as MSG_BURST_DATA, so MSG_PIX_WRITE cannot occur;
//   explicit   -- rx_classifier gates all of its outputs on !burst_active.
// The assertion below proves the two valids never overlap.
//
// SINGLE PIXEL WRITE IS UNAFFECTED. With rx_burst_cmd_valid low -- which is
// its value at every instant outside a burst -- both expressions reduce
// exactly to the Stage 2C forms:
//     cmd_fifo_wr_en   = rx_cmd_valid
//     cmd_fifo_wr_data = {rx_cmd_addr, rx_cmd_pixel}
// so the validated path is bit-identical, not merely equivalent.
// -----------------------------------------------------------------------------
assign cmd_fifo_wr_en   = rx_burst_cmd_valid || rx_cmd_valid;
assign cmd_fifo_wr_data = rx_burst_cmd_valid
                            ? {rx_burst_cmd_addr, rx_burst_cmd_pixel}
                            : {rx_cmd_addr,       rx_cmd_pixel};

`ifndef SYNTHESIS
    // The two command producers must never drive the FIFO in the same cycle.
    a_no_cmd_overlap: assert property (
        @(posedge pll_clk_out) disable iff (!sync_pll_rst_n)
        !(rx_burst_cmd_valid && rx_cmd_valid)
    ) else $error("chip_top: burst and single-pixel commands overlapped");

    // rx_classifier has no cmd_ready input -- it relies on CTS and the
    // overflow detector. Make a drop visible rather than silent.
    a_cls_cmd_not_dropped: assert property (
        @(posedge pll_clk_out) disable iff (!sync_pll_rst_n)
        rx_cmd_valid |-> !cmd_fifo_full
    ) else $error("chip_top: single-pixel command presented into a full FIFO");
`endif

// .DW(48) is REQUIRED. Without it the ports default to FIFO_DATA_WIDTH (24)
// and this 48-bit payload is silently truncated to its low half -- cmd_pixel
// would cross and cmd_addr would be discarded, sending every write to
// word_addr 0 / byte_lane 0. Truncation is only a width warning, never an
// elaboration error, so nothing stops a build without it.
async_fifo #(
    .DW (CMD_FIFO_W)
) u_cmd_fifo (
    .wr_clk       (pll_clk_out),
    .wr_rst_n     (sync_pll_rst_n),
    .wr_en        (cmd_fifo_wr_en),
    .wr_data      (cmd_fifo_wr_data),
    .full         (cmd_fifo_full),
    .almost_full  (),                  // Stage 2C: -> UART_CTS backpressure
    .rd_clk       (CLK100MHZ),
    .rd_rst_n     (sync_rst_n),
    .rd_en        (cmd_fifo_rd_en),
    .rd_data      (cmd_fifo_rd_data),
    .empty        (cmd_fifo_empty),
    .almost_empty ()                   // unused
);

// -----------------------------------------------------------------------------
// STAGE 2C: single-pixel SRAM write path
// -----------------------------------------------------------------------------
// cmd_monitor is gone -- sram_wr_ctrl is the real consumer of the command FIFO
// now. It pops one command per clock when permitted, maps the raw 24-bit pixel
// index onto a word address and byte lane, and drives all three channel SRAMs
// on the same cycle.
//
// mem_interlock keeps the SRAM read and write ports mutually exclusive. It is
// not the full arbiter -- no drain barrier, no CTS hold-off -- just the
// smallest structure that makes exclusion structural rather than a consequence
// of UART pacing. See mem_interlock.sv for why gating on rom_seq_busy alone is
// insufficient.
// -----------------------------------------------------------------------------
sram_wr_ctrl #(
    .CMD_W (CMD_FIFO_W)
) u_sram_wr_ctrl (
    .clk         (CLK100MHZ),
    .rst_n       (sync_rst_n),
    .cmd_empty   (cmd_fifo_empty),
    .cmd_rd_data (cmd_fifo_rd_data),
    .cmd_rd_en   (cmd_fifo_rd_en),
    .wr_allowed  (sram_wr_allowed),
    .wr_busy     (sram_wr_busy),
    .wr_en       (sram_wr_en),
    .wr_be       (sram_wr_be),
    .wr_addr     (sram_wr_addr),
    .wr_data_r   (sram_wr_data_r),
    .wr_data_g   (sram_wr_data_g),
    .wr_data_b   (sram_wr_data_b),
    .wr_seen     (sram_wr_seen),
    .wr_rejected (sram_wr_rejected)
);

// -----------------------------------------------------------------------------
// burst_active: 130 MHz -> 100 MHz
// -----------------------------------------------------------------------------
// A LEVEL, so a plain two-flop synchroniser is correct and sufficient. There
// is no bus to skew: the destination sees either the old value or the new
// one, never a mixture. cdc_pulse_sync would be wrong here -- it converts its
// input to a toggle, so a level held high would flip the toggle every cycle.
//
// Both edges are delayed by two to three destination clocks. Deassertion
// arriving late is harmless: it merely defers a read a few tens of
// nanoseconds. Assertion arriving late opens a ~30 ns window in which
// mem_interlock still sees burst_active low. See the analysis in the
// integration notes for why read_go cannot fire in that window.
// -----------------------------------------------------------------------------
logic burst_active_100;

cdc_level_sync u_cdc_burst_active (
    .src_level (rx_burst_active),
    .dst_clk   (CLK100MHZ),
    .dst_rst_n (sync_rst_n),
    .dst_level (burst_active_100)
);

// -----------------------------------------------------------------------------
// SINGLE PIXEL READ: request crossing, 130 MHz -> 100 MHz
// -----------------------------------------------------------------------------
// rx_classifier validates and emits the request on pll_clk_out; pixel_rd_ctrl
// consumes it on CLK100MHZ. Reuses cdc_cmd_sync rather than adding a new
// primitive: it is the same atomic single-entry crossing already carrying the
// RGF command and the Register Read value, and the pacing here is even gentler
// -- one request per 16-byte UART frame, ~21.7 us, against a handful of cycles
// of synchroniser latency.
//
// The 20-bit payload is {row, col}. ADDR_W is 1 because only the strobe and
// the data are needed; is_write is tied off.
//
// dst_valid is a single 100 MHz cycle, which is exactly the one-cycle strobe
// pixel_rd_ctrl's req_valid expects, and dst_wdata is held between transfers
// so the coordinates are stable when it fires.
// -----------------------------------------------------------------------------
cdc_cmd_sync #(
    .ADDR_W (1),
    .DATA_W (20)
) u_cdc_pix_req (
    .src_clk      (pll_clk_out),
    .src_rst_n    (sync_pll_rst_n),
    .src_valid    (rx_pr_cmd_valid),
    .src_is_write (1'b0),
    .src_addr     (1'b0),
    .src_wdata    ({rx_pr_cmd_row, rx_pr_cmd_col}),
    .dst_valid    (pix_req_valid_100),
    .dst_is_write (),
    .dst_addr     (),
    .dst_wdata    (pix_req_data_100),
    .dst_clk      (CLK100MHZ),
    .dst_rst_n    (sync_rst_n)
);

// -----------------------------------------------------------------------------
// SINGLE PIXEL READ: controller, 100 MHz memory domain
// -----------------------------------------------------------------------------
pixel_rd_ctrl u_pixel_rd_ctrl (
    .clk          (CLK100MHZ),
    .rst_n        (sync_rst_n),
    .req_valid    (pix_req_valid_100),
    .req_row      (pix_req_data_100[19:10]),
    .req_col      (pix_req_data_100[ 9: 0]),
    .pix_rd_req   (pix_rd_req),
    .pix_rd_gnt   (pix_rd_gnt),
    .pix_rd_done  (pix_rd_done),
    .sram_rd_en   (pix_sram_rd_en),
    .sram_rd_addr (pix_sram_rd_addr),
    .red_data     (red_data),
    .green_data   (green_data),
    .blue_data    (blue_data),
    .rpy_valid    (pix_rpy_valid_100),
    .rpy_accept   (pix_rpy_accept_100),
    .rpy_row      (pix_rpy_row),
    .rpy_col      (pix_rpy_col),
    .rpy_pixel    (pix_rpy_pixel),
    .busy         (pix_rd_busy),
    .req_overrun  (pix_rd_overrun)
);

// -----------------------------------------------------------------------------
// SINGLE PIXEL READ: reply crossing, 100 MHz -> 130 MHz
// -----------------------------------------------------------------------------
// The reply is a HELD handshake, not a pulse, because tx_reply_ctrl can refuse
// for the entire duration of an image transfer (~1.5 s). Three signals cross:
//
//   pix_rpy_valid_100   level, 100 -> 130, via cdc_level_sync
//   {row, col, pixel}   data, 100 -> 130, UNSYNCHRONISED and guarded by the
//                       level above
//   pix_accept          pulse, 130 -> 100, via cdc_pulse_sync
//
// The unsynchronised data path is correct rather than a shortcut, and for the
// same reason cdc_cmd_sync crosses its payload unsynchronised: pixel_rd_ctrl
// writes the payload in P_CAP, one full 100 MHz cycle BEFORE rpy_valid rises
// in P_SEND, and then holds it until the accept comes back. The level needs
// two to three 130 MHz clocks to traverse its synchroniser, so by the time
// this domain can observe pix_valid high the payload has been stable for
// >= 30 ns and it stays stable until this domain answers. Synchronising the
// bits individually would actively break that atomicity.
//
// msg_composer is REUSED here rather than duplicated. It already emits
// {R<..>,C<..>,P<R,G,B>} for every pixel of a full-image transfer, which is
// byte-for-byte the Single Pixel Read reply, so the reply format is identical
// to image traffic by construction. Its msg output is a packed [15:0][7:0]
// with byte 0 in bits [7:0] -- exactly the LSB-first order tx_mac expects.
// -----------------------------------------------------------------------------
// ---- source side: turn the held valid into ONE send event -------------------
// pixel_rd_ctrl holds rpy_valid for the whole of P_SEND, which may be
// milliseconds. cdc_cmd_sync wants a single-cycle src_valid, so the rising
// edge is extracted here. pixel_rd_ctrl itself is untouched -- its unit test
// passes and its interface is unchanged.
always_ff @(posedge CLK100MHZ or negedge sync_rst_n) begin
    if (!sync_rst_n) pix_rpy_valid_100_d <= 1'b0;
    else             pix_rpy_valid_100_d <= pix_rpy_valid_100;
end

assign pix_rpy_send = pix_rpy_valid_100 && !pix_rpy_valid_100_d;

// ---- the crossing itself: ONE atomic 44-bit transaction ---------------------
// The whole reply -- {row, col, pixel} -- crosses as a single payload behind a
// single toggle, captured into cdc_cmd_sync's source register on the same edge
// that flips that toggle. The destination cannot observe a mixture of two
// transactions, and cannot observe the payload before it is stable, because the
// toggle needs two destination edges to traverse the synchroniser.
//
// This replaces an earlier arrangement that synchronised only the valid LEVEL
// and let the payload cross combinationally underneath it. That was not a
// coherent transfer: it made the 128-bit msg_composer OUTPUT the thing crossing
// domains, with no structural guarantee of stability during the destination's
// settle window -- correctness rested entirely on a timing argument rather than
// on the hardware.
cdc_cmd_sync #(
    .ADDR_W (1),
    .DATA_W (44)
) u_cdc_pix_rpy (
    .src_clk      (CLK100MHZ),
    .src_rst_n    (sync_rst_n),
    .src_valid    (pix_rpy_send),
    .src_is_write (1'b0),
    .src_addr     (1'b0),
    .src_wdata    ({pix_rpy_row, pix_rpy_col, pix_rpy_pixel}),
    .dst_valid    (pix_rpy_valid_130),
    .dst_is_write (),
    .dst_addr     (),
    .dst_wdata    (pix_rpy_data_130),
    .dst_clk      (pll_clk_out),
    .dst_rst_n    (sync_pll_rst_n)
);

// ---- destination side: hold the transaction until tx_reply_ctrl takes it ----
// cdc_cmd_sync delivers exactly one dst_valid strobe. tx_reply_ctrl may be
// unable to accept for the entire duration of an image transfer, so the strobe
// is converted back into a held request HERE, in the destination domain, where
// holding it is free and race-free. The payload is captured at the same edge.
//
// This is what propagates backpressure: pix_rpy_held stays up, tx_reply_ctrl's
// pix_accept stays down, no acknowledge crosses back, and pixel_rd_ctrl remains
// in P_SEND with its payload intact.
always_ff @(posedge pll_clk_out or negedge sync_pll_rst_n) begin
    if (!sync_pll_rst_n) begin
        pix_rpy_held        <= 1'b0;
        pix_rpy_payload_130 <= '0;
    end
    else if (pix_rpy_valid_130) begin
        pix_rpy_held        <= 1'b1;
        pix_rpy_payload_130 <= pix_rpy_data_130;
    end
    else if (pix_rpy_accept_130) begin
        pix_rpy_held        <= 1'b0;
    end
end

// msg_composer now lives ENTIRELY in the 130 MHz domain, driven from a
// registered payload in that same domain. Nothing combinational crosses.
msg_composer u_pix_reply_composer (
    .row   (pix_rpy_payload_130[43:34]),
    .col   (pix_rpy_payload_130[33:24]),
    .pixel (pix_rpy_payload_130[23: 0]),
    .msg   (pix_reply_msg)
);

`ifndef SYNTHESIS
    // A transaction is never delivered on top of one still awaiting acceptance.
    a_pix_rpy_no_overwrite: assert property (
        @(posedge pll_clk_out) disable iff (!sync_pll_rst_n)
        pix_rpy_valid_130 |-> !pix_rpy_held
    ) else $error("chip_top: pixel reply delivered while one was still held");

    // The captured payload is stable for as long as it is on offer.
    a_pix_rpy_stable: assert property (
        @(posedge pll_clk_out) disable iff (!sync_pll_rst_n)
        (pix_rpy_held && !pix_rpy_accept_130) |=> $stable(pix_rpy_payload_130)
    ) else $error("chip_top: held pixel reply payload changed");

    // The source must not withdraw the reply before the acknowledge lands.
    a_pix_src_holds: assert property (
        @(posedge CLK100MHZ) disable iff (!sync_rst_n)
        (pix_rpy_valid_100 && !pix_rpy_accept_100)
            |=> ($stable(pix_rpy_row) && $stable(pix_rpy_col) &&
                 $stable(pix_rpy_pixel))
    ) else $error("chip_top: source released the reply payload before the ack");
`endif

cdc_pulse_sync u_cdc_pix_rpy_accept (
    .src_clk   (pll_clk_out),
    .src_rst_n (sync_pll_rst_n),
    .src_pulse (pix_rpy_accept_130),
    .dst_clk   (CLK100MHZ),
    .dst_rst_n (sync_rst_n),
    .dst_pulse (pix_rpy_accept_100)
);

mem_interlock u_mem_interlock (
    .clk          (CLK100MHZ),
    .rst_n        (sync_rst_n),
    .start_req    (start_pulse),       // raw RGF request
    .rom_seq_busy (rom_seq_busy),
    .img_done     (tx_img_done_100),   // full-transmission completion
    .read_go      (read_go),           // -> rom_sequencer.start
    .cmd_empty    (cmd_fifo_empty),
    .wr_busy      (sram_wr_busy),
    .burst_active (burst_active_100),   // closes the inter-frame gaps
    .wr_allowed   (sram_wr_allowed),
    .pix_rd_req   (pix_rd_req),
    .pix_rd_done  (pix_rd_done),
    .pix_rd_gnt   (pix_rd_gnt),
    .pix_rd_owner (pix_rd_owner)       // -> SRAM read-port mux above
);

// Command FIFO overflow detector, 130 MHz write domain. Sticky, and must stay
// clear: the write path drains at one command per clock against a producer
// limited to roughly one command per 15 us.
logic cmd_ovf_sticky;

always_ff @(posedge pll_clk_out or negedge sync_pll_rst_n) begin
    if (!sync_pll_rst_n)                       cmd_ovf_sticky <= 1'b0;
    else if (cmd_fifo_wr_en && cmd_fifo_full)  cmd_ovf_sticky <= 1'b1;
end

// -----------------------------------------------------------------------------
// Config RGF + dispatcher
// -----------------------------------------------------------------------------
// Register indexing: reuses the existing {R###,C###,V###} framing rather
// than a new wire format. row_q acts as a REGISTER INDEX (not a raw byte
// address): index 0=IMG_STATUS, 1=IMG_TX_MON, 2=IMG_CTRL, 3=FIFO_STATUS,
// matching rgf_pkg.sv's ADDR = index*4 spacing. pixel_q (24 bits, zero-
// extended) is the write data.
//
// col_q now carries the read/write opcode: col EVEN = write (col=000,
// the value every command so far has used, so existing write commands
// are unaffected), col ODD = read. e.g. {R001,C001,V000} reads (and
// read-to-clears) IMG_TX_MON.
//
// pc_addr is asserted ONLY during the classifier_valid cycle, parked at
// an unmapped address (8'hFF) otherwise. This matters even for the
// write-only V1: leaving pc_addr as a continuous passthrough of row_q
// (as an earlier version of this file did) meant it stayed parked at
// whatever address the LAST message used, indefinitely, with pc_wen=0
// almost always -- which would have caused IMG_TX_MON to be silently
// read-cleared every single cycle the instant row_q ever latched to 1,
// breaking the interlock without any command asking for that.
//
// Read REPLIES (PC reading a value back over TX) still don't exist --
// msg_composer has no register-read-response format yet. A read command
// is still accepted and still clears IMG_TX_MON as intended; the PC just
// can't see the value it read yet, only that the clear/re-arm happened
// (observable via a subsequent start succeeding).
//
// To trigger an image send: {R002,C000,V001} (row=2 IMG_CTRL, col even
// = write, val=1 sets start).
// To clear IMG_TX_MON and re-arm the interlock: {R001,C001,V000}
// (row=1 IMG_TX_MON, col odd = read).
// -----------------------------------------------------------------------------
logic [7:0]  rgf_pc_addr;
logic        rgf_pc_wen;
logic [31:0] rgf_pc_wdata;
logic [31:0] rgf_pc_rdata;   // unused until a read-reply path exists
logic        rgf_start_img_read;
logic        rgf_status_wen;
logic [7:0]  rgf_status_addr;
logic [31:0] rgf_status_wdata;

// -----------------------------------------------------------------------------
// CHANGE A: legacy RGF command crossing, 130 MHz -> 100 MHz
// -----------------------------------------------------------------------------
// Previously these three signals were driven combinationally straight from
// rx_classifier's 130 MHz outputs and sampled by rgf on CLK100MHZ:
//
//   assign rgf_pc_wen   = rx_classifier_valid && !rx_col_q[0];
//   assign rgf_pc_addr  = rx_classifier_valid ? {2'b00,rx_row_q[5:0],2'b00} : 8'hFF;
//   assign rgf_pc_wdata = {8'b0, rx_pixel_q};
//
// Two defects. First, rx_classifier_valid is a single 130 MHz cycle -- 7.69 ns
// against a 10 ns sampling period -- so the command could be missed outright.
// Second, rgf_pc_addr was QUALIFIED BY that pulse, which made the address bus
// itself a 7.69 ns transient; a 100 MHz edge landing mid-transition could
// capture a mixture of old and new bits.
//
// That is what made the IMG_TX_MON read-to-clear unreliable: rgf implements
// it as a level-sensitive decode (!pc_wen && pc_addr == IMG_TX_MON_ADDR), so a
// missed address transient means complete is never cleared and the following
// start stays blocked by the interlock -- the Stage 4 failure observed on
// hardware.
//
// cdc_cmd_sync captures {is_write, addr, wdata} atomically in the 130 MHz
// domain and delivers them with a single clean 100 MHz strobe. The IDLE_ADDR
// behaviour lives inside that module, so the address is 8'hFF on every cycle
// except the one valid cycle and the read-to-clear fires exactly once.
//
// The source-side expressions are unchanged from the originals above; they
// simply move from combinational assigns into the module's source port.
// rx_classifier_valid needs no extra qualification -- Stage 2A already gates
// it to msg_kind == MSG_LEGACY_RGF.
// -----------------------------------------------------------------------------
logic        rgf_cmd_valid_100;
logic        rgf_cmd_is_write_100;
logic [7:0]  rgf_cmd_addr_100;
logic [31:0] rgf_cmd_wdata_100;

// -----------------------------------------------------------------------------
// RGF command producer mux, 130 MHz domain
// -----------------------------------------------------------------------------
// Two producers now drive the single forward cdc_cmd_sync: the legacy
// {R,C,V} path and Register Read. They are mutually exclusive because a
// frame carries exactly one msg_kind_q -- rx_classifier asserts an assertion
// on that, and the one below repeats it at the point of use.
//
// The legacy expressions are unchanged; they simply move behind the mux.
// -----------------------------------------------------------------------------
logic        rgf_src_valid, rgf_src_is_write;
logic [7:0]  rgf_src_addr;
logic [31:0] rgf_src_wdata;

assign rgf_src_valid    = rx_classifier_valid || rx_rr_cmd_valid;
assign rgf_src_is_write = rx_classifier_valid ? !rx_col_q[0] : 1'b0;
assign rgf_src_addr     = rx_classifier_valid ? {2'b00, rx_row_q[5:0], 2'b00}
                                              : {2'b00, rx_rr_cmd_addr};
assign rgf_src_wdata    = rx_classifier_valid ? {8'b0, rx_pixel_q} : 32'd0;

`ifndef SYNTHESIS
    a_one_rgf_producer_top: assert property (
        @(posedge pll_clk_out) disable iff (!sync_pll_rst_n)
        !(rx_classifier_valid && rx_rr_cmd_valid)
    ) else $error("chip_top: legacy and Register Read RGF commands overlapped");
`endif

cdc_cmd_sync #(
    .ADDR_W    (8),
    .DATA_W    (32),
    .IDLE_ADDR (8'hFF)     // decodes to none of 0x00/04/08/0C/10/14
) u_cdc_rgf_cmd (
    .src_clk      (pll_clk_out),
    .src_rst_n    (sync_pll_rst_n),
    .src_valid    (rgf_src_valid),
    .src_is_write (rgf_src_is_write),
    .src_addr     (rgf_src_addr),
    .src_wdata    (rgf_src_wdata),
    .dst_valid    (rgf_cmd_valid_100),
    .dst_is_write (rgf_cmd_is_write_100),
    .dst_addr     (rgf_cmd_addr_100),
    .dst_wdata    (rgf_cmd_wdata_100),
    .dst_clk      (CLK100MHZ),
    .dst_rst_n    (sync_rst_n)
);

// -----------------------------------------------------------------------------
// REGISTER READ: capture the value and return it to the 130 MHz domain
// -----------------------------------------------------------------------------
// rgf.pc_rdata is a combinational mux on pc_addr, and cdc_cmd_sync presents
// the address for exactly ONE cycle (IDLE_ADDR 0xFF otherwise). So the value
// is captured in precisely the cycle the RGF read happens -- which is also
// the cycle IMG_TX_MON's read-to-clear fires, leaving that side effect
// completely untouched.
// -----------------------------------------------------------------------------
logic        rgf_rd_strobe;
logic [31:0] rgf_rd_value;

always_ff @(posedge CLK100MHZ or negedge sync_rst_n) begin
    if (!sync_rst_n) begin
        rgf_rd_strobe <= 1'b0;
        rgf_rd_value  <= 32'd0;
    end
    else begin
        rgf_rd_strobe <= rgf_cmd_valid_100 && !rgf_cmd_is_write_100;
        if (rgf_cmd_valid_100 && !rgf_cmd_is_write_100)
            rgf_rd_value <= rgf_pc_rdata;
    end
end

// Return leg, 100 -> 130 MHz. Reuses the hardware-validated atomic command
// CDC rather than adding a new primitive; ADDR_W is 1 because only the data
// and the strobe are needed.
logic        rx_rd_reply_valid;
logic [31:0] rx_rd_reply_data;

cdc_cmd_sync #(
    .ADDR_W (1),
    .DATA_W (32)
) u_cdc_rgf_rdata (
    .src_clk      (CLK100MHZ),
    .src_rst_n    (sync_rst_n),
    .src_valid    (rgf_rd_strobe),
    .src_is_write (1'b0),
    .src_addr     (1'b0),
    .src_wdata    (rgf_rd_value),
    .dst_valid    (rx_rd_reply_valid),
    .dst_is_write (),
    .dst_addr     (),
    .dst_wdata    (rx_rd_reply_data),
    .dst_clk      (pll_clk_out),
    .dst_rst_n    (sync_pll_rst_n)
);

assign rgf_pc_wen   = rgf_cmd_valid_100 && rgf_cmd_is_write_100;
assign rgf_pc_addr  = rgf_cmd_addr_100;   // IDLE_ADDR (8'hFF) while !dst_valid
assign rgf_pc_wdata = rgf_cmd_wdata_100;

// -----------------------------------------------------------------------------
// CHANGE B: 130 MHz -> 100 MHz pulse clock-domain crossings
// -----------------------------------------------------------------------------
// tx_img_done and rx_parity_err_pulse are both single-cycle pulses
// generated on pll_clk_out (130 MHz, 7.69 ns) and consumed by rgf on
// CLK100MHZ (10 ns). A 7.69 ns pulse is narrower than the destination
// sampling period, so a direct connection can miss it entirely.
//
// Because both clocks come from the same MMCM their phase relationship is
// fixed rather than random, which is why the direct connection worked at
// all -- but it is fixed only for a given placement, and shifts with any
// re-place-and-route. That is what made the legacy RGF interlock path
// unreliable: a missed tx_img_done leaves IMG_TX_MON.complete clear, so
// the PC's completion poll times out.
//
// cdc_pulse_sync converts each pulse to a toggle, synchronises the toggle
// with two flops, and regenerates a clean one-cycle pulse in the 100 MHz
// domain. A level change cannot be missed regardless of clock ratio.
//
// Neither source module is modified: tx_sequencer and rx_phy still emit
// exactly the pulses they always did.
// -----------------------------------------------------------------------------
logic rx_parity_err_100;    // rx_parity_err_pulse, recovered on CLK100MHZ

cdc_pulse_sync u_cdc_tx_img_done (
    .src_clk   (pll_clk_out),
    .src_rst_n (sync_pll_rst_n),
    .src_pulse (tx_img_done),
    .dst_clk   (CLK100MHZ),
    .dst_rst_n (sync_rst_n),
    .dst_pulse (tx_img_done_100)
);

cdc_pulse_sync u_cdc_parity_err (
    .src_clk   (pll_clk_out),
    .src_rst_n (sync_pll_rst_n),
    .src_pulse (rx_parity_err_pulse),
    .dst_clk   (CLK100MHZ),
    .dst_rst_n (sync_rst_n),
    .dst_pulse (rx_parity_err_100)
);

// -----------------------------------------------------------------------------
// IMG_TX_MON status write: fires once, exactly when a transfer finishes.
// tx_img_done is a clean single-cycle pulse (registered off tx_sequencer's
// DONE state); tx_row/tx_col hold the last pixel's coordinates (255,255)
// at that same cycle, since they only get re-latched during the NEXT
// transfer's WAIT_DATA state. This is what actually closes the interlock
// loop: after this write, IMG_TX_MON.complete=1 blocks a new start until
// the PC reads IMG_TX_MON (read-to-clear), same as rgf's own testbench
// already proved in isolation.
//
// CHANGE B: the enable is now the synchronised pulse rather than the raw
// 130 MHz one. rgf_status_wdata still crosses unsynchronised, which is
// safe here and NOT changed by this milestone: tx_row/tx_col are held
// from the moment tx_img_done fires until the next transfer's WAIT_DATA,
// which cannot occur until the PC issues a fresh start command tens of
// microseconds later. They are therefore stable for far longer than the
// ~3-cycle synchroniser latency added below.
// -----------------------------------------------------------------------------
assign rgf_status_wen   = tx_img_done_100;
assign rgf_status_addr  = IMG_TX_MON_ADDR;
assign rgf_status_wdata = {10'b0, 1'b0 /*error*/, 1'b1 /*complete*/, tx_col, tx_row};

rgf u_rgf (
    .clk                (CLK100MHZ),
    .rst_n              (sync_rst_n),
    .pc_wen             (rgf_pc_wen),
    .pc_addr            (rgf_pc_addr),
    .pc_wdata           (rgf_pc_wdata),
    .pc_rdata           (rgf_pc_rdata),
    .status_wen         (rgf_status_wen),
    .status_addr        (rgf_status_addr),
    .status_wdata       (rgf_status_wdata),
    .img_height_in      (10'd256), // TODO: pull from rom_sequencer's IMG_HEIGHT param once exposed
    .img_width_in       (10'd256),
    .img_ready_in       (1'b1),    // ROM contents are static, always "ready"
    .start_img_read_out (rgf_start_img_read),
    .clk_sel_out        (clk_sel),
    .parity_fault_incr  (rx_parity_err_100),  // CHANGE B: was rx_parity_err_pulse (130 MHz)
    .fifo_full          (fifo_full),
    .fifo_empty         (fifo_empty_100),    // synchronised, see above
    .fifo_almost_full   (almost_full),
    .fifo_almost_empty  (almost_empty_100)   // synchronised, see above
);

assign start_pulse = rgf_start_img_read;

// -----------------------------------------------------------------------------
// RTS: assert (active-low, so drive 0) whenever the PC should hold off
// sending a new command. Previously only reflected tx_seq_busy; now also
// covers rom_seq_busy (ROM->FIFO drain can be active slightly out of
// phase with tx_seq_busy) and rx_mac_busy (spec: "when the sequencer is
// processing a received message"). Covers the case the RGF interlock
// alone doesn't: nothing in rgf.sv stops a second IMG_CTRL write from
// being ACCEPTED mid-transfer (complete/error are only set at the END of
// a transfer, so they're still both clear while one is running) --
// rom_sequencer happens to silently ignore a redundant start since it
// only samples `start` in IDLE, but RTS is the actual protocol-level
// guard telling the PC not to send one in the first place.
// -----------------------------------------------------------------------------
assign UART_CTS = ~(rom_seq_busy || tx_seq_busy || rx_mac_busy);

// -----------------------------------------------------------------------------
// LEDs: tie off unused for now
// -----------------------------------------------------------------------------
// TEMPORARY Stage 2C observability.
// LED[14]: at least one pixel has been written into the SRAMs.
// LED[13]: a command was discarded -- FIFO overflow or out-of-range address.
//          Must remain dark for well-formed, in-range traffic.
// TEMPORARY DIAGNOSTIC OVERRIDE -- restore to sram_wr_seen afterwards.
// The stall investigation sends no Single Pixel Write commands, so
// sram_wr_seen is guaranteed dark and carries no information.
assign LED[14] = tx_done_sticky;
// Either rejection cause. Separable by construction: run in-range traffic
// only and this must stay dark; then send a deliberately out-of-range
// command and it must light.
assign LED[13] = cmd_ovf_sticky || sram_wr_rejected;
// TEMPORARY Stage 2A observability -- see pix_wr_seen_sticky above.
// Remove together with the sticky flag when the Stage 2B command FIFO lands.
// TEMPORARY DIAGNOSTIC OVERRIDE -- restore to pix_wr_seen_sticky afterwards.
// LED[15] normally reports the Stage 2A pixel-write parser. The stall
// investigation sends no Single Pixel Write commands, so that flag is
// guaranteed dark and carries no information; the image FIFO overflow flag is
// the signal we actually need to see.
assign LED[15] = img_fifo_ovf_sticky;
assign LED[12] = counter_heartbeat_cnt[26];
assign LED[11] = clk_sel;
assign LED[10] = UART_RTS;
assign LED[9] = UART_CTS;
assign LED[8] = start_pulse;
assign LED[7] = rx_phy_busy;
assign LED[6] = rx_classifier_error;
assign LED[5] = rx_classifier_valid;
assign LED[4] = rx_mac_busy;
assign LED[3] = ~fifo_empty;
assign LED[2] = tx_seq_busy;
assign LED[1] = rom_seq_busy;
// -----------------------------------------------------------------------------
// 7-segment: tie off until RGF is added
// -----------------------------------------------------------------------------
//assign CATHODES = 8'hFF;    // all segments off (common anode display)
//assign AN       = 8'hFF;    // all digits off

endmodule : chip_top