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
// RGB BRAM SRAMs (Stage 1: read-only)
// -----------------------------------------------------------------------------
// Stage 1 of the final project: the three `rom` instances that used to
// live here are now rgb_sram instances. The read interface is
// intentionally bit- and cycle-identical to the ROM's -- same 1-cycle
// registered latency, same hold-last-value behaviour when the enable is
// low, same widths, depth and contents -- so rom_sequencer.sv below is
// UNCHANGED and sees no difference at all.
//
// Each instance's write port is tied inactive here. Stage 2 is what
// drives it; nothing on the receive side writes memory yet.
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

// Red channel SRAM
rgb_sram #(
    .DATA_WIDTH (memory_pkg::SRAM_DATA_WIDTH),
    .DEPTH      (memory_pkg::SRAM_DEPTH),
    .INIT_FILE  ("red_hex.mem")
) u_sram_red (
    .clk     (CLK100MHZ),
    .rd_en   (rom_rd_en),
    .rd_addr (rom_addr),
    .rd_data (red_data),
    .wr_en   (1'b0),
    .wr_be   ('0),
    .wr_addr ('0),
    .wr_data ('0)
);

// Green channel SRAM
rgb_sram #(
    .DATA_WIDTH (memory_pkg::SRAM_DATA_WIDTH),
    .DEPTH      (memory_pkg::SRAM_DEPTH),
    .INIT_FILE  ("green_hex.mem")
) u_sram_green (
    .clk     (CLK100MHZ),
    .rd_en   (rom_rd_en),
    .rd_addr (rom_addr),
    .rd_data (green_data),
    .wr_en   (1'b0),
    .wr_be   ('0),
    .wr_addr ('0),
    .wr_data ('0)
);

// Blue channel SRAM
rgb_sram #(
    .DATA_WIDTH (memory_pkg::SRAM_DATA_WIDTH),
    .DEPTH      (memory_pkg::SRAM_DEPTH),
    .INIT_FILE  ("blue_hex.mem")
) u_sram_blue (
    .clk     (CLK100MHZ),
    .rd_en   (rom_rd_en),
    .rd_addr (rom_addr),
    .rd_data (blue_data),
    .wr_en   (1'b0),
    .wr_be   ('0),
    .wr_addr ('0),
    .wr_data ('0)
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
// ROM Sequencer
// -----------------------------------------------------------------------------
rom_sequencer u_rom_sequencer (
    .clk         (CLK100MHZ),
    .rst_n       (sync_rst_n),
    .start       (start_pulse),
    .almost_full (almost_full),
    .almost_empty(almost_empty),
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
// TX MAC
// -----------------------------------------------------------------------------
logic        phy_valid;
logic [7:0]  phy_data;
logic        phy_ready;

tx_mac u_tx_mac (
    .clk      (pll_clk_out),
    .rst_n    (sync_pll_rst_n),
    .rx_mode  (1'b0),           // always TX mode
    .msg_valid(msg_valid),
    .msg_data (msg_data),
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
    .bypass_active    (rx_bypass_active)
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

logic                  cmd_seen_100;
logic [CMD_FIFO_W-1:0] cmd_last_100;

assign cmd_fifo_wr_en   = rx_cmd_valid;
assign cmd_fifo_wr_data = {rx_cmd_addr, rx_cmd_pixel};

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

// TEMPORARY Stage 2B consumer -- delete in Stage 2C along with LED[14].
cmd_monitor #(
    .CMD_W (CMD_FIFO_W)
) u_cmd_monitor (
    .clk         (CLK100MHZ),
    .rst_n       (sync_rst_n),
    .cmd_empty   (cmd_fifo_empty),
    .cmd_rd_data (cmd_fifo_rd_data),
    .cmd_rd_en   (cmd_fifo_rd_en),
    .cmd_seen    (cmd_seen_100),
    .cmd_last    (cmd_last_100)
);

// Overflow detector, 130 MHz write domain. Sticky, and must stay clear.
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

cdc_cmd_sync #(
    .ADDR_W    (8),
    .DATA_W    (32),
    .IDLE_ADDR (8'hFF)     // decodes to none of 0x00/04/08/0C/10/14
) u_cdc_rgf_cmd (
    .src_clk      (pll_clk_out),
    .src_rst_n    (sync_pll_rst_n),
    .src_valid    (rx_classifier_valid),
    .src_is_write (!rx_col_q[0]),                        // col even = write
    .src_addr     ({2'b00, rx_row_q[5:0], 2'b00}),       // register index * 4
    .src_wdata    ({8'b0, rx_pixel_q}),
    .dst_valid    (rgf_cmd_valid_100),
    .dst_is_write (rgf_cmd_is_write_100),
    .dst_addr     (rgf_cmd_addr_100),
    .dst_wdata    (rgf_cmd_wdata_100),
    .dst_clk      (CLK100MHZ),
    .dst_rst_n    (sync_rst_n)
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
logic tx_img_done_100;      // tx_img_done, recovered on CLK100MHZ
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
    .fifo_empty         (fifo_empty),
    .fifo_almost_full   (almost_full),
    .fifo_almost_empty  (almost_empty)
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
// TEMPORARY Stage 2B observability -- remove with cmd_monitor in Stage 2C.
// LED[14]: a command completed the 130 -> 100 MHz crossing and was captured.
// LED[13]: command FIFO overflow. Must remain dark; if it ever lights, a
//          pixel-write command was silently discarded.
assign LED[14] = cmd_seen_100;
assign LED[13] = cmd_ovf_sticky;
// TEMPORARY Stage 2A observability -- see pix_wr_seen_sticky above.
// Remove together with the sticky flag when the Stage 2B command FIFO lands.
assign LED[15] = pix_wr_seen_sticky;
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