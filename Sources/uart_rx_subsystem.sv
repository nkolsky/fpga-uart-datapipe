// uart_rx_subsystem.sv
// --------------------
// HIERARCHICAL CONTAINER ONLY. Introduced by the hierarchy refactor; it adds
// no logic of any kind.
//
// Holds the three modules that turn a serial line into complete, classified
// frames: the PHY that recovers bytes, the combinational decoder that says how
// long the current frame is, and the MAC that collects bytes into a frame.
//
//   rx_in --> rx_phy --> rx_mac --> msg_valid / msg_data / msg_kind_q
//                          ^ |
//                          | v
//                     rx_msg_decode
//
// -----------------------------------------------------------------------
// WHAT THIS MODULE CONTAINS -- AND DELIBERATELY DOES NOT
// -----------------------------------------------------------------------
// CONTAINS: three instantiations and the six nets that connect them.
// Nothing else. No always_ff, no always_comb, no assign, no state, no
// parameters, no protocol decisions. Every line of behaviour lives inside
// the three instantiated modules, exactly as it did when they sat directly
// in chip_top.
//
// The instantiations below are BYTE-FOR-BYTE the ones removed from
// chip_top.sv, with four port connections re-pointed at this module's ports:
//
//   chip_top net        this module's port
//   ------------------  ------------------
//   pll_clk_out         clk
//   sync_pll_rst_n      rst_n
//   UART_TXD_IN         rx_in
//   rx_bypass_active    bypass_active
//
// Every other connection keeps its original net name, so the diff against
// the old chip_top text is a pure relocation.
//
// -----------------------------------------------------------------------
// SINGLE CLOCK DOMAIN -- 130 MHz
// -----------------------------------------------------------------------
// Everything here runs on pll_clk_out and resets from sync_pll_rst_n. There
// is NO clock-domain crossing inside this module and none may ever be added:
// every crossing in this design lives at chip_top, deliberately, so the whole
// CDC map is visible in one file.
//
// bypass_active arrives from rx_burst_ctrl, which is also in the 130 MHz
// domain, so it needs no synchroniser and is used combinationally by
// rx_msg_decode -- unchanged from the original wiring.
//
// -----------------------------------------------------------------------
// NO PACKAGE IMPORTS -- INTENTIONAL, AND LOAD-BEARING
// -----------------------------------------------------------------------
// This file deliberately contains NO `import` statement. Types and constants
// from rx_msg_pkg are written fully qualified (rx_msg_pkg::msg_kind_t,
// rx_msg_pkg::BYTE_CNT_W), which is exactly how chip_top.sv already declares
// these same nets.
//
// The reason is specific, not stylistic. This design has three identifiers
// defined in more than one package (ADDR_WIDTH in fifo_pkg and rgf_pkg;
// mac_state_t in tx_pkg and tx_mac_pkg; state_t in rom_sequencer_pkg and
// tx_seq_pkg). Several modules wildcard-import their package at FILE scope,
// which places those names in the shared compilation unit, where a duplicate
// resolves by compile order. Adding a wildcard import here would put one more
// package into that same scope for no benefit. Fully qualified references
// cannot participate in the ambiguity at all.
//
// -----------------------------------------------------------------------
// THE rx_mac <-> rx_msg_decode LOOP IS A PATH, NOT A LOOP
// -----------------------------------------------------------------------
// rx_mac drives frame_buf/byte_cnt from registers; rx_msg_decode answers
// combinationally with expected_len/msg_kind_prov; rx_mac consumes those in
// its next-state logic. The cycle closes through msg_buf and byte_idx, so it
// is a combinational PATH between two flop boundaries. Wrapping it changes
// nothing about that -- the four nets are now internal to this module rather
// than internal to chip_top, and the timing arc is identical.
//
// -----------------------------------------------------------------------
// SIGNALS ABSORBED FROM chip_top (no longer visible at the top level)
// -----------------------------------------------------------------------
//   rx_byte_val        rx_phy -> rx_mac, byte payload
//   rx_expected_len    rx_msg_decode -> rx_mac
//   rx_msg_kind_prov   rx_msg_decode -> rx_mac (provisional kind)
//   rx_frame_buf       rx_mac -> rx_msg_decode (live buffer tap)
//   rx_byte_cnt        rx_mac -> rx_msg_decode (live byte count)
//   rx_byte_valid      rx_phy -> rx_mac
//   rx_kind_known      rx_msg_decode output, UNCONNECTED DOWNSTREAM
//
// rx_kind_known was already driven-but-unread in chip_top; that is preserved
// exactly rather than tidied away, because deleting the connection would be a
// change to the rx_msg_decode instantiation.
//
// rx_byte_valid is intentionally NOT exposed as a port. It is consumed only by
// rx_mac. tb_stage3_burst_pipeline observes it, and now reaches it through the
// hierarchy (dut.u_uart_rx_subsystem.rx_byte_valid) rather than through a
// port that exists only for the testbench.

`timescale 1ns/1ps

module uart_rx_subsystem (
    // ---- 130 MHz UART domain -------------------------------------------
    input  logic                       clk,             // pll_clk_out
    input  logic                       rst_n,           // sync_pll_rst_n

    // ---- serial input --------------------------------------------------
    input  logic                       rx_in,           // UART_TXD_IN

    // ---- live burst-mode override, from rx_burst_ctrl (130 MHz) --------
    input  logic                       bypass_active,   // rx_bypass_active

    // ---- PHY status ----------------------------------------------------
    output logic                       rx_phy_busy,
    output logic                       rx_parity_err_pulse,

    // ---- completed frame, out to the parsers and classifier ------------
    output logic                       rx_mac_msg_valid,
    output logic [127:0]               rx_mac_msg_data,
    output rx_msg_pkg::msg_kind_t      rx_msg_kind_q,
    output logic                       rx_mac_busy
);

// -------------------------------------------------------------------------
// Local interconnect. These are the chip_top nets that became internal;
// names are unchanged from chip_top.sv so the relocation is auditable.
// -------------------------------------------------------------------------
logic                              rx_byte_valid;
logic [7:0]                        rx_byte_val;

logic [rx_msg_pkg::BYTE_CNT_W-1:0] rx_expected_len;
rx_msg_pkg::msg_kind_t             rx_msg_kind_prov;   // provisional - rx_mac only
logic                              rx_kind_known;      // observability / TB use
logic [127:0]                      rx_frame_buf;       // LIVE buffer tap
logic [rx_msg_pkg::BYTE_CNT_W-1:0] rx_byte_cnt;        // LIVE byte count

rx_phy u_rx_phy (
    .clk              (clk),
    .rst_n            (rst_n),
    .rx_in            (rx_in),
    .byte_valid       (rx_byte_valid),
    .rx_byte          (rx_byte_val),
    .parity_err_pulse (rx_parity_err_pulse),
    .rx_busy          (rx_phy_busy)
);

// Combinational frame-length / kind decoder. bypass_active is driven live
// by rx_classifier's burst countdown: while a burst write is in flight it
// forces every incoming frame to decode as MSG_BURST_DATA, which is what
// makes raw pixel bytes at byte 1 safe.
rx_msg_decode u_rx_msg_decode (
    .frame_buf     (rx_frame_buf),
    .byte_cnt      (rx_byte_cnt),
    .bypass_active (bypass_active),
    .expected_len  (rx_expected_len),
    .msg_kind_prov (rx_msg_kind_prov),
    .kind_known    (rx_kind_known)
);

rx_mac u_rx_mac (
    .clk           (clk),
    .rst_n         (rst_n),
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

endmodule : uart_rx_subsystem
