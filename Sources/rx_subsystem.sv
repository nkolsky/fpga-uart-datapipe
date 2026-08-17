// rx_subsystem.sv
// ---------------
// Complete 256 MHz receive path: serial line in, ONE message out.
//
//   rx_in -> rx_phy -> rx_mac -> rx_msg_parser -> rx_classifier -> msg out
//                        ^            |                  |
//                        |            +-> rx_burst_ctrl --+
//                        |                     |
//                        |                     +-> bypass_active
//                        +---- stall ----------------+
//
// -----------------------------------------------------------------------
// WHO OWNS WHICH BYTE
// -----------------------------------------------------------------------
//   rx_mac         bytes 0, 5, 10, 15 -- '{', ',' and '}'. Framing only.
//   rx_msg_parser  bytes 1, 6, 11 -- opcodes. Extracts 2-4, 7-9, 12-14.
//   rx_classifier  no bytes at all. Range-checks the extracted fields and
//                  packs one message.
//
// -----------------------------------------------------------------------
// ONE MESSAGE OUT, NOT SIX COMMAND CHANNELS
// -----------------------------------------------------------------------
// This module used to expose six command channels -- legacy RGF, pixel
// write, register read, register write, pixel read, burst read -- plus a
// 48-bit command FIFO write port, each with its own crossing in chip_top.
// They are now one:
//
//     msg_valid / msg_ready / msg_kind / msg_payload
//
// Every message frame crosses, in order, on one interface. Burst data frames
// cross as data rather than being unpacked here into pixel commands: the
// memory side owns addresses because it owns the geometry.
//
// -----------------------------------------------------------------------
// BACK-PRESSURE RUNS THE WHOLE WAY BACK
// -----------------------------------------------------------------------
// The memory side can block for far longer than the gap between messages --
// mem_interlock gives writes to reads, and an image burst read is UART
// transmitter rate limited. So every stage waits:
//
//     msg_ready low -> classifier holds its message -> out_busy
//                   -> rx_mac stalls a completed frame in its buffer
//                   -> mac_busy stays high -> UART_CTS -> PC pauses
//
// Each link is ONE REGISTER DEEP. No queue is needed anywhere, because UART
// with RTS/CTS is a stallable source. The old design instead had a 16-deep
// FIFO and cmd_ovf_sticky on LED[13] to report when it overflowed.
//
// -----------------------------------------------------------------------
// SINGLE CLOCK DOMAIN -- 256 MHz
// -----------------------------------------------------------------------
// Everything here runs on pll_clk_out and resets from sync_pll_rst_n. There
// is NO clock-domain crossing inside this module and none may ever be added:
// every crossing in this design lives at chip_top, deliberately, so the whole
// CDC map is visible in one file.

`timescale 1ns/1ps

module rx_subsystem
    import msg_format_pkg::*;
(
    // ---- 256 MHz UART domain -------------------------------------------
    input  logic                 clk,             // pll_clk_out
    input  logic                 rst_n,           // sync_pll_rst_n

    // ---- serial input ---------------------------------------------------
    input  logic                 rx_in,           // UART_TXD_IN

    // ---- message out, towards cdc_msg_sync ------------------------------
    output logic                 msg_valid,
    input  logic                 msg_ready,
    output msg_kind_t            msg_kind,
    output msg_payload_t         msg_payload,

    // ---- status ----------------------------------------------------------
    output logic                 rx_phy_busy,
    output logic                 rx_parity_err_pulse,
    output logic                 rx_mac_busy,
    output logic                 rx_burst_active,
    output logic                 rx_classifier_error,

    // ---- diagnostics -----------------------------------------------------
    output logic                 pix_wr_seen_sticky,
    output logic                 burst_err_sticky
);

// -------------------------------------------------------------------------
// Internal interconnect
// -------------------------------------------------------------------------
logic [7:0]            rx_byte_val;
logic                  rx_byte_valid;

logic [FRAME_W-1:0]    rx_frame_buf;    // live tap AND completed frame
logic [BYTE_CNT_W-1:0] rx_byte_cnt;
logic                  rx_frame_done;
logic                  rx_frame_err;

msg_kind_t             rx_msg_kind;
logic [PAYLOAD_W-1:0]  rx_field0, rx_field1, rx_field2;
logic [BURST_PIX_PER_MSG-1:0][BURST_PIX_W-1:0] rx_burst_pixels;

logic                  rx_bypass_active;
logic                  rx_hdr_accept;
// Dimensions travel WITH hdr_accept out of the classifier: they are
// registered there, so taking them from the parser would give rx_burst_ctrl
// values that had already moved on.
logic [9:0]            rx_hdr_height, rx_hdr_width;
logic                  rx_out_busy;
logic                  rx_burst_done;

// =========================================================================
// PHY -- bit recovery, parity, byte assembly. No protocol knowledge.
// =========================================================================
rx_phy u_rx_phy (
    .clk              (clk),
    .rst_n            (rst_n),
    .rx_in            (rx_in),
    .byte_valid       (rx_byte_valid),
    .rx_byte          (rx_byte_val),
    .parity_err_pulse (rx_parity_err_pulse),
    .rx_busy          (rx_phy_busy)
);

// =========================================================================
// MAC -- framing. Owns bytes 0, 5, 10 and 15.
//
// stall holds a COMPLETED frame in the buffer rather than announcing it, so
// the classifier's held message cannot be overwritten.
// =========================================================================
rx_mac u_rx_mac (
    .clk         (clk),
    .rst_n       (rst_n),
    .byte_valid  (rx_byte_valid),
    .rx_byte     (rx_byte_val),
    .par_val_rst (rx_parity_err_pulse),
    .stall       (rx_out_busy),
    .frame_buf   (rx_frame_buf),
    .byte_cnt    (rx_byte_cnt),
    .frame_done  (rx_frame_done),
    .frame_err   (rx_frame_err),
    .mac_busy    (rx_mac_busy)
);

// =========================================================================
// PARSER -- identification. Owns bytes 1, 6 and 11; extracts the payload.
//
// Combinational on the live buffer, so its outputs are only meaningful while
// rx_frame_done is high. Every consumer below qualifies on that.
// =========================================================================
rx_msg_parser u_rx_msg_parser (
    .frame_buf     (rx_frame_buf),
    .byte_cnt      (rx_byte_cnt),
    .bypass_active (rx_bypass_active),
    .msg_kind      (rx_msg_kind),
    .field0        (rx_field0),
    .field1        (rx_field1),
    .field2        (rx_field2),
    .burst_pixels  (rx_burst_pixels)
);

// =========================================================================
// CLASSIFIER -- range checks, payload packing, and the message register.
// =========================================================================
rx_classifier u_rx_classifier (
    .clk              (clk),
    .rst_n            (rst_n),

    .msg_valid        (rx_frame_done),
    .frame_err        (rx_frame_err),
    .msg_kind         (rx_msg_kind),
    .field0           (rx_field0),
    .field1           (rx_field1),
    .field2           (rx_field2),
    .burst_pixels     (rx_burst_pixels),

    .out_valid        (msg_valid),
    .out_ready        (msg_ready),
    .out_kind         (msg_kind),
    .out_payload      (msg_payload),
    .out_busy         (rx_out_busy),
    .hdr_accept       (rx_hdr_accept),
    .hdr_height       (rx_hdr_height),
    .hdr_width        (rx_hdr_width),

    .classifier_error (rx_classifier_error)
);

// =========================================================================
// BURST CONTROLLER -- burst mode only.
//
// Arms on a header the classifier has already validated, counts pixels down,
// and drops bypass after the last data frame. It generates no addresses:
// pixel_word_packer on the memory side does that, because that is where the
// image geometry lives.
// =========================================================================
rx_burst_ctrl u_rx_burst_ctrl (
    .clk            (clk),
    .rst_n          (rst_n),
    .msg_valid      (rx_frame_done),
    .msg_kind       (rx_msg_kind),
    .hdr_accept     (rx_hdr_accept),
    .height         (BURST_DIM_W'(rx_hdr_height)),
    .width          (BURST_DIM_W'(rx_hdr_width)),
   // .burst_abort    (1'b0),
    .bypass_active  (rx_bypass_active),
    .burst_active   (rx_burst_active),
    .burst_done     (rx_burst_done),
    .err_unexpected (burst_err_sticky)
);

// =========================================================================
// DIAGNOSTICS
//
// cmd_ovf_sticky is gone with the FIFO. Overflow was its failure mode; with
// back-pressure the design stalls instead, so there is nothing to report.
// =========================================================================
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        pix_wr_seen_sticky <= 1'b0;
    else if (msg_valid && msg_ready && (msg_kind == MSG_PIX_WRITE))
        pix_wr_seen_sticky <= 1'b1;
end

`ifndef SYNTHESIS
    // The stall contract: rx_mac must never announce a frame while the
    // classifier is holding one. This is the property the whole
    // back-pressure chain rests on.
    a_no_frame_while_held: assert property (
        @(posedge clk) disable iff (!rst_n)
        rx_out_busy |-> !rx_frame_done
    ) else $error("%m: frame announced while a message was still held");

    // Burst data only ever appears while a burst is armed.
    a_data_needs_burst: assert property (
        @(posedge clk) disable iff (!rst_n)
        (rx_frame_done && (rx_msg_kind == MSG_BURST_DATA)) |-> rx_burst_active
    ) else $error("%m: burst data frame with no burst armed");
`endif

endmodule : rx_subsystem
