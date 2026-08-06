// -----------------------------------------------------------------------------
// chip_top.sv
// Top-level wrapper for the full-duplex RGB image data exchange system.
// -----------------------------------------------------------------------------

import uart_pkg::*;
import rgf_pkg::*;

module chip_top (
    input  logic        CLK100MHZ,      // 100 MHz clock
    input  logic        CPU_RESETN,     // active-low, async, from push button

    // UART interface
    input  logic        UART_TXD_IN,    // PC to FPGA
    input  logic        UART_RTS,       // active-low flow-control input
    output logic        UART_RXD_OUT,   // FPGA to PC
    output logic        UART_CTS,       // active-low flow-control output

    output logic [15:0] LED
);

// -----------------------------------------------------------------------------
// CLOCKING SUBSYSTEM
// -----------------------------------------------------------------------------
logic sync_rst_n;
logic pll_clk_out;
logic sync_pll_rst_n;
logic pll_locked;
logic clk_sel;              // driven by register_subsystem below
logic heartbeat;

clocking_subsystem u_clocking_subsystem (
    .clk_100_in   (CLK100MHZ),
    .async_reset_n(CPU_RESETN),
    .clk_sel      (clk_sel),
    .clk_130_out  (pll_clk_out),
    .rst_100_n    (sync_rst_n),
    .rst_130_n    (sync_pll_rst_n),
    .pll_locked   (pll_locked),
    .heartbeat    (heartbeat)
);

logic start_pulse;
logic tx_img_done_100;

// -----------------------------------------------------------------------------
// Read-command and reply interconnect
// -----------------------------------------------------------------------------

// Single-pixel read
logic        rx_pr_cmd_valid;
logic [9:0]  rx_pr_cmd_row, rx_pr_cmd_col;

logic        pix_req_valid_100;
logic [19:0] pix_req_data_100;      // {row[9:0], col[9:0]}, atomic
logic        pix_rpy_accept_100;
logic        pix_rpy_send;
logic        pix_rd_busy, pix_rd_overrun;

logic        pix_rpy_valid_130;     // one-cycle strobe
logic [43:0] pix_rpy_data_130;      // {row, col, pixel}, coherent
logic        pix_rpy_accept_130;

// Image-burst read
logic        rx_br_cmd_valid;
logic [9:0]  rx_br_cmd_base_row, rx_br_cmd_base_col;
logic [9:0]  rx_br_cmd_height,   rx_br_cmd_width;

logic        brd_req_valid_100;
logic [39:0] brd_req_data_100;     // {base_row, base_col, height, width}
logic        brd_msg_accept_100;
logic        brd_msg_send;
logic        brd_busy, brd_overrun;

logic        brd_msg_valid_130;
logic [95:0] brd_msg_data_130;
logic        brd_msg_accept_130;

// -----------------------------------------------------------------------------
// FIFO, status and register-reply interconnect
// -----------------------------------------------------------------------------
localparam int CMD_FIFO_W = 48;

// Message crossing, 130 MHz side and 100 MHz side.
logic                            rx_msg_valid, rx_msg_ready;
msg_format_pkg::msg_kind_t       rx_msg_kind;
msg_format_pkg::msg_payload_t    rx_msg_payload;

logic                            mem_msg_valid, mem_msg_ready;
msg_format_pkg::msg_kind_t       mem_msg_kind;
msg_format_pkg::msg_payload_t    mem_msg_payload;

logic                            burst_err_sticky;

logic                  sram_wr_seen;
logic                  sram_wr_rejected;
logic                  img_fifo_ovf_sticky;
logic                  burst_active_100;

logic                  rx_rd_reply_valid;
logic [31:0]           rx_rd_reply_data;

// -----------------------------------------------------------------------------
// Image FIFO interconnect
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
logic        rom_seq_busy;

// -----------------------------------------------------------------------------
// Image FIFO status CDC (130 MHz -> 100 MHz)
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
// MEMORY SUBSYSTEM -- 100 MHz
// -----------------------------------------------------------------------------
logic [43:0] pix_rpy_payload_100;
logic [95:0] brd_msg_payload_100;

memory_subsystem #(
    .CMD_W (CMD_FIFO_W)
) u_memory_subsystem (
    .clk                   (CLK100MHZ),
    .rst_n                 (sync_rst_n),

    .start_req             (start_pulse),
    .img_done              (tx_img_done_100),
    .burst_active          (burst_active_100),

    .img_fifo_almost_full  (almost_full),
    .img_fifo_almost_empty (almost_empty_100),
    .img_fifo_full         (fifo_full),
    .img_fifo_wr_en        (fifo_wr_en),
    .img_fifo_wr_data      (fifo_wr_data),

    .msg_valid             (mem_msg_valid),
    .msg_ready             (mem_msg_ready),
    .msg_kind              (mem_msg_kind),
    .msg_payload           (mem_msg_payload),

    .pix_req_valid         (pix_req_valid_100),
    .pix_req_data          (pix_req_data_100),
    .pix_rpy_accept        (pix_rpy_accept_100),
    .pix_rpy_send          (pix_rpy_send),
    .pix_rpy_payload       (pix_rpy_payload_100),

    .brd_req_valid         (brd_req_valid_100),
    .brd_req_data          (brd_req_data_100),
    .brd_msg_accept        (brd_msg_accept_100),
    .brd_msg_send          (brd_msg_send),
    .brd_msg_payload       (brd_msg_payload_100),

    .seq_done              (seq_done),
    .rom_seq_busy          (rom_seq_busy),
    .sram_wr_seen          (sram_wr_seen),
    .sram_wr_rejected      (sram_wr_rejected),
    .pix_rd_busy           (pix_rd_busy),
    .pix_rd_overrun        (pix_rd_overrun),
    .brd_busy              (brd_busy),
    .brd_overrun           (brd_overrun),
    .img_fifo_ovf_sticky   (img_fifo_ovf_sticky)
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
    .rd_clk      (pll_clk_out),
    .rd_rst_n    (sync_pll_rst_n),
    .rd_en       (fifo_rd_en),
    .rd_data     (fifo_rd_data),
    .empty       (fifo_empty),
    .almost_empty(almost_empty)
);

// -----------------------------------------------------------------------------
// UART transmit subsystem (130 MHz)
// -----------------------------------------------------------------------------
logic        mac_busy;
logic        tx_img_done;
logic        tx_seq_busy;
logic [9:0]  tx_row;
logic [9:0]  tx_col;
logic        tx_done_sticky;
logic        tx_activity_led;
logic        rr_reply_pending;
logic        rr_reply_overrun;

uart_tx_subsystem u_uart_tx_subsystem (
    .clk                (pll_clk_out),
    .rst_n              (sync_pll_rst_n),

    .fifo_empty         (fifo_empty),
    .fifo_rd_data       (fifo_rd_data),
    .fifo_rd_en         (fifo_rd_en),
    .cts                (UART_RTS),

    .rd_reply_valid     (rx_rd_reply_valid),
    .rd_reply_data      (rx_rd_reply_data),

    .pix_reply_valid    (pix_rpy_valid_130),
    .pix_reply_data     (pix_rpy_data_130),
    .pix_reply_accept   (pix_rpy_accept_130),

    .burst_reply_valid  (brd_msg_valid_130),
    .burst_reply_data   (brd_msg_data_130),
    .burst_reply_accept (brd_msg_accept_130),

    .uart_tx            (UART_RXD_OUT),
    .tx_activity_led    (tx_activity_led),
    .mac_busy           (mac_busy),
    .tx_img_done        (tx_img_done),
    .tx_seq_busy        (tx_seq_busy),
    .tx_row             (tx_row),
    .tx_col             (tx_col),
    .tx_done_sticky     (tx_done_sticky),
    .reply_pending      (rr_reply_pending),
    .reply_overrun      (rr_reply_overrun)
);

// -----------------------------------------------------------------------------
// RX subsystem interconnect (130 MHz)
// -----------------------------------------------------------------------------
logic        rx_phy_busy;
logic        rx_mac_busy;
logic        rx_parity_err_pulse;

logic        rx_burst_active;

logic        rx_classifier_valid;
logic        rx_classifier_error;
logic [9:0]  rx_row_q, rx_col_q;
logic [23:0] rx_pixel_q;

logic        rx_rr_cmd_valid;
logic [rgf_pkg::ADDR_WIDTH-1:0] rx_rr_cmd_addr;
logic        rx_rw_cmd_valid;
logic [rgf_pkg::ADDR_WIDTH-1:0] rx_rw_cmd_addr;
logic [rgf_pkg::DATA_WIDTH-1:0] rx_rw_cmd_data;

logic        pix_wr_seen_sticky;

// -----------------------------------------------------------------------------
// RX subsystem (130 MHz)
//
// Merged from uart_rx_subsystem + command_processing_subsystem. rx_mac_msg_valid,
// rx_mac_msg_data, rx_msg_kind_q and rx_bypass_active were only ever wires
// between those two modules and are now internal to this one.
// -----------------------------------------------------------------------------
rx_subsystem u_rx_subsystem (
    .clk                  (pll_clk_out),
    .rst_n                (sync_pll_rst_n),
    .rx_in                (UART_TXD_IN),

    .msg_valid            (rx_msg_valid),
    .msg_ready            (rx_msg_ready),
    .msg_kind             (rx_msg_kind),
    .msg_payload          (rx_msg_payload),

    .rx_phy_busy          (rx_phy_busy),
    .rx_parity_err_pulse  (rx_parity_err_pulse),
    .rx_mac_busy          (rx_mac_busy),
    .rx_burst_active      (rx_burst_active),
    .rx_classifier_error  (rx_classifier_error),

    .pix_wr_seen_sticky   (pix_wr_seen_sticky),
    .burst_err_sticky     (burst_err_sticky)
);

// -----------------------------------------------------------------------------
// MESSAGE CROSSING, 130 MHz -> 100 MHz
//
// Replaces the 16-deep asynchronous command FIFO. There was no rate mismatch
// to absorb: at 8.125 Mbaud a message takes ~2816 receive clocks and carries
// four pixels, against a memory side at 100 MHz. There was also nothing to
// buffer into -- the RX side holds no image data.
//
// The FIFO's real function was hiding a missing back-pressure path. It turned
// "lose the next command" into "lose the seventeenth", and cmd_ovf_sticky
// existed to report when that happened. This crossing stalls instead:
// msg_ready propagates to rx_classifier, rx_mac and UART_CTS.
//
// ~105 flops against roughly 800.
//
// XDC: the data register inside needs a false path or max-delay constraint.
// Without one the tool tries to close it as a single-cycle path between
// asynchronous clocks and reports a failure that is not real.
// -----------------------------------------------------------------------------
cdc_msg_sync #(
    .WIDTH ($bits(msg_format_pkg::msg_kind_t) +
            $bits(msg_format_pkg::msg_payload_t))
) u_msg_cdc (
    .src_clk   (pll_clk_out),
    .src_rst_n (sync_pll_rst_n),
    .src_valid (rx_msg_valid),
    .src_ready (rx_msg_ready),
    .src_data  ({rx_msg_kind, rx_msg_payload}),

    .dst_clk   (CLK100MHZ),
    .dst_rst_n (sync_rst_n),
    .dst_valid (mem_msg_valid),
    .dst_ready (mem_msg_ready),
    .dst_data  ({mem_msg_kind, mem_msg_payload})
);

// -----------------------------------------------------------------------------
// Burst-active CDC (130 MHz -> 100 MHz)
// -----------------------------------------------------------------------------

cdc_level_sync u_cdc_burst_active (
    .src_level (rx_burst_active),
    .dst_clk   (CLK100MHZ),
    .dst_rst_n (sync_rst_n),
    .dst_level (burst_active_100)
);

// -----------------------------------------------------------------------------
// Pixel-read request CDC (130 MHz -> 100 MHz)
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
// Pixel-read reply CDC (100 MHz -> 130 MHz)
// -----------------------------------------------------------------------------
cdc_cmd_sync #(
    .ADDR_W (1),
    .DATA_W (44)
) u_cdc_pix_rpy (
    .src_clk      (CLK100MHZ),
    .src_rst_n    (sync_rst_n),
    .src_valid    (pix_rpy_send),
    .src_is_write (1'b0),
    .src_addr     (1'b0),
    .src_wdata    (pix_rpy_payload_100),
    .dst_valid    (pix_rpy_valid_130),
    .dst_is_write (),
    .dst_addr     (),
    .dst_wdata    (pix_rpy_data_130),
    .dst_clk      (pll_clk_out),
    .dst_rst_n    (sync_pll_rst_n)
);

cdc_pulse_sync u_cdc_pix_rpy_accept (
    .src_clk   (pll_clk_out),
    .src_rst_n (sync_pll_rst_n),
    .src_pulse (pix_rpy_accept_130),
    .dst_clk   (CLK100MHZ),
    .dst_rst_n (sync_rst_n),
    .dst_pulse (pix_rpy_accept_100)
);

// -----------------------------------------------------------------------------
// Burst-read request CDC (130 MHz -> 100 MHz)
// -----------------------------------------------------------------------------
cdc_cmd_sync #(
    .ADDR_W (1),
    .DATA_W (40)
) u_cdc_brd_req (
    .src_clk      (pll_clk_out),
    .src_rst_n    (sync_pll_rst_n),
    .src_valid    (rx_br_cmd_valid),
    .src_is_write (1'b0),
    .src_addr     (1'b0),
    .src_wdata    ({rx_br_cmd_base_row, rx_br_cmd_base_col,
                    rx_br_cmd_height,   rx_br_cmd_width}),
    .dst_valid    (brd_req_valid_100),
    .dst_is_write (),
    .dst_addr     (),
    .dst_wdata    (brd_req_data_100),
    .dst_clk      (CLK100MHZ),
    .dst_rst_n    (sync_rst_n)
);

// -----------------------------------------------------------------------------
// Burst-read reply CDC (100 MHz -> 130 MHz)
// -----------------------------------------------------------------------------
cdc_cmd_sync #(
    .ADDR_W (1),
    .DATA_W (96)
) u_cdc_brd_msg (
    .src_clk      (CLK100MHZ),
    .src_rst_n    (sync_rst_n),
    .src_valid    (brd_msg_send),
    .src_is_write (1'b0),
    .src_addr     (1'b0),
    .src_wdata    (brd_msg_payload_100),
    .dst_valid    (brd_msg_valid_130),
    .dst_is_write (),
    .dst_addr     (),
    .dst_wdata    (brd_msg_data_130),
    .dst_clk      (pll_clk_out),
    .dst_rst_n    (sync_pll_rst_n)
);

cdc_pulse_sync u_cdc_brd_accept (
    .src_clk   (pll_clk_out),
    .src_rst_n (sync_pll_rst_n),
    .src_pulse (brd_msg_accept_130),
    .dst_clk   (CLK100MHZ),
    .dst_rst_n (sync_rst_n),
    .dst_pulse (brd_msg_accept_100)
);

// -----------------------------------------------------------------------------
// Register command path
// -----------------------------------------------------------------------------
logic        rgf_cmd_valid_100;
logic        rgf_cmd_is_write_100;
logic [7:0]  rgf_cmd_addr_100;
logic [31:0] rgf_cmd_wdata_100;

// Register-command mux (130 MHz)
logic        rgf_src_valid, rgf_src_is_write;
logic [7:0]  rgf_src_addr;
logic [31:0] rgf_src_wdata;

assign rgf_src_valid    = rx_classifier_valid || rx_rw_cmd_valid ||
                          rx_rr_cmd_valid;

assign rgf_src_is_write = rx_classifier_valid ? !rx_col_q[0]
                        : rx_rw_cmd_valid     ? 1'b1
                        :                       1'b0;

// rx_rw_cmd_addr / rx_rr_cmd_addr are now rgf_pkg::ADDR_WIDTH wide and need
// no padding. The legacy path still forms a word address from the row field.
assign rgf_src_addr     = rx_classifier_valid ? {rx_row_q[5:0], 2'b00}
                        : rx_rw_cmd_valid     ? rx_rw_cmd_addr
                        :                       rx_rr_cmd_addr;

assign rgf_src_wdata    = rx_classifier_valid ? {8'b0, rx_pixel_q}
                        : rx_rw_cmd_valid     ? rx_rw_cmd_data
                        :                       32'd0;

`ifndef SYNTHESIS
    a_one_rgf_producer_top: assert property (
        @(posedge pll_clk_out) disable iff (!sync_pll_rst_n)
        $onehot0({rx_classifier_valid, rx_rw_cmd_valid, rx_rr_cmd_valid})
    ) else $error("chip_top: more than one RGF command producer asserted");

    a_regwr_is_write: assert property (
        @(posedge pll_clk_out) disable iff (!sync_pll_rst_n)
        rx_rw_cmd_valid |-> (rgf_src_is_write && rgf_src_valid)
    ) else $error("chip_top: Register Write did not present a write command");
`endif

cdc_cmd_sync #(
    .ADDR_W    (8),
    .DATA_W    (32),
    .IDLE_ADDR (8'hFF)
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

// Event CDCs (130 MHz -> 100 MHz)
logic rx_parity_err_100;

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
// Register subsystem (100 MHz)
// -----------------------------------------------------------------------------
logic        rgf_rd_strobe;
logic [31:0] rgf_rd_value;
logic        rgf_start_img_read;

register_subsystem u_register_subsystem (
    .clk               (CLK100MHZ),
    .rst_n             (sync_rst_n),

    .cmd_valid         (rgf_cmd_valid_100),
    .cmd_is_write      (rgf_cmd_is_write_100),
    .cmd_addr          (rgf_cmd_addr_100),
    .cmd_wdata         (rgf_cmd_wdata_100),

    .tx_img_done       (tx_img_done_100),
    .tx_row            (tx_row),
    .tx_col            (tx_col),
    .parity_fault_incr (rx_parity_err_100),
    .fifo_full         (fifo_full),
    .fifo_empty        (fifo_empty_100),
    .fifo_almost_full  (almost_full),
    .fifo_almost_empty (almost_empty_100),

    .rd_strobe         (rgf_rd_strobe),
    .rd_value          (rgf_rd_value),
    .start_img_read    (rgf_start_img_read),
    .clk_sel           (clk_sel)
);

// Register-read reply CDC (100 MHz -> 130 MHz)
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

assign start_pulse = rgf_start_img_read;

// -----------------------------------------------------------------------------
// UART flow control
// -----------------------------------------------------------------------------
assign UART_CTS = (rom_seq_busy || tx_seq_busy || rx_mac_busy);

// -----------------------------------------------------------------------------
// LEDs
// -----------------------------------------------------------------------------
assign LED[14] = tx_done_sticky;
// cmd_ovf_sticky is gone with the command FIFO: overflow was its failure
// mode, and the crossing back-pressures instead of dropping. What remains
// worth reporting is a write addressed outside the image, and a burst data
// frame arriving with no burst armed.
assign LED[13] = sram_wr_rejected || burst_err_sticky;
assign LED[15] = img_fifo_ovf_sticky;
assign LED[12] = heartbeat;
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
assign LED[0] = tx_activity_led;
assign LED[1] = rom_seq_busy;
endmodule : chip_top