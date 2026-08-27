// -----------------------------------------------------------------------------
// chip_top.sv
// Top-level wrapper for the RGB image transport and register/control path.
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

// Message crossing, 130 MHz side and 100 MHz side.
logic                            rx_msg_valid, rx_msg_ready;
msg_format_pkg::msg_kind_t       rx_msg_kind;
msg_format_pkg::msg_payload_t    rx_msg_payload;

logic                            mem_msg_valid, mem_msg_ready;
msg_format_pkg::msg_kind_t       mem_msg_kind;
msg_format_pkg::msg_payload_t    mem_msg_payload;

logic                            burst_err_sticky;

// Register file commands, routed on the memory side from the message stream.
logic        mem_rgf_cmd_valid;
logic        mem_rgf_cmd_is_write;
logic [7:0]  mem_rgf_cmd_addr;
logic [31:0] mem_rgf_cmd_wdata;

logic                  sram_wr_seen;
logic                  sram_wr_rejected;
logic                  img_fifo_ovf_sticky;
logic                  burst_active_100;

logic                  rx_rd_reply_valid;
logic [31:0]           rx_rd_reply_data;

// -----------------------------------------------------------------------------
// Image FIFO interconnect
// -----------------------------------------------------------------------------
// PER CHANNEL. A burst fills one channel at a time, so exactly one of these
// is high on any beat.
logic [2:0]  fifo_wr_en;
// One shared write-data bus: a returned beat belongs to exactly one channel,
// and fifo_wr_en says which.
logic [31:0] fifo_wr_data;
logic        almost_full;       // image FIFO -> rom_sequencer backpressure
logic        almost_empty;      // image FIFO -> rom_sequencer resume signal
logic        fifo_rd_en;        // tx_sequencer -> image FIFO read enable
logic [31:0] fifo_rd_data_r;
logic [31:0] fifo_rd_data_g;
logic [31:0] fifo_rd_data_b;
logic        fifo_empty;        // image FIFO -> tx_sequencer empty flag
logic        fifo_full;         // image FIFO full flag (monitored, not used for control)
// G and B mirror R exactly at this step -- see the lockstep assertions.
logic        fifo_full_g,      fifo_full_b;
logic        fifo_empty_g,     fifo_empty_b;
logic        almost_full_g,    almost_full_b;
logic        almost_empty_g,   almost_empty_b;
logic        fifo_empty_cdc_g, fifo_empty_cdc_b;
logic        seq_done;          // rom_sequencer → chip_top: one-cycle done pulse
logic        rom_seq_busy;

// -----------------------------------------------------------------------------
// Image FIFO status CDC (130 MHz -> 100 MHz)
// -----------------------------------------------------------------------------
logic almost_empty_100;
logic fifo_empty_100;
// Registered copy of fifo_empty, from the FIFO's read domain. Only the
// crossing uses it -- see the note on async_fifo's empty_cdc port.
logic fifo_empty_cdc;

cdc_level_sync u_cdc_almost_empty (
    .src_level (almost_empty),      // pll_clk_out domain
    .dst_clk   (CLK100MHZ),
    .dst_rst_n (sync_rst_n),
    .dst_level (almost_empty_100)
);

cdc_level_sync u_cdc_fifo_empty (
    .src_level (fifo_empty_cdc),    // pll_clk_out domain, REGISTERED
    .dst_clk   (CLK100MHZ),
    .dst_rst_n (sync_rst_n),
    .dst_level (fifo_empty_100)
);

// -----------------------------------------------------------------------------
// MEMORY SUBSYSTEM -- 100 MHz
// -----------------------------------------------------------------------------
logic [43:0] pix_rpy_payload_100;
logic [95:0] brd_msg_payload_100;

memory_subsystem u_memory_subsystem (
    .clk                   (CLK100MHZ),
    .rst_n                 (sync_rst_n),

    .start_req             (start_pulse),
    .img_done              (tx_img_done_100),
    .burst_active          (burst_active_100),

    .img_fifo_almost_full  ({almost_full_b, almost_full_g, almost_full}),
    .img_fifo_almost_empty (almost_empty_100),
    .img_fifo_full         ({fifo_full_b, fifo_full_g, fifo_full}),
    .img_fifo_wr_en        (fifo_wr_en),
    .img_fifo_wr_data      (fifo_wr_data),

    .msg_valid             (mem_msg_valid),
    .msg_ready             (mem_msg_ready),
    .msg_kind              (mem_msg_kind),
    .msg_payload           (mem_msg_payload),

    .pix_rpy_accept        (pix_rpy_accept_100),
    .pix_rpy_send          (pix_rpy_send),
    .pix_rpy_payload       (pix_rpy_payload_100),

    .brd_msg_accept        (brd_msg_accept_100),
    .brd_msg_send          (brd_msg_send),
    .brd_msg_payload       (brd_msg_payload_100),

    .rgf_cmd_valid         (mem_rgf_cmd_valid),
    .rgf_cmd_is_write      (mem_rgf_cmd_is_write),
    .rgf_cmd_addr          (mem_rgf_cmd_addr),
    .rgf_cmd_wdata         (mem_rgf_cmd_wdata),
    .apb_busy              (apb_busy),

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
// THREE FIFOS, ONE PER COLOUR CHANNEL.
//
// This was a single 24-bit FIFO carrying packed pixels. It is three 32-bit
// FIFOs carrying whole SRAM words because, at step 6b, a per-channel INCR4
// burst delivers four words of R, then four of G, then four of B -- so the
// channels arrive at different times and each needs its own buffer.
//
// At THIS step the SRAM reads are still parallel, so all three are written
// and popped in the same cycle and hold identical occupancy. Only R's flags
// drive control; the assertions below check the other two agree, which is
// precisely what stops being true once bursts arrive per channel.
//
// Same total storage as the single FIFO: 3 x 16 x 32 = 1536 bits, 64 pixels.
async_fifo u_fifo_r (
    .wr_clk      (CLK100MHZ),
    .wr_rst_n    (sync_rst_n),
    .wr_en       (fifo_wr_en[0]),
    .wr_data     (fifo_wr_data),
    .full        (fifo_full),
    .almost_full (almost_full),
    .rd_clk      (pll_clk_out),
    .rd_rst_n    (sync_pll_rst_n),
    .rd_en       (fifo_rd_en),
    .rd_data     (fifo_rd_data_r),
    .empty       (fifo_empty),
    .almost_empty(almost_empty),
    .empty_cdc   (fifo_empty_cdc)
);

async_fifo u_fifo_g (
    .wr_clk      (CLK100MHZ),
    .wr_rst_n    (sync_rst_n),
    .wr_en       (fifo_wr_en[1]),
    .wr_data     (fifo_wr_data),
    .full        (fifo_full_g),
    .almost_full (almost_full_g),
    .rd_clk      (pll_clk_out),
    .rd_rst_n    (sync_pll_rst_n),
    .rd_en       (fifo_rd_en),
    .rd_data     (fifo_rd_data_g),
    .empty       (fifo_empty_g),
    .almost_empty(almost_empty_g),
    .empty_cdc   (fifo_empty_cdc_g)
);

async_fifo u_fifo_b (
    .wr_clk      (CLK100MHZ),
    .wr_rst_n    (sync_rst_n),
    .wr_en       (fifo_wr_en[2]),
    .wr_data     (fifo_wr_data),
    .full        (fifo_full_b),
    .almost_full (almost_full_b),
    .rd_clk      (pll_clk_out),
    .rd_rst_n    (sync_pll_rst_n),
    .rd_en       (fifo_rd_en),
    .rd_data     (fifo_rd_data_b),
    .empty       (fifo_empty_b),
    .almost_empty(almost_empty_b),
    .empty_cdc   (fifo_empty_cdc_b)
);

`ifndef SYNTHESIS
// THE CHANNELS ARE EXPECTED TO DIVERGE NOW.
//
// Under the parallel read all three FIFOs were written in the same cycle and
// held identical occupancy -- there were lockstep assertions here saying so.
// A per-channel INCR4 burst fills one channel at a time, so red runs up to
// two bursts ahead of blue. Checking for lockstep would now fire constantly.
//
// What still must hold is that exactly one channel is written per beat, and
// that no channel is written while full.
a_fifo_wr_onehot: assert property (
    @(posedge CLK100MHZ) disable iff (!sync_rst_n)
    $onehot0(fifo_wr_en)
) else $error("chip_top: more than one channel FIFO written in a cycle");

a_fifo_no_overflow: assert property (
    @(posedge CLK100MHZ) disable iff (!sync_rst_n)
    !(|(fifo_wr_en & {fifo_full_b, fifo_full_g, fifo_full}))
) else $error("chip_top: a channel FIFO was written while full");
`endif

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

// UART_RTS is the host's RTS pin: asynchronous to pll_clk_out, since the
// FTDI runs from its own crystal. Sampled directly it can violate setup or
// hold on a transmit-path flop and go metastable, and an invalid level fanned
// out to several gates can be read as 0 by one and 1 by another -- which puts
// the transmit state machine into a state its own logic says cannot exist.
// Two flops give the first one a full clock period, with nothing downstream,
// to settle before the second samples it.
//
// Reset value is 1 = deasserted = hold off, so the transmitter never starts
// before the real level has been observed.
// ASYNC_REG marks these as synchroniser stages: keeps them in adjacent slices
// so the first has the full clock period to settle, and stops synthesis
// retiming or merging them. The chain itself is unchanged.
(* ASYNC_REG = "TRUE" *) logic cts_meta, cts_sync;
always_ff @(posedge pll_clk_out or negedge sync_pll_rst_n) begin
    if (!sync_pll_rst_n) {cts_sync, cts_meta} <= 2'b11;
    else                 {cts_sync, cts_meta} <= {cts_meta, UART_RTS};
end

uart_tx_subsystem u_uart_tx_subsystem (
    .clk                (pll_clk_out),
    .rst_n              (sync_pll_rst_n),

    .fifo_empty         (fifo_empty),
    .fifo_rd_data_r     (fifo_rd_data_r),
    .fifo_rd_data_g     (fifo_rd_data_g),
    .fifo_rd_data_b     (fifo_rd_data_b),
    .fifo_rd_en         (fifo_rd_en),
    .cts                (cts_sync),

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

logic        rx_classifier_error;


logic        pix_wr_seen_sticky;

// -----------------------------------------------------------------------------
// RX subsystem (130 MHz)
//
// This block receives UART data, decodes frames, and emits one message at a
// time for the memory side.
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
// RX messages are handed to the memory side with back-pressure. The source
// stalls when the destination is busy, so the receiver does not silently drop
// commands or frames.
//
// The CDC data path inside cdc_msg_sync needs a false-path or max-delay
// constraint in the XDC.
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
// Single-pixel reads are carried as messages and ordered with the rest of the
// request stream.

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
// Burst requests are also sent as messages, then dispatched by the memory-side
// router.

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
// Register reads/writes arrive as messages and are decoded on the memory side
// before driving the register subsystem.

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
// APB fabric nets. The fabric is interconnect, so it lives here with the CDC
// primitives rather than inside a subsystem.
logic                           apb_busy;
logic                           apb_rsp_valid;
logic                           apb_rsp_is_read;
logic [apb_pkg::DATA_W-1:0]     apb_rsp_rdata;
logic                           apb_rsp_error;

logic                           apb_active;
logic                           apb_penable;
logic                           apb_pwrite;
logic [apb_pkg::ADDR_W-1:0]     apb_paddr;
logic [apb_pkg::DATA_W-1:0]     apb_pwdata;
logic [apb_pkg::STRB_W-1:0]     apb_pstrb;
logic                           apb_pready;
logic [apb_pkg::DATA_W-1:0]     apb_prdata;
logic                           apb_pslverr;

logic [apb_pkg::NUM_SLAVES-1:0] apb_psel;
logic [apb_pkg::NUM_SLAVES-1:0] apb_s_pready;
logic [apb_pkg::NUM_SLAVES-1:0] apb_s_pslverr;
logic [apb_pkg::DATA_W-1:0]     apb_s_prdata [apb_pkg::NUM_SLAVES];
logic                           apb_decode_err;
logic                           apb_decode_err_sticky;

// Driven from apb_master's response now, not from register_subsystem.
logic        rgf_rd_strobe;
logic [31:0] rgf_rd_value;
logic        rgf_start_img_read;

// The register path is fed from the message stream after routing on the memory
// side.
register_subsystem u_register_subsystem (
    .clk               (CLK100MHZ),
    .rst_n             (sync_rst_n),

    // APB slave port. The command group that used to arrive here is turned
    // into a bus transfer by apb_master below.
    .psel              (apb_psel[apb_pkg::SLAVE_RGF]),
    .penable           (apb_penable),
    .pwrite            (apb_pwrite),
    .paddr             (apb_paddr),
    .pwdata            (apb_pwdata),
    .pready            (apb_s_pready[apb_pkg::SLAVE_RGF]),
    .prdata            (apb_s_prdata[apb_pkg::SLAVE_RGF]),
    .pslverr           (apb_s_pslverr[apb_pkg::SLAVE_RGF]),

    .tx_img_done       (tx_img_done_100),
    .tx_row            (tx_row),
    .tx_col            (tx_col),
    .parity_fault_incr (rx_parity_err_100),
    .fifo_full         (fifo_full),
    .fifo_empty        (fifo_empty_100),
    .fifo_almost_full  (almost_full),
    .fifo_almost_empty (almost_empty_100),

    .start_img_read    (rgf_start_img_read),
    .clk_sel           (clk_sel)
);

// -----------------------------------------------------------------------------
// APB fabric (100 MHz)
// -----------------------------------------------------------------------------
// Instantiated bare, alongside the CDC primitives above: chip_top is where this
// design keeps its interconnect, and an APB fabric is interconnect. The slave
// sits with rgf inside register_subsystem because it translates to rgf's
// private pc_* port.
//
// Entirely within CLK100MHZ -- the bus introduces NO new clock crossing. The
// register reply still leaves on the same cdc_cmd_sync it always did.
apb_master u_apb_master (
    .clk         (CLK100MHZ),
    .rst_n       (sync_rst_n),

    .req_valid   (mem_rgf_cmd_valid),
    .req_write   (mem_rgf_cmd_is_write),
    .req_addr    (apb_pkg::addr_from_msg(mem_rgf_cmd_addr)),
    .req_wdata   (mem_rgf_cmd_wdata),
    .busy        (apb_busy),

    .rsp_valid   (apb_rsp_valid),
    .rsp_is_read (apb_rsp_is_read),
    .rsp_rdata   (apb_rsp_rdata),
    .rsp_error   (apb_rsp_error),

    .m_active    (apb_active),
    .m_penable   (apb_penable),
    .m_pwrite    (apb_pwrite),
    .m_paddr     (apb_paddr),
    .m_pwdata    (apb_pwdata),
    .m_pstrb     (apb_pstrb),
    .m_pready    (apb_pready),
    .m_prdata    (apb_prdata),
    .m_pslverr   (apb_pslverr)
);

apb_bar u_apb_bar (
    .m_active   (apb_active),
    .m_penable  (apb_penable),
    .m_paddr    (apb_paddr),
    .m_pready   (apb_pready),
    .m_prdata   (apb_prdata),
    .m_pslverr  (apb_pslverr),
    .psel       (apb_psel),
    .s_pready   (apb_s_pready),
    .s_pslverr  (apb_s_pslverr),
    .s_prdata   (apb_s_prdata),
    .decode_err (apb_decode_err)
);

// Only a READ produces a reply on the wire. A write still completes and still
// reports PSLVERR, but the host is not waiting for anything.
assign rgf_rd_strobe = apb_rsp_valid && apb_rsp_is_read;
assign rgf_rd_value  = apb_rsp_rdata;

// An unmapped aperture would otherwise be invisible. Sticky, so one bad
// address is still visible on the board afterwards.
always_ff @(posedge CLK100MHZ or negedge sync_rst_n) begin
    if (!sync_rst_n)                          apb_decode_err_sticky <= 1'b0;
    else if (apb_decode_err || apb_rsp_error) apb_decode_err_sticky <= 1'b1;
end

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
// The transmit path stalls while the memory side is busy, the TX sequencer is
// active, the RX MAC is busy, or a reply is waiting for its single-slot output
// buffer.
assign UART_CTS = (rom_seq_busy || tx_seq_busy || rx_mac_busy ||
                   rr_reply_pending);

// -----------------------------------------------------------------------------
// LEDs
// -----------------------------------------------------------------------------
assign LED[14] = tx_done_sticky;
assign LED[13] = sram_wr_rejected || burst_err_sticky || apb_decode_err_sticky;
assign LED[15] = img_fifo_ovf_sticky;
assign LED[12] = heartbeat;
assign LED[11] = clk_sel;
assign LED[10] = UART_RTS;
assign LED[9] = UART_CTS;
assign LED[8] = start_pulse;
assign LED[7] = rx_phy_busy;
assign LED[6] = rx_classifier_error;
assign LED[5] = rx_msg_valid;
assign LED[4] = rx_mac_busy;
assign LED[3] = ~fifo_empty;
assign LED[2] = tx_seq_busy;
assign LED[0] = tx_activity_led;
assign LED[1] = rom_seq_busy;
endmodule : chip_top