// -----------------------------------------------------------------------------
// memory_subsystem.sv
//
// 100 MHz memory domain. It routes message traffic, stores image data in the
// three SRAM banks, streams image reads, and owns the read/write arbitration
// for the shared memory port.
// -----------------------------------------------------------------------------
`timescale 1ns/1ps

module memory_subsystem (
    input  logic        clk,
    input  logic        rst_n,

    // Full-image request and completion.
    input  logic        start_req,
    input  logic        img_done,
    input  logic        burst_active,

    // Image FIFO write side. The FIFO itself remains in chip_top.
    // PER CHANNEL now. A burst fills one channel at a time, so a channel
    // whose FIFO is backing up drops out of the arbitration while the others
    // carry on -- it no longer halts the whole drain.
    input  logic [2:0]  img_fifo_almost_full,
    input  logic        img_fifo_almost_empty,
    input  logic [2:0]  img_fifo_full,
    output logic [2:0]  img_fifo_wr_en,
    // One 32-bit SRAM word per channel, written to three FIFOs in the same
    // cycle. This was a single 24-bit packed pixel; the unpacking moved to
    // tx_sequencer, where pixels are consumed one per message.
    // One shared data bus: a returned beat belongs to exactly one channel,
    // and img_fifo_wr_en says which.
    output logic [31:0] img_fifo_wr_data,

    // Message input. Messages arrive here in order and are dispatched by the
    // router. Back-pressure is carried back to the source.
    input  logic                        msg_valid,
    output logic                        msg_ready,
    input  msg_format_pkg::msg_kind_t   msg_kind,
    input  msg_format_pkg::msg_payload_t msg_payload,

    // Read requests arrive as messages and are routed by the message router.
    // Responses travel back on their own CDC paths.
    input  logic        pix_rpy_accept,
    output logic        pix_rpy_send,
    output logic [43:0] pix_rpy_payload,

    input  logic        brd_msg_accept,
    output logic        brd_msg_send,
    output logic [95:0] brd_msg_payload,

    // Register file commands, routed from the same message stream.
    output logic        rgf_cmd_valid,
    output logic        rgf_cmd_is_write,
    output logic [7:0]  rgf_cmd_addr,
    output logic [31:0] rgf_cmd_wdata,
    input  logic        apb_busy,

    // Status/diagnostics retained for top-level integration and LEDs.
    output logic        seq_done,
    output logic        rom_seq_busy,
    output logic        sram_wr_seen,
    output logic        sram_wr_rejected,
    output logic        pix_rd_busy,
    output logic        pix_rd_overrun,
    output logic        brd_busy,
    output logic        brd_overrun,
    output logic        img_fifo_ovf_sticky
);

logic [31:0] red_data;
logic [31:0] green_data;
logic [31:0] blue_data;

logic                                    sram_wr_en;
logic [memory_pkg::SRAM_DATA_WIDTH/8-1:0] sram_wr_be;
logic [memory_pkg::SRAM_ADDR_WIDTH-1:0]   sram_wr_addr;
logic [memory_pkg::SRAM_DATA_WIDTH-1:0]   sram_wr_data_r;
logic [memory_pkg::SRAM_DATA_WIDTH-1:0]   sram_wr_data_g;
logic [memory_pkg::SRAM_DATA_WIDTH-1:0]   sram_wr_data_b;
logic                                    sram_wr_busy;
logic                                    read_go;

logic        pix_sram_rd_en;
logic [memory_pkg::SRAM_ADDR_WIDTH-1:0] pix_sram_rd_addr;
logic        pix_rd_owner;
logic        pix_rd_req;
logic        pix_rd_gnt;
logic        pix_rd_done;
logic        pix_rpy_valid;
logic [9:0]  pix_rpy_row;
logic [9:0]  pix_rpy_col;
logic [23:0] pix_rpy_pixel;
logic        pix_rpy_valid_d;

logic        brd_rd_req;
logic        brd_rd_gnt;
logic        brd_rd_done;
logic        brd_sram_rd_en;
logic [memory_pkg::SRAM_ADDR_WIDTH-1:0] brd_sram_rd_addr;
logic        brd_msg_valid;
logic [rx_burst_pkg::BURST_PIX_PER_MSG-1:0]
      [rx_burst_pkg::BURST_PIX_W-1:0] brd_msg_pixels;
logic        brd_msg_valid_d;

logic shared_rd_req;
logic shared_rd_gnt;
logic shared_rd_done;
logic arb_locked;
logic arb_pix_owns;
logic arb_brd_owns;

// Write path
logic wr_port_grant;
logic wr_rect_busy;

// Router -> write path
logic                                wr_msg_valid, wr_msg_ready;
msg_format_pkg::msg_kind_t           wr_msg_kind;
msg_format_pkg::msg_payload_t        wr_msg_payload;

// Router -> read controllers
logic       pix_req_valid;
logic [9:0] pix_req_row, pix_req_col;
logic       brd_req_valid;
logic [9:0] brd_req_base_row, brd_req_base_col, brd_req_height, brd_req_width;

logic        sram_rd_en_mux;
logic [memory_pkg::SRAM_ADDR_WIDTH-1:0] sram_rd_addr_mux;

// The write path is NOT a read client: byte enables mean a partial-word
// update needs no read.
// pix_rd_owner already means "the pixel/burst readers own the port, not the
// full-image path". That is unchanged. What changed is what drives the port
// on the OTHER side: the three AHB slaves, one per channel, instead of one
// broadcast address from rom_sequencer.
//
// So each SRAM now needs its own read pins. Under the direct path all three
// still see the same address -- a single-pixel read wants R, G and B of one
// pixel, which is exactly why that path did not move to AHB.
assign sram_rd_en_mux   = pix_rd_owner
                        ? (arb_brd_owns ? brd_sram_rd_en   : pix_sram_rd_en)
                        : 1'b0;
assign sram_rd_addr_mux = pix_rd_owner
                        ? (arb_brd_owns ? brd_sram_rd_addr : pix_sram_rd_addr)
                        : '0;

// AHB-side nets the muxes below need. Declared HERE, ahead of first use --
// Vivado raises Synth 8-6901 for use-before-declaration even though it
// resolves them, and a wall of those warnings hides real ones.
logic                                    wr_burst_active;
logic [2:0]                              ahb_sram_rd_en;
logic [memory_pkg::SRAM_ADDR_WIDTH-1:0]  ahb_sram_rd_addr [3];
logic [31:0]                             ahb_sram_rd_data [3];

// Read data back from the SRAMs into the AHB slaves. The SRAM outputs are
// named per colour; the slaves index by channel. Without this the slaves
// return an undriven bus and every AHB read yields nothing -- which is what
// Synth 8-3848 was reporting.
assign ahb_sram_rd_data[0] = red_data;
assign ahb_sram_rd_data[1] = green_data;
assign ahb_sram_rd_data[2] = blue_data;
logic [2:0]                              ahb_sram_wr_en;
logic [3:0]                              ahb_sram_wr_be   [3];
logic [memory_pkg::SRAM_ADDR_WIDTH-1:0]  ahb_sram_wr_addr [3];
logic [31:0]                             ahb_sram_wr_data [3];

// WRITE MUX, per SRAM. The direct path keeps its byte enables for single
// pixels and partial rectangles; the AHB slaves take over only for a
// full-image write. wr_burst_active is high for the whole of one.
logic [2:0]                             sram_wr_en_ch;
logic [3:0]                             sram_wr_be_ch   [3];
logic [memory_pkg::SRAM_ADDR_WIDTH-1:0] sram_wr_addr_ch [3];

for (genvar ch = 0; ch < 3; ch++) begin : g_wr_mux
    assign sram_wr_en_ch[ch]   = wr_burst_active ? ahb_sram_wr_en[ch]
                                                 : sram_wr_en;
    assign sram_wr_be_ch[ch]   = wr_burst_active ? ahb_sram_wr_be[ch]
                                                 : sram_wr_be;
    assign sram_wr_addr_ch[ch] = wr_burst_active ? ahb_sram_wr_addr[ch]
                                                 : sram_wr_addr;
end : g_wr_mux

logic [2:0]                             sram_rd_en_ch;
logic [memory_pkg::SRAM_ADDR_WIDTH-1:0] sram_rd_addr_ch [3];

for (genvar ch = 0; ch < 3; ch++) begin : g_rd_mux
    assign sram_rd_en_ch[ch]   = pix_rd_owner ? sram_rd_en_mux
                                              : ahb_sram_rd_en[ch];
    assign sram_rd_addr_ch[ch] = pix_rd_owner ? sram_rd_addr_mux
                                              : ahb_sram_rd_addr[ch];
end : g_rd_mux

rgb_sram #(
    .DATA_WIDTH (memory_pkg::SRAM_DATA_WIDTH),
    .DEPTH      (memory_pkg::SRAM_DEPTH),
    .INIT_FILE  (memory_pkg::SRAM_INIT_R)
) u_sram_red (
    .clk     (clk),
    .rd_en   (sram_rd_en_ch[0]),
    .rd_addr (sram_rd_addr_ch[0]),
    .rd_data (red_data),
    .wr_en   (sram_wr_en_ch[0]),
    .wr_be   (sram_wr_be_ch[0]),
    .wr_addr (sram_wr_addr_ch[0]),
    .wr_data ((wr_burst_active ? ahb_sram_wr_data[0] : sram_wr_data_r))
);

rgb_sram #(
    .DATA_WIDTH (memory_pkg::SRAM_DATA_WIDTH),
    .DEPTH      (memory_pkg::SRAM_DEPTH),
    .INIT_FILE  (memory_pkg::SRAM_INIT_G)
) u_sram_green (
    .clk     (clk),
    .rd_en   (sram_rd_en_ch[1]),
    .rd_addr (sram_rd_addr_ch[1]),
    .rd_data (green_data),
    .wr_en   (sram_wr_en_ch[1]),
    .wr_be   (sram_wr_be_ch[1]),
    .wr_addr (sram_wr_addr_ch[1]),
    .wr_data ((wr_burst_active ? ahb_sram_wr_data[1] : sram_wr_data_g))
);

rgb_sram #(
    .DATA_WIDTH (memory_pkg::SRAM_DATA_WIDTH),
    .DEPTH      (memory_pkg::SRAM_DEPTH),
    .INIT_FILE  (memory_pkg::SRAM_INIT_B)
) u_sram_blue (
    .clk     (clk),
    .rd_en   (sram_rd_en_ch[2]),
    .rd_addr (sram_rd_addr_ch[2]),
    .rd_data (blue_data),
    .wr_en   (sram_wr_en_ch[2]),
    .wr_be   (sram_wr_be_ch[2]),
    .wr_addr (sram_wr_addr_ch[2]),
    .wr_data ((wr_burst_active ? ahb_sram_wr_data[2] : sram_wr_data_b))
);

// rom_sequencer declares its OWN IMG_WIDTH / IMG_HEIGHT / PIXELS_PER_WORD
// parameters with 256x256 defaults. They were never overridden, so ROM_DEPTH
// and ADDR_WIDTH were pinned at 16384 / 14 regardless of memory_pkg -- the
// module looked parameterised but was not connected to anything. Bind them.
// -----------------------------------------------------------------------------
// Full-image read path: AHB-Lite INCR4 bursts
// -----------------------------------------------------------------------------
// rom_sequencer read all three SRAMs in parallel at one address. A
// single-manager bus cannot: accesses serialise. img_burst_reader asks the
// arbiter for a channel, issues an INCR4 covering four words of that channel,
// and pushes the four beats into that channel's FIFO. rom_sequencer.sv is
// left in the tree unreferenced -- swapping this instantiation back is the
// whole revert if the burst path misbehaves.
logic [2:0]  arb_req, arb_gnt;
logic        ahb_req_valid, ahb_req_write, ahb_req_burst;
logic [1:0]  ahb_req_channel;
logic [13:0] ahb_req_word;
// Writer-side request, from the gather inside mem_write_subsystem.
// Reader-side request. The writer has its own set; the mux below picks
// between them for the shared master.
logic        rdr_req_valid, rdr_req_write, rdr_req_burst;
logic [1:0]  rdr_req_channel;
logic [13:0] rdr_req_word;
logic        wtr_req_valid, wtr_req_write, wtr_req_burst;
logic [1:0]  wtr_req_channel;
logic [13:0] wtr_req_word;
logic [31:0] wtr_ahb_wr_data;
logic        ahb_wr_ack;
logic [1:0]  ahb_wr_beat;

logic        ahb_busy, ahb_rd_valid;
logic [31:0] ahb_rd_data;
logic [1:0]  ahb_rd_beat;
logic        ahb_err_sticky;

img_burst_reader #(
    .IMG_WIDTH       (memory_pkg::IMG_WIDTH),
    .IMG_HEIGHT      (memory_pkg::IMG_HEIGHT),
    .PIXELS_PER_WORD (memory_pkg::PIXELS_PER_WORD)
) u_burst_reader (
    .clk              (clk),
    .rst_n            (rst_n),
    .start            (read_go),
    .seq_done         (seq_done),
    .busy             (rom_seq_busy),
    .fifo_almost_full (img_fifo_almost_full),
    .fifo_wr_en       (img_fifo_wr_en),
    .fifo_wr_data     (img_fifo_wr_data),
    .req_valid        (rdr_req_valid),
    .req_write        (rdr_req_write),
    .req_burst        (rdr_req_burst),
    .req_channel      (rdr_req_channel),
    .req_word         (rdr_req_word),
    .ahb_busy         (ahb_busy),
    .rd_valid         (ahb_rd_valid),
    .rd_data          (ahb_rd_data),
    .arb_req          (arb_req),
    .arb_gnt          (arb_gnt)
);

// Round-robin with lock. A channel requests only when its FIFO can take a
// whole burst; the lock holds the grant so a burst is never cut in half.
round_robin_arbiter #(.N(3)) u_ch_arbiter (
    .clk   (clk),
    .rst_n (rst_n),
    .req   (arb_req),
    .gnt   (arb_gnt)
);

// -----------------------------------------------------------------------------
// AHB-Lite fabric
// -----------------------------------------------------------------------------
logic [31:0] ahb_haddr, ahb_hwdata, ahb_hrdata;
logic        ahb_hwrite, ahb_hready, ahb_hresp;
logic [2:0]  ahb_hsize, ahb_hburst;
logic [1:0]  ahb_htrans;
logic [2:0]  ahb_hsel, ahb_s_hreadyout, ahb_s_hresp;
logic [31:0] ahb_s_hrdata [3];
logic        ahb_decode_err;

// -----------------------------------------------------------------------------
// The read and write burst engines share one AHB master
// -----------------------------------------------------------------------------
// Safe without a second arbiter because mem_interlock already makes reads and
// writes mutually exclusive, in BOTH directions:
//
//   wr_port_grant = !read_active && !pix_rd_active
//   read_go       = start_pending && !wr_pending && ...
//
// so only one engine can ever be running. wr_burst_active is high for the
// whole of a full-image write.
assign ahb_req_valid   = wr_burst_active ? wtr_req_valid   : rdr_req_valid;
assign ahb_req_write   = wr_burst_active ? wtr_req_write   : rdr_req_write;
assign ahb_req_burst   = wr_burst_active ? wtr_req_burst   : rdr_req_burst;
assign ahb_req_channel = wr_burst_active ? wtr_req_channel : rdr_req_channel;
assign ahb_req_word    = wr_burst_active ? wtr_req_word    : rdr_req_word;

ahb_master u_ahb_master (
    .hclk(clk), .hresetn(rst_n),
    .req_valid(ahb_req_valid), .req_write(ahb_req_write),
    .req_burst(ahb_req_burst), .req_channel(ahb_req_channel),
    .req_word(ahb_req_word), .busy(ahb_busy),
    .rd_valid(ahb_rd_valid), .rd_data(ahb_rd_data), .rd_beat(ahb_rd_beat),
    // Write data from the gather; read data to the burst reader. Only one is
    // ever active. Leaving these tied off is what Synth 8-3848 caught --
    // ahb_wr_beat had no driver and the gather's data mux indexed with X.
    .wr_data(wtr_ahb_wr_data), .wr_ack(ahb_wr_ack), .wr_beat(ahb_wr_beat),
    .haddr(ahb_haddr), .hwrite(ahb_hwrite), .hsize(ahb_hsize),
    .hburst(ahb_hburst), .htrans(ahb_htrans), .hwdata(ahb_hwdata),
    .hready(ahb_hready), .hrdata(ahb_hrdata), .hresp(ahb_hresp),
    .err_sticky(ahb_err_sticky)
);

ahb_decoder u_ahb_decoder (
    .hclk(clk), .hresetn(rst_n),
    .haddr(ahb_haddr), .htrans(ahb_htrans),
    .m_hready(ahb_hready), .m_hrdata(ahb_hrdata), .m_hresp(ahb_hresp),
    .hsel(ahb_hsel),
    .s_hreadyout(ahb_s_hreadyout), .s_hresp(ahb_s_hresp),
    .s_hrdata(ahb_s_hrdata),
    .decode_err(ahb_decode_err)
);

// One slave per channel. Read-only here: the write path keeps the direct
// port, because AHB-Lite has no byte strobes and a single-pixel write needs
// an arbitrary lane mask.


for (genvar ch = 0; ch < 3; ch++) begin : g_ahb_slave
    ahb_slave_sram #(
        .SRAM_ADDR_W (memory_pkg::SRAM_ADDR_WIDTH)
    ) u_slave (
        .hclk(clk), .hresetn(rst_n),
        .hsel(ahb_hsel[ch]),
        .hready(ahb_hready),                 // AGGREGATED segment ready
        .haddr(ahb_haddr), .hwrite(ahb_hwrite), .hsize(ahb_hsize),
        .hburst(ahb_hburst), .htrans(ahb_htrans), .hwdata(ahb_hwdata),
        .hreadyout(ahb_s_hreadyout[ch]), .hrdata(ahb_s_hrdata[ch]),
        .hresp(ahb_s_hresp[ch]),
        .sram_rd_en(ahb_sram_rd_en[ch]),
        .sram_rd_addr(ahb_sram_rd_addr[ch]),
        .sram_rd_data(ahb_sram_rd_data[ch]),
        // The write side goes live here for the first time. It was tied off
        // through step 6b, so the slaves' wr_pending / wr_addr_q registers
        // were optimised away entirely -- expect the flop count to rise.
        .sram_wr_en   (ahb_sram_wr_en[ch]),
        .sram_wr_be   (ahb_sram_wr_be[ch]),
        .sram_wr_addr (ahb_sram_wr_addr[ch]),
        .sram_wr_data (ahb_sram_wr_data[ch])
    );
end : g_ahb_slave

// -----------------------------------------------------------------------
// WRITE PATH
//
// Messages are decoded into writes, packed into SRAM words, and committed only
// when the shared memory port grants access.
// -----------------------------------------------------------------------
// -----------------------------------------------------------------------
// MESSAGE ROUTER
//
// The router dispatches each incoming message to the correct handler in order.
// If a destination is busy, it holds the message and back-pressures the input
// stream.
// -----------------------------------------------------------------------
mem_msg_router u_msg_router (
    .clk              (clk),
    .rst_n            (rst_n),

    .msg_valid        (msg_valid),
    .msg_ready        (msg_ready),
    .msg_kind         (msg_kind),
    .msg_payload      (msg_payload),

    .wr_msg_valid     (wr_msg_valid),
    .wr_msg_ready     (wr_msg_ready),
    .wr_msg_kind      (wr_msg_kind),
    .wr_msg_payload   (wr_msg_payload),

    .pix_req_valid    (pix_req_valid),
    .pix_req_row      (pix_req_row),
    .pix_req_col      (pix_req_col),
    .pix_busy         (pix_rd_busy),

    .brd_req_valid    (brd_req_valid),
    .brd_req_base_row (brd_req_base_row),
    .brd_req_base_col (brd_req_base_col),
    .brd_req_height   (brd_req_height),
    .brd_req_width    (brd_req_width),
    .brd_busy         (brd_busy),

    .rgf_cmd_valid    (rgf_cmd_valid),
    .rgf_cmd_is_write (rgf_cmd_is_write),
    .rgf_cmd_addr     (rgf_cmd_addr),
    .rgf_cmd_wdata    (rgf_cmd_wdata),
    .apb_busy         (apb_busy)
);

mem_write_subsystem u_write_path (
    .clk          (clk),
    .rst_n        (rst_n),

    .msg_valid    (wr_msg_valid),
    .msg_ready    (wr_msg_ready),
    .msg_kind     (wr_msg_kind),
    .msg_payload  (wr_msg_payload),

    .wr_en        (sram_wr_en),
    .wr_be        (sram_wr_be),
    .wr_addr      (sram_wr_addr),
    .wr_data_r    (sram_wr_data_r),
    .wr_data_g    (sram_wr_data_g),
    .burst_active    (wr_burst_active),
    .ahb_req_valid   (wtr_req_valid),
    .ahb_req_write   (wtr_req_write),
    .ahb_req_burst   (wtr_req_burst),
    .ahb_req_channel (wtr_req_channel),
    .ahb_req_word    (wtr_req_word),
    .ahb_busy        (ahb_busy),
    .ahb_wr_beat     (ahb_wr_beat),
    .ahb_wr_ack      (ahb_wr_ack),
    .ahb_wr_data     (wtr_ahb_wr_data),
    .wr_data_b    (sram_wr_data_b),

    .wr_allowed   (wr_port_grant),

    .rect_busy    (wr_rect_busy),
    .wr_rejected  (sram_wr_rejected)
);

// sram_wr_seen: at least one pixel has been written.
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)          sram_wr_seen <= 1'b0;
    else if (sram_wr_en) sram_wr_seen <= 1'b1;
end

pixel_rd_ctrl u_pixel_rd_ctrl (
    .clk          (clk),
    .rst_n        (rst_n),
    .req_valid    (pix_req_valid),
    .req_row      (pix_req_row),
    .req_col      (pix_req_col),
    .pix_rd_req   (pix_rd_req),
    .pix_rd_gnt   (pix_rd_gnt),
    .pix_rd_done  (pix_rd_done),
    .sram_rd_en   (pix_sram_rd_en),
    .sram_rd_addr (pix_sram_rd_addr),
    .red_data     (red_data),
    .green_data   (green_data),
    .blue_data    (blue_data),
    .rpy_valid    (pix_rpy_valid),
    .rpy_accept   (pix_rpy_accept),
    .rpy_row      (pix_rpy_row),
    .rpy_col      (pix_rpy_col),
    .rpy_pixel    (pix_rpy_pixel),
    .busy         (pix_rd_busy),
    .req_overrun  (pix_rd_overrun)
);

burst_rd_ctrl u_burst_rd_ctrl (
    .clk          (clk),
    .rst_n        (rst_n),
    .req_valid    (brd_req_valid),
    .req_base_row (brd_req_base_row),
    .req_base_col (brd_req_base_col),
    .req_height   (brd_req_height),
    .req_width    (brd_req_width),
    .brd_rd_req   (brd_rd_req),
    .brd_rd_gnt   (brd_rd_gnt),
    .brd_rd_done  (brd_rd_done),
    .sram_rd_en   (brd_sram_rd_en),
    .sram_rd_addr (brd_sram_rd_addr),
    .red_data     (red_data),
    .green_data   (green_data),
    .blue_data    (blue_data),
    .msg_valid    (brd_msg_valid),
    .msg_accept   (brd_msg_accept),
    .msg_pixels   (brd_msg_pixels),
    .busy         (brd_busy),
    .req_overrun  (brd_overrun)
);

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        arb_locked   <= 1'b0;
        arb_pix_owns <= 1'b0;
        arb_brd_owns <= 1'b0;
    end
    else if (!arb_locked) begin
        if (pix_rd_req) begin
            arb_locked   <= 1'b1;
            arb_pix_owns <= 1'b1;
        end
        else if (brd_rd_req) begin
            arb_locked   <= 1'b1;
            arb_brd_owns <= 1'b1;
        end
    end
    else if (shared_rd_done) begin
        arb_locked   <= 1'b0;
        arb_pix_owns <= 1'b0;
        arb_brd_owns <= 1'b0;
    end
end

assign shared_rd_req  = (arb_pix_owns && pix_rd_req) ||
                        (arb_brd_owns && brd_rd_req);
assign shared_rd_done = (arb_pix_owns && pix_rd_done) ||
                        (arb_brd_owns && brd_rd_done);
assign pix_rd_gnt     = arb_pix_owns && shared_rd_gnt;
assign brd_rd_gnt     = arb_brd_owns && shared_rd_gnt;

mem_interlock u_mem_interlock (
    .clk          (clk),
    .rst_n        (rst_n),
    .start_req    (start_req),
    .rom_seq_busy (rom_seq_busy),
    .img_done     (img_done),
    .read_go      (read_go),
    .wr_port_req  (wr_msg_valid),
    .wr_busy      (wr_rect_busy),
    .burst_active (burst_active),
    .wr_port_grant (wr_port_grant),
    .pix_rd_req   (shared_rd_req),
    .pix_rd_done  (shared_rd_done),
    .pix_rd_gnt   (shared_rd_gnt),
    .pix_rd_owner (pix_rd_owner)
);

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) pix_rpy_valid_d <= 1'b0;
    else        pix_rpy_valid_d <= pix_rpy_valid;
end
assign pix_rpy_send    = pix_rpy_valid && !pix_rpy_valid_d;
assign pix_rpy_payload = {pix_rpy_row, pix_rpy_col, pix_rpy_pixel};

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) brd_msg_valid_d <= 1'b0;
    else        brd_msg_valid_d <= brd_msg_valid;
end
assign brd_msg_send    = brd_msg_valid && !brd_msg_valid_d;
assign brd_msg_payload = brd_msg_pixels;

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)                              img_fifo_ovf_sticky <= 1'b0;
    else if (|(img_fifo_wr_en & img_fifo_full)) img_fifo_ovf_sticky <= 1'b1;
end

`ifndef SYNTHESIS
    // The AHB slaves and the direct readers must never drive a read port in
    // the same cycle. pix_rd_owner selects between them, so this catches an
    // ownership handoff that let both through.
    a_read_client_exclusive: assert property (
        @(posedge clk) disable iff (!rst_n)
        !((|ahb_sram_rd_en) && pix_sram_rd_en)
    ) else $error("memory_subsystem: image and pixel readers drove SRAM together");

    // The burst path may only reach the SRAMs when it owns the port.
    a_ahb_owns_port: assert property (
        @(posedge clk) disable iff (!rst_n)
        (|ahb_sram_rd_en) |-> !pix_rd_owner
    ) else $error("memory_subsystem: AHB read while the pixel readers owned the port");

    a_pix_rd_owns: assert property (
        @(posedge clk) disable iff (!rst_n)
        pix_sram_rd_en |-> pix_rd_owner
    ) else $error("memory_subsystem: pixel read drove SRAM without ownership");

    a_arb_exclusive: assert property (
        @(posedge clk) disable iff (!rst_n)
        !(arb_pix_owns && arb_brd_owns)
    ) else $error("memory_subsystem: both read clients own the shared port");

    a_brd_owns_when_reading: assert property (
        @(posedge clk) disable iff (!rst_n)
        brd_sram_rd_en |-> (arb_brd_owns && pix_rd_owner)
    ) else $error("memory_subsystem: burst read drove SRAM without ownership");

    a_no_write_during_burst: assert property (
        @(posedge clk) disable iff (!rst_n)
        (arb_brd_owns && pix_rd_owner) |-> !wr_port_grant
    ) else $error("memory_subsystem: write allowed during burst read");

    a_pix_src_holds: assert property (
        @(posedge clk) disable iff (!rst_n)
        (pix_rpy_valid && !pix_rpy_accept)
            |=> ($stable(pix_rpy_row) && $stable(pix_rpy_col) &&
                 $stable(pix_rpy_pixel))
    ) else $error("memory_subsystem: source released pixel reply before ack");
`endif

endmodule : memory_subsystem
