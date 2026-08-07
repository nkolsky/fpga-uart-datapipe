// -----------------------------------------------------------------------------
// memory_subsystem.sv
//
// Pure hierarchy extraction of the complete 100 MHz memory domain.
//
// Contains:
//   - rgb_sram x3
//   - rom_sequencer
//   - sram_wr_ctrl
//   - pixel_rd_ctrl
//   - burst_rd_ctrl
//   - shared pixel/burst read arbiter
//   - mem_interlock
//   - SRAM read-port mux
//   - image FIFO overflow sticky diagnostic
//
// All CDC primitives and both async FIFOs remain in chip_top.
// -----------------------------------------------------------------------------
`timescale 1ns/1ps

module memory_subsystem #(
    parameter int CMD_W = 48
) (
    input  logic        clk,
    input  logic        rst_n,

    // Full-image request and completion.
    input  logic        start_req,
    input  logic        img_done,
    input  logic        burst_active,

    // Image FIFO write side. The FIFO itself remains in chip_top.
    input  logic        img_fifo_almost_full,
    input  logic        img_fifo_almost_empty,
    input  logic        img_fifo_full,
    output logic        img_fifo_wr_en,
    output logic [23:0] img_fifo_wr_data,

    // Pixel-write command FIFO read side. The FIFO remains in chip_top.
    // ---- message in, from cdc_msg_sync ---------------------------------
    // Replaces the command FIFO. Every message frame crosses here, in order,
    // and back-pressure runs from this port all the way to the PC.
    input  logic                        msg_valid,
    output logic                        msg_ready,
    input  msg_format_pkg::msg_kind_t   msg_kind,
    input  msg_format_pkg::msg_payload_t msg_payload,

    // Read REQUESTS no longer arrive as separate crossings: they are
    // messages like any other, and mem_msg_router dispatches them. Only the
    // REPLIES still cross on their own, since they travel the other way.
    input  logic        pix_rpy_accept,
    output logic        pix_rpy_send,
    output logic [43:0] pix_rpy_payload,

    input  logic        brd_msg_accept,
    output logic        brd_msg_send,
    output logic [95:0] brd_msg_payload,

    // Register file commands, routed from the same message stream. The RGF
    // itself lives in register_subsystem at the top level.
    output logic        rgf_cmd_valid,
    output logic        rgf_cmd_is_write,
    output logic [7:0]  rgf_cmd_addr,
    output logic [31:0] rgf_cmd_wdata,

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

logic [memory_pkg::SRAM_ADDR_WIDTH-1:0] rom_addr;
logic        rom_rd_en;
logic [31:0] red_data;
logic [31:0] green_data;
logic [31:0] blue_data;

logic                                    sram_wr_en;
logic [memory_pkg::SRAM_DATA_WIDTH/8-1:0] sram_wr_be;
logic [memory_pkg::SRAM_ADDR_WIDTH-1:0]   sram_wr_addr;
logic [memory_pkg::SRAM_DATA_WIDTH-1:0]   sram_wr_data_r;
logic [memory_pkg::SRAM_DATA_WIDTH-1:0]   sram_wr_data_g;
logic [memory_pkg::SRAM_DATA_WIDTH-1:0]   sram_wr_data_b;
logic                                    sram_wr_allowed;
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
assign sram_rd_en_mux   = pix_rd_owner
                        ? (arb_brd_owns ? brd_sram_rd_en   : pix_sram_rd_en)
                        : rom_rd_en;
assign sram_rd_addr_mux = pix_rd_owner
                        ? (arb_brd_owns ? brd_sram_rd_addr : pix_sram_rd_addr)
                        : rom_addr;

rgb_sram #(
    .DATA_WIDTH (memory_pkg::SRAM_DATA_WIDTH),
    .DEPTH      (memory_pkg::SRAM_DEPTH),
    .INIT_FILE  (memory_pkg::SRAM_INIT_R)
) u_sram_red (
    .clk     (clk),
    .rd_en   (sram_rd_en_mux),
    .rd_addr (sram_rd_addr_mux),
    .rd_data (red_data),
    .wr_en   (sram_wr_en),
    .wr_be   (sram_wr_be),
    .wr_addr (sram_wr_addr),
    .wr_data (sram_wr_data_r)
);

rgb_sram #(
    .DATA_WIDTH (memory_pkg::SRAM_DATA_WIDTH),
    .DEPTH      (memory_pkg::SRAM_DEPTH),
    .INIT_FILE  (memory_pkg::SRAM_INIT_G)
) u_sram_green (
    .clk     (clk),
    .rd_en   (sram_rd_en_mux),
    .rd_addr (sram_rd_addr_mux),
    .rd_data (green_data),
    .wr_en   (sram_wr_en),
    .wr_be   (sram_wr_be),
    .wr_addr (sram_wr_addr),
    .wr_data (sram_wr_data_g)
);

rgb_sram #(
    .DATA_WIDTH (memory_pkg::SRAM_DATA_WIDTH),
    .DEPTH      (memory_pkg::SRAM_DEPTH),
    .INIT_FILE  (memory_pkg::SRAM_INIT_B)
) u_sram_blue (
    .clk     (clk),
    .rd_en   (sram_rd_en_mux),
    .rd_addr (sram_rd_addr_mux),
    .rd_data (blue_data),
    .wr_en   (sram_wr_en),
    .wr_be   (sram_wr_be),
    .wr_addr (sram_wr_addr),
    .wr_data (sram_wr_data_b)
);

// rom_sequencer declares its OWN IMG_WIDTH / IMG_HEIGHT / PIXELS_PER_WORD
// parameters with 256x256 defaults. They were never overridden, so ROM_DEPTH
// and ADDR_WIDTH were pinned at 16384 / 14 regardless of memory_pkg -- the
// module looked parameterised but was not connected to anything. Bind them.
rom_sequencer #(
    .IMG_WIDTH       (memory_pkg::IMG_WIDTH),
    .IMG_HEIGHT      (memory_pkg::IMG_HEIGHT),
    .PIXELS_PER_WORD (memory_pkg::PIXELS_PER_WORD)
) u_rom_sequencer (
    .clk          (clk),
    .rst_n        (rst_n),
    .start        (read_go),
    .almost_full  (img_fifo_almost_full),
    .almost_empty (img_fifo_almost_empty),
    .red_data     (red_data),
    .green_data   (green_data),
    .blue_data    (blue_data),
    .rom_addr     (rom_addr),
    .rom_rd_en    (rom_rd_en),
    .wr_en        (img_fifo_wr_en),
    .wr_data      (img_fifo_wr_data),
    .seq_done     (seq_done),
    .busy         (rom_seq_busy)
);

// -----------------------------------------------------------------------
// WRITE PATH
//
// Replaces sram_wr_ctrl and the command FIFO. Messages arrive whole:
//
//   mem_msg_writer     decodes msg_kind and opens a rectangle
//   pixel_word_packer  accumulates pixels into 32-bit words
//   sram_rmw           writes the word, reading first only when the update
//                      does not cover every lane
//
// A burst data message carries four pixels and four pixels are exactly one
// word per channel, so burst traffic writes whole words and needs no reads.
// -----------------------------------------------------------------------
// -----------------------------------------------------------------------
// MESSAGE ROUTER
//
// Every message arrives on one ordered stream and is dispatched to the one
// thing that handles it. Reads and register commands used to reach this
// domain through their OWN cdc_cmd_sync instances, in parallel with the
// command FIFO carrying writes, and nothing ordered those paths against each
// other. Now dispatch follows the order the PC sent.
//
// The router HOLDS a message until its destination can take it. Both read
// controllers take a one-cycle strobe and drop anything arriving while busy
// -- each carries a req_overrun sticky saying so -- and that stall
// propagates back through the crossing to the PC.
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
    .rgf_cmd_wdata    (rgf_cmd_wdata)
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
    else if (img_fifo_wr_en && img_fifo_full) img_fifo_ovf_sticky <= 1'b1;
end

`ifndef SYNTHESIS
    a_read_client_exclusive: assert property (
        @(posedge clk) disable iff (!rst_n)
        !(rom_rd_en && pix_sram_rd_en)
    ) else $error("memory_subsystem: image and pixel readers drove SRAM together");

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
        (arb_brd_owns && pix_rd_owner) |-> !sram_wr_allowed
    ) else $error("memory_subsystem: write allowed during burst read");

    a_pix_src_holds: assert property (
        @(posedge clk) disable iff (!rst_n)
        (pix_rpy_valid && !pix_rpy_accept)
            |=> ($stable(pix_rpy_row) && $stable(pix_rpy_col) &&
                 $stable(pix_rpy_pixel))
    ) else $error("memory_subsystem: source released pixel reply before ack");
`endif

endmodule : memory_subsystem
