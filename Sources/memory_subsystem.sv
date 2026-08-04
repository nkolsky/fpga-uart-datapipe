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
    input  logic             cmd_fifo_empty,
    input  logic [CMD_W-1:0] cmd_fifo_rd_data,
    output logic             cmd_fifo_rd_en,

    // Single-pixel read request/reply, already in the 100 MHz domain.
    input  logic        pix_req_valid,
    input  logic [19:0] pix_req_data,
    input  logic        pix_rpy_accept,
    output logic        pix_rpy_send,
    output logic [43:0] pix_rpy_payload,

    // Burst-read request/reply, already in the 100 MHz domain.
    input  logic        brd_req_valid,
    input  logic [39:0] brd_req_data,
    input  logic        brd_msg_accept,
    output logic        brd_msg_send,
    output logic [95:0] brd_msg_payload,

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

logic        sram_rd_en_mux;
logic [memory_pkg::SRAM_ADDR_WIDTH-1:0] sram_rd_addr_mux;

assign sram_rd_en_mux   = pix_rd_owner
                        ? (arb_brd_owns ? brd_sram_rd_en   : pix_sram_rd_en)
                        : rom_rd_en;
assign sram_rd_addr_mux = pix_rd_owner
                        ? (arb_brd_owns ? brd_sram_rd_addr : pix_sram_rd_addr)
                        : rom_addr;

rgb_sram #(
    .DATA_WIDTH (memory_pkg::SRAM_DATA_WIDTH),
    .DEPTH      (memory_pkg::SRAM_DEPTH),
    .INIT_FILE  ("red_hex.mem")
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
    .INIT_FILE  ("green_hex.mem")
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
    .INIT_FILE  ("blue_hex.mem")
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

sram_wr_ctrl #(
    .CMD_W (CMD_W)
) u_sram_wr_ctrl (
    .clk         (clk),
    .rst_n       (rst_n),
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

pixel_rd_ctrl u_pixel_rd_ctrl (
    .clk          (clk),
    .rst_n        (rst_n),
    .req_valid    (pix_req_valid),
    .req_row      (pix_req_data[19:10]),
    .req_col      (pix_req_data[9:0]),
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
    .req_base_row (brd_req_data[39:30]),
    .req_base_col (brd_req_data[29:20]),
    .req_height   (brd_req_data[19:10]),
    .req_width    (brd_req_data[9:0]),
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
    .cmd_empty    (cmd_fifo_empty),
    .wr_busy      (sram_wr_busy),
    .burst_active (burst_active),
    .wr_allowed   (sram_wr_allowed),
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
