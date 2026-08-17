// -----------------------------------------------------------------------------
// uart_tx_subsystem.sv
//
// Pure hierarchy extraction of the complete 130 MHz UART transmit datapath.
//
// Contains:
//   - tx_sequencer
//   - tx_reply_ctrl
//   - destination-side pixel/burst reply holding registers
//   - msg_composer and burst_msg_composer
//   - reply/image MAC ownership and message mux
//   - tx_mac
//   - tx_phy
//   - tx_done_sticky
//
// CDC primitives and async FIFOs remain in chip_top.
// -----------------------------------------------------------------------------
`timescale 1ns/1ps

module uart_tx_subsystem (
    input  logic         clk,
    input  logic         rst_n,

    // Full-image FIFO read side. The async FIFO remains in chip_top.
    input  logic         fifo_empty,
    input  logic [23:0]  fifo_rd_data,
    output logic         fifo_rd_en,

    // Host-side flow control.
    input  logic         cts,

    // Register-read reply, already delivered into the 130 MHz domain.
    input  logic         rd_reply_valid,
    input  logic [31:0]  rd_reply_data,

    // Atomic pixel-reply transaction from the 100->130 MHz CDC.
    input  logic         pix_reply_valid,
    input  logic [43:0]  pix_reply_data,
    output logic         pix_reply_accept,

    // Atomic burst-reply transaction from the 100->130 MHz CDC.
    input  logic         burst_reply_valid,
    input  logic [95:0]  burst_reply_data,
    output logic         burst_reply_accept,

    // UART and diagnostic/status outputs.
    output logic         uart_tx,
    output logic         tx_activity_led,
    output logic         mac_busy,
    output logic         tx_img_done,
    output logic         tx_seq_busy,
    output logic [9:0]   tx_row,
    output logic [9:0]   tx_col,
    output logic         tx_done_sticky,
    output logic         reply_pending,
    output logic         reply_overrun
);

logic         image_msg_valid;
logic [127:0] image_msg_data;

logic         pix_reply_held;
logic [43:0]  pix_reply_payload;
logic [127:0] pix_reply_msg;

logic         burst_reply_held;
logic [95:0]  burst_reply_payload;
logic [127:0] burst_reply_msg;

logic         reply_req;
logic         reply_sent;
logic [127:0] reply_msg;
logic [4:0]   reply_len;

logic         reply_owns_mac;
logic         reply_drives_mac;
logic         tx_mac_msg_valid;
logic [127:0] tx_mac_msg_data;
logic [4:0]   tx_mac_msg_len;

logic         phy_valid;
logic [7:0]   phy_data;
logic         phy_ready;

// tx_sequencer declares its OWN IMG_WIDTH / IMG_HEIGHT parameters with
// 256x256 defaults and does not import memory_pkg, so without these
// overrides its end-of-image test was pinned at 256x256 regardless of the
// build. Under -DSIMULATION the geometry is 8x8: the sequencer would wait
// for last_pixel at (255,255) while rom_sequencer pushes only 64 pixels, so
// tx_img_done never fires, IMG_TX_MON is never written and img_in_flight in
// mem_interlock never clears. Harmless in the hardware build only because
// 256 happens to be the right answer there.
//
// Same failure and same fix as rom_sequencer in memory_subsystem.sv.
tx_sequencer #(
    .IMG_WIDTH  (memory_pkg::IMG_WIDTH),
    .IMG_HEIGHT (memory_pkg::IMG_HEIGHT)
) u_tx_sequencer (
    .clk          (clk),
    .rst_n        (rst_n),
    .fifo_empty   (fifo_empty),
    .cts          (cts),
    .mac_busy     (mac_busy),
    .fifo_rd_data (fifo_rd_data),
    .msg_valid    (image_msg_valid),
    .fifo_pop     (fifo_rd_en),
    .tx_img_done  (tx_img_done),
    .busy         (tx_seq_busy),
    .msg          (image_msg_data),
    .row_cnt_out  (tx_row),
    .col_cnt_out  (tx_col)
);

// Convert each one-cycle CDC delivery into a held transaction. The request
// stays asserted until tx_reply_ctrl accepts it, preserving backpressure.
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        pix_reply_held    <= 1'b0;
        pix_reply_payload <= '0;
    end
    else if (pix_reply_valid) begin
        pix_reply_held    <= 1'b1;
        pix_reply_payload <= pix_reply_data;
    end
    else if (pix_reply_accept) begin
        pix_reply_held    <= 1'b0;
    end
end

msg_composer u_pix_reply_composer (
    .row   (pix_reply_payload[43:34]),
    .col   (pix_reply_payload[33:24]),
    .pixel (pix_reply_payload[23:0]),
    .msg   (pix_reply_msg)
);

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        burst_reply_held    <= 1'b0;
        burst_reply_payload <= '0;
    end
    else if (burst_reply_valid) begin
        burst_reply_held    <= 1'b1;
        burst_reply_payload <= burst_reply_data;
    end
    else if (burst_reply_accept) begin
        burst_reply_held    <= 1'b0;
    end
end

burst_msg_composer u_burst_msg_composer (
    .pixels (burst_reply_payload),
    .msg    (burst_reply_msg)
);

tx_reply_ctrl u_tx_reply_ctrl (
    .clk           (clk),
    .rst_n         (rst_n),
    .rd_valid      (rd_reply_valid),
    .rd_data       (rd_reply_data),
    .pix_valid     (pix_reply_held),
    .pix_msg       (pix_reply_msg),
    .pix_accept    (pix_reply_accept),
    .brd_valid     (burst_reply_held),
    .brd_msg       (burst_reply_msg),
    .brd_accept    (burst_reply_accept),
    .tx_seq_busy   (tx_seq_busy),
    .mac_busy      (mac_busy),
    .cts           (cts),
    .reply_req     (reply_req),
    .reply_msg     (reply_msg),
    .reply_len     (reply_len),
    .reply_pending (reply_pending),
    .reply_sent    (reply_sent),
    .reply_overrun (reply_overrun)
);

// tx_mac captures data one cycle after first observing msg_valid. Extend the
// reply mux ownership through MAC_LOAD so reply data and length remain stable.
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)           reply_owns_mac <= 1'b0;
    else if (reply_req)   reply_owns_mac <= 1'b1;
    else if (reply_sent)  reply_owns_mac <= 1'b0;
end

assign reply_drives_mac = reply_req || reply_owns_mac;

assign tx_mac_msg_valid = reply_drives_mac ? reply_req       : image_msg_valid;
assign tx_mac_msg_data  = reply_drives_mac ? reply_msg       : image_msg_data;
assign tx_mac_msg_len   = reply_drives_mac ? reply_len       : 5'd16;

tx_mac u_tx_mac (
    .clk       (clk),
    .rst_n     (rst_n),
    .rx_mode   (1'b0),
    .msg_valid (tx_mac_msg_valid),
    .msg_data  (tx_mac_msg_data),
    .msg_len   (tx_mac_msg_len),
    .mac_busy  (mac_busy),
    .phy_ready (phy_ready),
    .phy_data  (phy_data),
    .phy_valid (phy_valid)
);

tx_phy u_tx_phy (
    .clk       (clk),
    .rst_n     (rst_n),
    .phy_valid (phy_valid),
    .phy_data  (phy_data),
    .phy_ready (phy_ready),
    .tx_out    (uart_tx),
    .led       (tx_activity_led)
);

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)             tx_done_sticky <= 1'b0;
    else if (tx_img_done)   tx_done_sticky <= 1'b1;
end

`ifndef SYNTHESIS
    a_tx_producers_exclusive: assert property (
        @(posedge clk) disable iff (!rst_n)
        !(reply_req && image_msg_valid)
    ) else $error("uart_tx_subsystem: reply and image message offered together");

    a_image_len_16: assert property (
        @(posedge clk) disable iff (!rst_n)
        (image_msg_valid && !reply_drives_mac) |-> (tx_mac_msg_len == 5'd16)
    ) else $error("uart_tx_subsystem: image message used non-16-byte length");

    a_reply_held_through_load: assert property (
        @(posedge clk) disable iff (!rst_n)
        ($rose(mac_busy) && $past(reply_drives_mac)) |-> reply_drives_mac
    ) else $error("uart_tx_subsystem: reply mux released before MAC capture");

    a_pix_reply_no_overwrite: assert property (
        @(posedge clk) disable iff (!rst_n)
        pix_reply_valid |-> !pix_reply_held
    ) else $error("uart_tx_subsystem: pixel reply overwritten before acceptance");

    a_pix_reply_stable: assert property (
        @(posedge clk) disable iff (!rst_n)
        (pix_reply_held && !pix_reply_accept) |=> $stable(pix_reply_payload)
    ) else $error("uart_tx_subsystem: held pixel payload changed");

    a_burst_reply_no_overwrite: assert property (
        @(posedge clk) disable iff (!rst_n)
        burst_reply_valid |-> !burst_reply_held
    ) else $error("uart_tx_subsystem: burst reply overwritten before acceptance");

    a_burst_reply_stable: assert property (
        @(posedge clk) disable iff (!rst_n)
        (burst_reply_held && !burst_reply_accept) |=> $stable(burst_reply_payload)
    ) else $error("uart_tx_subsystem: held burst payload changed");
`endif

endmodule : uart_tx_subsystem
