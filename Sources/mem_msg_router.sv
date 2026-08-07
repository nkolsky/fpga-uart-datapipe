// mem_msg_router.sv
// =================
// Routes each arriving message to the one thing on the memory side that
// handles it, and holds the message until that destination can take it.
//
//                        +-> write path      pixel write, burst header/data
//   msg -> mem_msg_router+-> pixel_rd_ctrl   single pixel read
//                        +-> burst_rd_ctrl   image burst read
//                        +-> register file   register read and write
//
// -----------------------------------------------------------------------
// WHY IT HOLDS
// -----------------------------------------------------------------------
// Both read controllers take a ONE-CYCLE STROBE and expose a `busy` flag,
// and each carries a `req_overrun` sticky meaning "a request arrived while I
// was busy and I dropped it". Firing a strobe blindly is therefore a silent
// data-loss path -- the same shape as the command FIFO overflow this rework
// removed.
//
// So the router presents a message to its destination and does not accept
// the next one until it has been taken. Because everything upstream is
// ready/valid, that stall propagates: router -> crossing -> rx_classifier
// -> rx_mac -> UART_CTS -> the PC pauses. Nothing is dropped, transfers
// just take longer.
//
// -----------------------------------------------------------------------
// ORDERING IS NOW GUARANTEED, WHICH IT WAS NOT BEFORE
// -----------------------------------------------------------------------
// Reads and register commands used to reach the memory domain through their
// OWN cdc_cmd_sync instances, in parallel with the command FIFO carrying
// writes. Nothing ordered those paths against each other: a register write
// enabling something and a burst read depending on it crossed independently,
// and only similar latencies made the design appear to work.
//
// One crossing and one router means messages arrive and are dispatched in
// the order the PC sent them.
//
// -----------------------------------------------------------------------
// THE REGISTER FILE HAS NO BACK-PRESSURE
// -----------------------------------------------------------------------
// register_subsystem accepts a command every cycle, so register traffic
// never stalls the router. Only the two read controllers and the write path
// can hold it up.

`timescale 1ns/1ps

module mem_msg_router
    import msg_format_pkg::*;
    import memory_pkg::*;
(
    input  logic clk,
    input  logic rst_n,

    // ---- message in, from cdc_msg_sync -----------------------------------
    input  logic          msg_valid,
    output logic          msg_ready,
    input  msg_kind_t     msg_kind,
    input  msg_payload_t  msg_payload,

    // ---- to the write path ------------------------------------------------
    output logic          wr_msg_valid,
    input  logic          wr_msg_ready,
    output msg_kind_t     wr_msg_kind,
    output msg_payload_t  wr_msg_payload,

    // ---- to pixel_rd_ctrl --------------------------------------------------
    // One-cycle strobe. Only issued when the controller is not busy.
    output logic          pix_req_valid,
    output logic [9:0]    pix_req_row,
    output logic [9:0]    pix_req_col,
    input  logic          pix_busy,

    // ---- to burst_rd_ctrl ---------------------------------------------------
    output logic          brd_req_valid,
    output logic [9:0]    brd_req_base_row,
    output logic [9:0]    brd_req_base_col,
    output logic [9:0]    brd_req_height,
    output logic [9:0]    brd_req_width,
    input  logic          brd_busy,

    // ---- to register_subsystem ----------------------------------------------
    // Always accepted, so this never stalls the router.
    output logic          rgf_cmd_valid,
    output logic          rgf_cmd_is_write,
    output logic [7:0]    rgf_cmd_addr,
    output logic [31:0]   rgf_cmd_wdata
);

    // -------------------------------------------------------------------
    // Decode. Combinational on the presented message.
    // -------------------------------------------------------------------
    pl_pix_read_t   pr;
    pl_burst_read_t br;
    pl_reg_read_t   rr;
    pl_reg_write_t  rw;

    assign pr = pl_pix_read_t'(msg_payload[$bits(pl_pix_read_t)-1:0]);
    assign br = pl_burst_read_t'(msg_payload[$bits(pl_burst_read_t)-1:0]);
    assign rr = pl_reg_read_t'(msg_payload[$bits(pl_reg_read_t)-1:0]);
    assign rw = pl_reg_write_t'(msg_payload[$bits(pl_reg_write_t)-1:0]);

    // A burst read carries a linear base address; the controller wants row
    // and column. IMG_WIDTH is a power of two so both collapse to wiring;
    // written as arithmetic so a non-power-of-two image stays correct.
    logic [9:0] br_base_row, br_base_col;
    assign br_base_row = 10'(br.base_addr / 24'(IMG_WIDTH));
    assign br_base_col = 10'(br.base_addr % 24'(IMG_WIDTH));

    // -------------------------------------------------------------------
    // Which destination, and is it free
    // -------------------------------------------------------------------
    logic to_write, to_pix, to_brd, to_rgf, to_drop;

    always_comb begin : select
        to_write = 1'b0;
        to_pix   = 1'b0;
        to_brd   = 1'b0;
        to_rgf   = 1'b0;
        to_drop  = 1'b0;

        unique case (msg_kind)
            MSG_PIX_WRITE,
            MSG_BURST_HDR,
            MSG_BURST_DATA : to_write = 1'b1;
            MSG_PIX_READ   : to_pix   = 1'b1;
            MSG_BURST_READ : to_brd   = 1'b1;
            MSG_REG_READ,
            MSG_REG_WRITE  : to_rgf   = 1'b1;
            // MSG_UNKNOWN never reaches here: rx_classifier does not forward
            // it. Dropped rather than stalling the router if it somehow does.
            default        : to_drop  = 1'b1;
        endcase
    end : select

    // The destination decides whether the message can be taken this cycle.
    // A read controller that is busy holds the whole chain, all the way back
    // to the PC, rather than having its request silently discarded.
    logic dest_ready;

    always_comb begin : ready_mux
        unique case (1'b1)
            to_write : dest_ready = wr_msg_ready;
            to_pix   : dest_ready = !pix_busy;
            to_brd   : dest_ready = !brd_busy;
            to_rgf   : dest_ready = 1'b1;     // no back-pressure
            default  : dest_ready = 1'b1;     // dropped
        endcase
    end : ready_mux

    assign msg_ready = dest_ready;

    logic fire;
    assign fire = msg_valid && msg_ready;

    // -------------------------------------------------------------------
    // Dispatch
    //
    // The write path is combinational pass-through: it has its own
    // ready/valid and does its own capture. The strobed destinations are
    // registered so their payload is stable for the cycle the strobe is
    // high.
    // -------------------------------------------------------------------
    assign wr_msg_valid   = msg_valid && to_write;
    assign wr_msg_kind    = msg_kind;
    assign wr_msg_payload = msg_payload;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pix_req_valid    <= 1'b0;
            pix_req_row      <= '0;
            pix_req_col      <= '0;
            brd_req_valid    <= 1'b0;
            brd_req_base_row <= '0;
            brd_req_base_col <= '0;
            brd_req_height   <= '0;
            brd_req_width    <= '0;
            rgf_cmd_valid    <= 1'b0;
            rgf_cmd_is_write <= 1'b0;
            rgf_cmd_addr     <= '0;
            rgf_cmd_wdata    <= '0;
        end else begin
            pix_req_valid <= 1'b0;
            brd_req_valid <= 1'b0;
            rgf_cmd_valid <= 1'b0;

            if (fire) begin
                if (to_pix) begin
                    pix_req_valid <= 1'b1;
                    pix_req_row   <= pr.row;
                    pix_req_col   <= pr.col;
                end

                if (to_brd) begin
                    brd_req_valid    <= 1'b1;
                    brd_req_base_row <= br_base_row;
                    brd_req_base_col <= br_base_col;
                    brd_req_height   <= br.height;
                    brd_req_width    <= br.width;
                end

                if (to_rgf) begin
                    rgf_cmd_valid <= 1'b1;

                    unique case (msg_kind)
                        MSG_REG_READ: begin
                            rgf_cmd_is_write <= 1'b0;
                            rgf_cmd_addr     <= rr.addr;
                            rgf_cmd_wdata    <= '0;
                        end
                        default: begin   // MSG_REG_WRITE
                            rgf_cmd_is_write <= 1'b1;
                            rgf_cmd_addr     <= rw.addr;
                            rgf_cmd_wdata    <= rw.data;
                        end
                    endcase
                end
            end
        end
    end

`ifndef SYNTHESIS
    // A strobe is never issued into a busy controller -- that is the
    // req_overrun path, and it loses the request.
    a_pix_not_busy: assert property (
        @(posedge clk) disable iff (!rst_n)
        pix_req_valid |-> !pix_busy
    ) else $error("%m: pixel read request issued while the controller was busy");

    a_brd_not_busy: assert property (
        @(posedge clk) disable iff (!rst_n)
        brd_req_valid |-> !brd_busy
    ) else $error("%m: burst read request issued while the controller was busy");

    // At most one destination per message.
    a_one_dest: assert property (
        @(posedge clk) disable iff (!rst_n)
        $onehot0({to_write, to_pix, to_brd, to_rgf, to_drop})
    ) else $error("%m: message routed to more than one destination");

    // Strobes are single cycle.
    a_pix_pulse: assert property (
        @(posedge clk) disable iff (!rst_n)
        pix_req_valid |=> !pix_req_valid
    ) else $error("%m: pixel read strobe held for more than one cycle");
`endif

endmodule : mem_msg_router
