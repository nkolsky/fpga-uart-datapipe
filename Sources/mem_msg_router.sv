// mem_msg_router.sv
// =================
// Dispatches each incoming message to the correct memory-side destination and
// holds the message until that destination is ready.
//
//                        +-> write path
//   msg -> mem_msg_router+-> pixel_rd_ctrl
//                        +-> burst_rd_ctrl
//                        +-> register file

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
    // Issued to apb_master, which converts it into one APB transfer. No
    // longer "always accepted": apb_busy gates dest_ready below.
    output logic          rgf_cmd_valid,
    output logic          rgf_cmd_is_write,
    output logic [7:0]    rgf_cmd_addr,
    output logic [31:0]   rgf_cmd_wdata,
    // From apb_master. High from acceptance until the transfer completes.
    input  logic          apb_busy
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

    // The destination decides whether the message can be taken this cycle. If a
    // read controller is busy, the router holds the message instead of dropping
    // it and silently losing work.
    logic dest_ready;

    always_comb begin : ready_mux
        unique case (1'b1)
            to_write : dest_ready = wr_msg_ready;
            to_pix   : dest_ready = !pix_busy;
            to_brd   : dest_ready = !brd_busy;
            // The register file used to be the one destination assumed
            // always ready. It sits behind APB now: a transfer occupies the
            // bus for at least two cycles, so the master's busy has to be
            // honoured or a second register message would be dropped.
            //
            // Free in rate terms -- 20 ns against a 22 us message -- but the
            // stall makes the path correct by construction rather than
            // correct by arithmetic.
            to_rgf   : dest_ready = !apb_busy;
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
            // Plain '0 is fine now. This used to be IDLE_ADDR because 8'h00
            // is IMG_STATUS and a held address was a live decode; pc_ren
            // qualifies the access in rgf, so the reset value is arbitrary.
            rgf_cmd_addr     <= '0;
            rgf_cmd_wdata    <= '0;
        end else begin
            pix_req_valid <= 1'b0;
            brd_req_valid <= 1'b0;
            rgf_cmd_valid <= 1'b0;

            // ADDRESS PARKING IS GONE.
            //
            // rgf's IMG_TX_MON read-to-clear used to be a level decode with no
            // access qualifier, so whoever drove pc_addr had to park it at an
            // address that decoded to nothing. That obligation had already
            // moved here from cdc_cmd_sync.
            //
            // rgf now takes a real one-cycle pc_ren from apb_slave_rgf, so the
            // address may sit wherever it likes between transfers. The hazard
            // is deleted rather than re-homed.

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
