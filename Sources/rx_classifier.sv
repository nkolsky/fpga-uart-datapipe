// rx_classifier.sv
// ----------------
// Routes one completed frame to its consumer, and registers the result.
//
// Takes rx_msg_parser's decoded kind plus three raw payload fields, decides
// whether the message is ACCEPTABLE, and emits a one-cycle command strobe on
// the matching channel. Outputs hold until the next accepted frame.
//
// =======================================================================
// WHAT MOVED HERE FROM THE OLD PARSERS
// =======================================================================
// The eight per-message parsers each did two jobs: check frame shape, and
// check whether the payload made sense. rx_msg_parser now owns the first;
// this module owns the second.
//
// That split is deliberate. "Is byte 5 a comma" is a question about the
// FORMAT. "Is this address inside the register window", "is this coordinate
// inside the image", "does this rectangle fit" are questions about what the
// field MEANS -- and meaning depends on which message it is, which is exactly
// what this module knows and the parser does not. A parser that range-checked
// would need the register map and the image geometry, and would stop being a
// parser.
//
// So the semantic checks below are lifted verbatim from the parsers they
// replace:
//
//   Register Read / Write  addr[23:6] == 0 && addr[1:0] == 0
//                          (fits the 6-bit port, word aligned)
//   Single Pixel Read      row < IMG_HEIGHT && col < IMG_WIDTH
//   Image Burst Read       H,W non-zero and within the image;
//                          base address inside the image;
//                          base + extent does not overrun
//   Single Pixel Write     none -- any 24-bit address is structurally legal,
//                          bounds are enforced downstream by sram_wr_ctrl
//   legacy {Rnnn,...}      every payload byte is ASCII '0'..'9'
//
// =======================================================================
// THE LEGACY PATH IS THE ONE MESSAGE WITH ASCII PAYLOAD
// =======================================================================
// Every other message carries raw binary. {Rnnn,Cnnn,Vnnn} carries three
// ASCII decimal digits per field, so it needs both digit validation and a
// decimal-to-binary decode. That conversion used to live in rx_parser and
// now lives here, unchanged: hundreds*100 + tens*10 + ones, each digit byte
// less ASCII_ZERO.
//
// =======================================================================
// ERROR POLICY -- unchanged
// =======================================================================
// classifier_error pulses only for traffic that is malformed or
// unclassifiable. A structurally valid message never errors merely because
// nothing consumes it.
//
//   MSG_LEGACY_RGF  a non-digit payload byte
//   MSG_REG_WRITE   address unusable
//   MSG_REG_READ    address unusable
//   MSG_PIX_READ    out-of-image coordinates
//   MSG_BURST_READ  bad dimensions or an overrunning extent
//   MSG_UNKNOWN     opcodes do not name any message at this length
//   (frame_err)     rx_mac rejected the framing -- passed through directly
//
// MSG_PIX_WRITE can no longer error: its framing is rx_mac's guarantee and
// any 24-bit address is structurally legal.
//
// MSG_BURST_HDR and MSG_BURST_DATA never appear: rx_burst_ctrl consumes both
// directly off rx_mac, and this module gates every output on !burst_active.
//
// NOTE ON THE REGISTER WRITE ERROR TERM. It stays framing-good-but-address-bad
// rather than the complement of the hit, matching the old behaviour. A frame
// that fails framing is dropped without an error pulse, as on the legacy and
// pixel-write paths. Only an address fault is reported, because only an
// address fault is unambiguously a well-formed request the design must refuse.
// The old KNOWN LIMITATION behind that -- rx_msg_decode and the parsers
// disagreeing about which bytes constitute valid framing -- is gone: there is
// now exactly one framing check, in rx_msg_parser, and it verifies every
// opcode position rather than only the deciding ones.

`timescale 1ns/1ps

module rx_classifier
    import msg_format_pkg::*;
    import memory_pkg::*;
(
    input  logic        clk,
    input  logic        rst_n,

    // ---- from rx_mac / rx_msg_parser -----------------------------------
    input  logic        msg_valid,    // rx_mac frame_done, one cycle
    input  logic        frame_err,    // rx_mac framing fault, one cycle
    input  msg_kind_t   msg_kind,     // from rx_msg_parser, combinational
    input  logic [PAYLOAD_W-1:0] field0,
    input  logic [PAYLOAD_W-1:0] field1,
    input  logic [PAYLOAD_W-1:0] field2,

    // ---- from rx_burst_ctrl --------------------------------------------
    // Makes this module completely inert for the duration of a burst.
    input  logic        burst_active,

    // ---- legacy RGF control path ---------------------------------------
    output logic        classifier_valid,
    output logic        classifier_error,
    output logic [9:0]  row_q,
    output logic [9:0]  col_q,
    output logic [PAYLOAD_W-1:0] pixel_q,

    // ---- Single Pixel Write -> command FIFO ----------------------------
    output logic        cmd_valid,
    output logic [BURST_ADDR_W-1:0] cmd_addr,
    output logic [BURST_PIX_W-1:0]  cmd_pixel,

    // ---- Register Read -------------------------------------------------
    output logic        rr_cmd_valid,
    output logic [rgf_pkg::ADDR_WIDTH-1:0] rr_cmd_addr,

    // ---- Register Write ------------------------------------------------
    output logic        rw_cmd_valid,
    output logic [rgf_pkg::ADDR_WIDTH-1:0] rw_cmd_addr,
    output logic [rgf_pkg::DATA_WIDTH-1:0] rw_cmd_data,

    // ---- Single Pixel Read ---------------------------------------------
    output logic        pr_cmd_valid,
    output logic [9:0]  pr_cmd_row,
    output logic [9:0]  pr_cmd_col,

    // ---- Image Burst Read ----------------------------------------------
    output logic        br_cmd_valid,
    output logic [9:0]  br_cmd_base_row,
    output logic [9:0]  br_cmd_base_col,
    output logic [9:0]  br_cmd_height,
    output logic [9:0]  br_cmd_width
);

    localparam int TOTAL_PIXELS = IMG_HEIGHT * IMG_WIDTH;   // 65536

    // =================================================================
    // SEMANTIC CHECKS
    // =================================================================

    // ---- register address: fits the RGF port, word aligned ------------
    //
    // The test is "does this fit the destination", nothing more. Whether a
    // given address is a POPULATED register is the RGF's business -- its read
    // decode already has a default arm. Width comes from rgf_pkg so the check
    // cannot drift from the port it is checking against; it was previously a
    // bare 6, unrelated to anything.
    logic rgf_addr_ok;
    assign rgf_addr_ok = (field0[PAYLOAD_W-1:rgf_pkg::ADDR_WIDTH] == '0) &&
                         (field0[1:0] == 2'd0);

    // ---- pixel coordinates inside the image ---------------------------
    logic pix_coord_ok;
    assign pix_coord_ok = (field0 < PAYLOAD_W'(IMG_HEIGHT)) &&
                          (field1 < PAYLOAD_W'(IMG_WIDTH));

    // ---- burst read: dimensions, base address, extent ------------------
    // Decomposed at full width so the sum cannot wrap. IMG_WIDTH is a power
    // of two so both of these collapse to wiring; written as arithmetic so a
    // non-power-of-two image stays correct.
    logic [PAYLOAD_W-1:0] br_base_row_full, br_base_col_full;
    logic br_dims_ok, br_addr_ok, br_extent_ok, br_ok;

    assign br_base_row_full = field0 / PAYLOAD_W'(IMG_WIDTH);
    assign br_base_col_full = field0 % PAYLOAD_W'(IMG_WIDTH);

    assign br_dims_ok   = (field1 != '0) && (field1 <= PAYLOAD_W'(IMG_HEIGHT)) &&
                          (field2 != '0) && (field2 <= PAYLOAD_W'(IMG_WIDTH));
    assign br_addr_ok   = (field0 < PAYLOAD_W'(TOTAL_PIXELS));
    assign br_extent_ok = ((br_base_row_full + field1) <= PAYLOAD_W'(IMG_HEIGHT)) &&
                          ((br_base_col_full + field2) <= PAYLOAD_W'(IMG_WIDTH));
    assign br_ok        = br_dims_ok && br_addr_ok && br_extent_ok;

    // ---- legacy: ASCII digits ------------------------------------------
    // Payload is nine ASCII bytes: field0 = row digits, field1 = col,
    // field2 = value, each most-significant digit first.
    function automatic logic is_digit(input logic [7:0] b);
        return (b >= ASCII_ZERO) && (b <= (ASCII_ZERO + 8'd9));
    endfunction

    function automatic int dec3(input logic [PAYLOAD_W-1:0] f);
        return (int'(f[23:16]) - int'(ASCII_ZERO)) * 100
             + (int'(f[15: 8]) - int'(ASCII_ZERO)) * 10
             + (int'(f[ 7: 0]) - int'(ASCII_ZERO));
    endfunction

    logic legacy_digits_ok;
    always_comb begin : check_digits
        legacy_digits_ok = 1'b1;
        for (int g = 0; g < 3; g++) begin
            logic [PAYLOAD_W-1:0] f;
            f = (g == 0) ? field0 : (g == 1) ? field1 : field2;
            if (!is_digit(f[23:16])) legacy_digits_ok = 1'b0;
            if (!is_digit(f[15: 8])) legacy_digits_ok = 1'b0;
            if (!is_digit(f[ 7: 0])) legacy_digits_ok = 1'b0;
        end
    end : check_digits

    logic [9:0]  legacy_row, legacy_col;
    logic [PAYLOAD_W-1:0] legacy_pixel;
    assign legacy_row   = 10'(dec3(field0));
    assign legacy_col   = 10'(dec3(field1));
    assign legacy_pixel = PAYLOAD_W'(dec3(field2));

    // =================================================================
    // KIND-QUALIFIED HIT / ERROR TERMS
    // =================================================================
    logic legacy_hit,  legacy_err;
    logic pixwr_hit,   pixwr_err;
    logic regrd_hit,   regrd_err;
    logic regwr_hit,   regwr_err;
    logic pixrd_hit,   pixrd_err;
    logic burstrd_hit, burstrd_err;
    logic unknown_msg;

    always_comb begin : qualify
        // `active` is the ONLY frame-acceptance condition. During a burst this
        // module emits nothing at all -- no command, no strobe, no error.
        // rx_burst_ctrl owns every frame for the duration.
        automatic logic active = msg_valid && !burst_active;

        // No frame_ok term anywhere. A frame only reaches this module if
        // rx_mac accepted its framing, so every test below is SEMANTIC.
        legacy_hit  = active && (msg_kind == MSG_LEGACY_RGF) &&  legacy_digits_ok;
        legacy_err  = active && (msg_kind == MSG_LEGACY_RGF) && !legacy_digits_ok;

        // Any 24-bit address is structurally legal; bounds are enforced
        // downstream by sram_wr_ctrl. So a pixel write cannot be rejected.
        pixwr_hit   = active && (msg_kind == MSG_PIX_WRITE);
        pixwr_err   = 1'b0;

        regrd_hit   = active && (msg_kind == MSG_REG_READ)  &&  rgf_addr_ok;
        regrd_err   = active && (msg_kind == MSG_REG_READ)  && !rgf_addr_ok;

        regwr_hit   = active && (msg_kind == MSG_REG_WRITE) &&  rgf_addr_ok;
        regwr_err   = active && (msg_kind == MSG_REG_WRITE) && !rgf_addr_ok;

        pixrd_hit   = active && (msg_kind == MSG_PIX_READ)  &&  pix_coord_ok;
        pixrd_err   = active && (msg_kind == MSG_PIX_READ)  && !pix_coord_ok;

        burstrd_hit = active && (msg_kind == MSG_BURST_READ) &&  br_ok;
        burstrd_err = active && (msg_kind == MSG_BURST_READ) && !br_ok;

        unknown_msg = active && (msg_kind == MSG_UNKNOWN);
    end : qualify

    // =================================================================
    // LATCHED FIELDS
    // =================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            row_q   <= 10'd0;
            col_q   <= 10'd0;
            pixel_q <= '0;
        end else if (legacy_hit) begin
            row_q   <= legacy_row;
            col_q   <= legacy_col;
            pixel_q <= legacy_pixel;
        end
    end

    // Single Pixel Write: field0 = address, field1 = {R,G,B}
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cmd_addr  <= '0;
            cmd_pixel <= '0;
        end else if (pixwr_hit) begin
            cmd_addr  <= field0;
            cmd_pixel <= field1;
        end
    end

    // =================================================================
    // REGISTERED ONE-CYCLE OUTPUT PULSES
    //
    // Fields are written on a HIT only, so a rejected request cannot leave a
    // stale-but-plausible value behind for the next producer to trip over.
    // =================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            classifier_valid <= 1'b0;
            classifier_error <= 1'b0;
            cmd_valid        <= 1'b0;
            rr_cmd_valid     <= 1'b0;
            rr_cmd_addr      <= '0;
            rw_cmd_valid     <= 1'b0;
            rw_cmd_addr      <= '0;
            rw_cmd_data      <= '0;
            pr_cmd_valid     <= 1'b0;
            pr_cmd_row       <= 10'd0;
            pr_cmd_col       <= 10'd0;
            br_cmd_valid     <= 1'b0;
            br_cmd_base_row  <= 10'd0;
            br_cmd_base_col  <= 10'd0;
            br_cmd_height    <= 10'd0;
            br_cmd_width     <= 10'd0;
        end else begin
            classifier_valid <= legacy_hit;
            // frame_err is NOT gated on !burst_active: a framing fault is
            // worth reporting whatever mode the receiver is in, and during a
            // burst it is the only signal a malformed data frame produces.
            classifier_error <= legacy_err || regrd_err  || regwr_err ||
                                pixrd_err  || burstrd_err || unknown_msg ||
                                frame_err;
            cmd_valid        <= pixwr_hit;

            rr_cmd_valid     <= regrd_hit;
            if (regrd_hit) rr_cmd_addr <= field0[rgf_pkg::ADDR_WIDTH-1:0];

            // Register Write data is bytes 8,9,13,14 -- the low two payload
            // bytes of each V group. The don't-care bytes the spec shows as
            // 0 are field[23:16] and are simply not used.
            rw_cmd_valid     <= regwr_hit;
            if (regwr_hit) begin
                rw_cmd_addr <= field0[rgf_pkg::ADDR_WIDTH-1:0];
                rw_cmd_data <= {field1[(rgf_pkg::DATA_WIDTH/2)-1:0],
                                field2[(rgf_pkg::DATA_WIDTH/2)-1:0]};
            end

            pr_cmd_valid <= pixrd_hit;
            if (pixrd_hit) begin
                pr_cmd_row <= field0[9:0];
                pr_cmd_col <= field1[9:0];
            end

            br_cmd_valid <= burstrd_hit;
            if (burstrd_hit) begin
                br_cmd_base_row <= 10'(br_base_row_full);
                br_cmd_base_col <= 10'(br_base_col_full);
                br_cmd_height   <= field1[9:0];
                br_cmd_width    <= field2[9:0];
            end
        end
    end

// -------------------------------------------------------------------------
// MUTUAL-EXCLUSION CONTRACT WITH rx_burst_ctrl
// -------------------------------------------------------------------------
// Two modules can drive a command towards the shared command FIFO: this one
// (Single Pixel Write) and rx_burst_ctrl (Image Burst Write). They are
// mutually exclusive, and the exclusion rests on two independent layers:
//
//   STRUCTURAL  While rx_burst_ctrl asserts bypass_active, rx_msg_parser
//               classifies EVERY frame as MSG_BURST_DATA. MSG_PIX_WRITE and
//               MSG_LEGACY_RGF therefore cannot occur, so this module would
//               emit nothing even without the gate.
//
//   EXPLICIT    Every output is additionally gated on !burst_active, so the
//               exclusion survives a mis-wired bypass_active.
//
// The second layer is not redundant. Burst payload bytes are RAW BINARY, so a
// data frame can legitimately carry 'W' at byte 1 and 'P' at byte 6 -- a
// perfect Single Pixel Write -- or 'R'/'C'/'V' at bytes 1/6/11, a perfect
// legacy RGF command. If bypass_active ever failed, ordinary image data would
// start issuing register writes, and a frame that happened to address IMG_CTRL
// would launch an image read in the middle of a burst.
// -------------------------------------------------------------------------
`ifndef SYNTHESIS
    a_one_rgf_producer: assert property (
        @(posedge clk) disable iff (!rst_n)
        $onehot0({classifier_valid, rr_cmd_valid, rw_cmd_valid})
    ) else $error("%m: more than one RGF command producer valid in the same cycle");

    a_regwr_hit_xor_err: assert property (
        @(posedge clk) disable iff (!rst_n)
        !(regwr_hit && regwr_err)
    ) else $error("%m: register write both accepted and rejected");

    a_inert_during_burst: assert property (
        @(posedge clk) disable iff (!rst_n)
        burst_active |-> (!classifier_valid && !cmd_valid && !pr_cmd_valid &&
                          !br_cmd_valid && !rr_cmd_valid && !rw_cmd_valid)
    ) else $error("%m: classifier active during a burst");

    a_burstrd_exclusive: assert property (
        @(posedge clk) disable iff (!rst_n)
        br_cmd_valid |-> (!classifier_valid && !rr_cmd_valid &&
                          !rw_cmd_valid && !cmd_valid && !pr_cmd_valid)
    ) else $error("%m: burst read command overlapped another command");

    a_burstrd_hit_xor_err: assert property (
        @(posedge clk) disable iff (!rst_n)
        !(burstrd_hit && burstrd_err)
    ) else $error("%m: burst read both accepted and rejected");

    a_pixrd_exclusive: assert property (
        @(posedge clk) disable iff (!rst_n)
        pr_cmd_valid |-> (!classifier_valid && !rr_cmd_valid &&
                          !rw_cmd_valid && !cmd_valid)
    ) else $error("%m: pixel read command overlapped another command");

    a_pixrd_hit_xor_err: assert property (
        @(posedge clk) disable iff (!rst_n)
        !(pixrd_hit && pixrd_err)
    ) else $error("%m: pixel read both accepted and rejected");

    // The structural layer, checked rather than assumed. If this fires,
    // bypass_active is mis-wired and only the explicit gate is protecting
    // the design.
    a_no_classify_during_burst: assert property (
        @(posedge clk) disable iff (!rst_n)
        (burst_active && msg_valid) |->
            (msg_kind != MSG_PIX_WRITE && msg_kind != MSG_LEGACY_RGF)
    ) else $error("%m: frame classified as an action type during a burst -- bypass_active mis-wired?");
`endif

endmodule : rx_classifier
