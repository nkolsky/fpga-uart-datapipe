// tb_stage2c_pipeline.sv
// ----------------------
// DEMONSTRATION TESTBENCH -- one pixel, end to end, through Stage 2C.
//
// Purpose is a clean presentable waveform, not coverage. It injects exactly
// ONE Single Pixel Write and follows it through every stage of the datapath,
// then reads the image back and proves the pixel changed.
//
//   UART frame   7B 57 00 04 02 2C 50 FF A5 00 7D
//   pixel index  1026  (24'h000402)   -> row 4, col 2
//   colour       (255,165,0) orange   (24'hFFA500)
//
// Orange is deliberate: a monochrome value cannot distinguish the three
// channel bytes, so a transposed R/G/B split would go unnoticed. With
// (255,165,0) each channel is distinct and any swap is visible.
//
// -----------------------------------------------------------------------
// WHY PIXEL 1026 IS A GOOD DEMONSTRATION
// -----------------------------------------------------------------------
//   1026 / 4 = 256   -> word address 256, a NON-ZERO address. Pixel 0 would
//                       have every address bit zero, so an address that was
//                       being dropped entirely would still appear correct.
//   1026 % 4 = 2     -> byte lane 2, which maps to wr_be[1] and word bits
//                       [15:8]. Lane 0 (the reversed end of the shift) alone
//                       would not prove the mapping.
//
// Word 256 of all three channels is 0xFFFFFFFF in the source image, so:
//
//   red   0xFFFFFFFF -> 0xFFFFFFFF   (R=0xFF, coincidentally unchanged)
//   green 0xFFFFFFFF -> 0xFFFFA5FF   (byte 1 becomes 0xA5)
//   blue  0xFFFFFFFF -> 0xFFFF00FF   (byte 1 becomes 0x00)
//
// Green and blue change visibly and in the same byte position, which makes
// the byte-lane selection obvious on screen.
//
// -----------------------------------------------------------------------
// WAVEFORM GROUPS -- add these in this order for the presentation
// -----------------------------------------------------------------------
//
//   [ 1 ] Clocks / control
//         clk_rx  clk_mem  rst_n  wr_allowed  read_go
//
//   [ 2 ] Incoming command            <-- pixel enters the design here
//         byte_valid  rx_byte  msg_valid  msg_kind_q
//         rx_cmd_valid  rx_cmd_addr  rx_cmd_pixel
//
//   [ 3 ] Command FIFO (130 -> 100 MHz crossing)
//         cmd_fifo_wr_en  cmd_fifo_wr_data
//         cmd_fifo_empty  cmd_fifo_rd_en  cmd_fifo_rd_data
//
//   [ 4 ] SRAM write controller       <-- address mapping happens here
//         u_wr_ctrl.pop_q
//         sram_wr_en  sram_wr_addr  sram_wr_be
//         sram_wr_data_r  sram_wr_data_g  sram_wr_data_b
//
//   [ 5 ] RGB SRAM contents
//         u_sram_red.mem[256]  u_sram_green.mem[256]  u_sram_blue.mem[256]
//
//   [ 6 ] Readback through rom_sequencer
//         rom_rd_en  rom_addr  red_data  green_data  blue_data
//         u_rom_seq.pixels[0] .. [3]
//         pix_fifo_wr_en  pix_fifo_wr_data
//         pix_fifo_rd_en  pix_fifo_rd_data  pop_index
//
// Radix: hexadecimal for all data buses, binary for sram_wr_be, unsigned
// decimal for sram_wr_addr and pop_index.
//
// -----------------------------------------------------------------------
// SCOPE
// -----------------------------------------------------------------------
// rx_phy is omitted; bytes are injected at rx_mac's byte interface. The
// serial line adds ~15 us and 121 bit transitions to the waveform without
// showing anything about Stage 2C. Everything from rx_mac onward is the real
// design, unmodified.
//
// rom_sequencer is instantiated with ROM_DEPTH = 260 rather than 16384 so
// the readback walk reaches word 256 in ~25 us instead of ~1.5 s. The SRAMs
// remain full depth and initialised from the real .mem files, so the data
// and the addressing are unchanged -- only the length of the walk differs.

`timescale 1ns/1ps

module tb_stage2c_pipeline;

    import msg_pkg::*;
    import rx_msg_pkg::*;
    import memory_pkg::*;

    // -----------------------------------------------------------------
    // Stimulus constants
    // -----------------------------------------------------------------
    localparam logic [23:0] PIX_ADDR   = 24'h00_0402;   // pixel index 1026
    localparam logic [23:0] PIX_COLOUR = 24'hFF_A500;   // orange
    localparam logic [47:0] EXP_CMD    = {PIX_ADDR, PIX_COLOUR};

    localparam int          EXP_WORD   = 1026 / 4;      // 256
    localparam logic [3:0]  EXP_BE     = 4'b0010;       // lane 2 -> bits[15:8]
    localparam logic [31:0] EXP_WDATA_R = {4{PIX_COLOUR[23:16]}};  // FFFFFFFF
    localparam logic [31:0] EXP_WDATA_G = {4{PIX_COLOUR[15: 8]}};  // A5A5A5A5
    localparam logic [31:0] EXP_WDATA_B = {4{PIX_COLOUR[ 7: 0]}};  // 00000000

    localparam int ROM_WORDS = 260;                     // shortened walk
    localparam int SEQ_AW    = $clog2(ROM_WORDS);       // 9

    // -----------------------------------------------------------------
    // Clocks and reset
    // -----------------------------------------------------------------
    logic clk_rx  = 1'b0;   // 130 MHz receive domain
    logic clk_mem = 1'b0;   // 100 MHz memory domain
    always #3.846 clk_rx  = ~clk_rx;
    always #5.000 clk_mem = ~clk_mem;

    logic rst_n;

    // =================================================================
    // [ 2 ] RECEIVE PATH -- the pixel enters the design here
    // =================================================================
    logic       byte_valid = 1'b0;
    logic [7:0] rx_byte    = 8'h00;

    logic [BYTE_CNT_W-1:0] expected_len, byte_cnt;
    msg_kind_t             msg_kind_prov, msg_kind_q;
    logic                  kind_known;
    logic [127:0]          frame_buf, msg_data;
    logic                  msg_valid, mac_busy;

    logic        parse_valid, parse_error;
    logic [9:0]  row, col;
    logic [23:0] pixel;

    logic        pw_parse_valid, pw_parse_error;
    logic [23:0] pw_addr, pw_pixel;

    logic        classifier_valid, classifier_error;
    logic [9:0]  row_q, col_q;
    logic [23:0] pixel_q;
    logic        rx_cmd_valid;
    logic [23:0] rx_cmd_addr, rx_cmd_pixel;
    logic        bypass_active;

    rx_msg_decode u_decode (
        .frame_buf(frame_buf), .byte_cnt(byte_cnt), .bypass_active(bypass_active),
        .expected_len(expected_len), .msg_kind_prov(msg_kind_prov),
        .kind_known(kind_known));

    rx_mac u_mac (
        .clk(clk_rx), .rst_n(rst_n),
        .byte_valid(byte_valid), .rx_byte(rx_byte), .par_val_rst(1'b0),
        .expected_len(expected_len), .msg_kind_prov(msg_kind_prov),
        .frame_buf(frame_buf), .byte_cnt(byte_cnt),
        .msg_valid(msg_valid), .msg_data(msg_data),
        .msg_kind_q(msg_kind_q), .mac_busy(mac_busy));

    rx_parser u_parser (
        .msg_in(msg_data), .parse_valid(parse_valid), .parse_error(parse_error),
        .row(row), .col(col), .pixel(pixel));

    rx_pixel_wr_parser u_pw_parser (
        .msg_in(msg_data),
        .pw_parse_valid(pw_parse_valid), .pw_parse_error(pw_parse_error),
        .pw_addr(pw_addr), .pw_pixel(pw_pixel));

    rx_classifier u_classifier (
        .clk(clk_rx), .rst_n(rst_n),
        .msg_valid(msg_valid), .msg_kind(msg_kind_q),
        .parse_valid(parse_valid), .parse_error(parse_error),
        .row(row), .col(col), .pixel(pixel),
        .pw_parse_valid(pw_parse_valid), .pw_parse_error(pw_parse_error),
        .pw_addr(pw_addr), .pw_pixel(pw_pixel),
        .classifier_valid(classifier_valid), .classifier_error(classifier_error),
        .row_q(row_q), .col_q(col_q), .pixel_q(pixel_q),
        .cmd_valid(rx_cmd_valid), .cmd_addr(rx_cmd_addr), .cmd_pixel(rx_cmd_pixel),
        .bypass_active(bypass_active));

    // =================================================================
    // [ 3 ] COMMAND FIFO -- 130 MHz -> 100 MHz, carries the whole command
    // =================================================================
    localparam int CMD_FIFO_W = 48;

    logic                  cmd_fifo_wr_en;
    logic [CMD_FIFO_W-1:0] cmd_fifo_wr_data;
    logic                  cmd_fifo_full, cmd_fifo_empty, cmd_fifo_rd_en;
    logic [CMD_FIFO_W-1:0] cmd_fifo_rd_data;

    assign cmd_fifo_wr_en   = rx_cmd_valid;
    assign cmd_fifo_wr_data = {rx_cmd_addr, rx_cmd_pixel};

    async_fifo #(.DW(CMD_FIFO_W)) u_cmd_fifo (
        .wr_clk(clk_rx),  .wr_rst_n(rst_n),
        .wr_en(cmd_fifo_wr_en), .wr_data(cmd_fifo_wr_data),
        .full(cmd_fifo_full), .almost_full(),
        .rd_clk(clk_mem), .rd_rst_n(rst_n),
        .rd_en(cmd_fifo_rd_en), .rd_data(cmd_fifo_rd_data),
        .empty(cmd_fifo_empty), .almost_empty());

    // =================================================================
    // [ 4 ] SRAM WRITE CONTROLLER -- address mapping happens here
    // =================================================================
    logic                          sram_wr_en;
    logic [SRAM_DATA_WIDTH/8-1:0]  sram_wr_be;
    logic [SRAM_ADDR_WIDTH-1:0]    sram_wr_addr;
    logic [SRAM_DATA_WIDTH-1:0]    sram_wr_data_r, sram_wr_data_g, sram_wr_data_b;
    logic                          sram_wr_seen, sram_wr_rejected;
    logic                          sram_wr_allowed, sram_wr_busy;

    sram_wr_ctrl #(.CMD_W(CMD_FIFO_W)) u_wr_ctrl (
        .clk(clk_mem), .rst_n(rst_n),
        .cmd_empty(cmd_fifo_empty), .cmd_rd_data(cmd_fifo_rd_data),
        .cmd_rd_en(cmd_fifo_rd_en),
        .wr_allowed(sram_wr_allowed), .wr_busy(sram_wr_busy),
        .wr_en(sram_wr_en), .wr_be(sram_wr_be), .wr_addr(sram_wr_addr),
        .wr_data_r(sram_wr_data_r), .wr_data_g(sram_wr_data_g),
        .wr_data_b(sram_wr_data_b),
        .wr_seen(sram_wr_seen), .wr_rejected(sram_wr_rejected));

    // -----------------------------------------------------------------
    // [ 1 ] Interlock -- keeps the write port and the read port exclusive
    // -----------------------------------------------------------------
    logic start_req = 1'b0;
    logic img_done  = 1'b0;
    logic read_go, rom_seq_busy;

    mem_interlock u_interlock (
        .clk(clk_mem), .rst_n(rst_n),
        .start_req(start_req), .rom_seq_busy(rom_seq_busy), .img_done(img_done),
        .read_go(read_go),
        .cmd_empty(cmd_fifo_empty), .wr_busy(sram_wr_busy),
        .wr_allowed(sram_wr_allowed));

    // =================================================================
    // [ 5 ] RGB SRAMs -- full depth, initialised from the real image
    // =================================================================
    logic [SRAM_ADDR_WIDTH-1:0] rom_addr_full;
    logic [SEQ_AW-1:0]          rom_addr;
    logic                       rom_rd_en;
    logic [31:0]                red_data, green_data, blue_data;

    assign rom_addr_full = SRAM_ADDR_WIDTH'(rom_addr);   // 9 -> 14 bits

    rgb_sram #(.INIT_FILE("red_hex.mem")) u_sram_red (
        .clk(clk_mem),
        .rd_en(rom_rd_en), .rd_addr(rom_addr_full), .rd_data(red_data),
        .wr_en(sram_wr_en), .wr_be(sram_wr_be), .wr_addr(sram_wr_addr),
        .wr_data(sram_wr_data_r));

    rgb_sram #(.INIT_FILE("green_hex.mem")) u_sram_green (
        .clk(clk_mem),
        .rd_en(rom_rd_en), .rd_addr(rom_addr_full), .rd_data(green_data),
        .wr_en(sram_wr_en), .wr_be(sram_wr_be), .wr_addr(sram_wr_addr),
        .wr_data(sram_wr_data_g));

    rgb_sram #(.INIT_FILE("blue_hex.mem")) u_sram_blue (
        .clk(clk_mem),
        .rd_en(rom_rd_en), .rd_addr(rom_addr_full), .rd_data(blue_data),
        .wr_en(sram_wr_en), .wr_be(sram_wr_be), .wr_addr(sram_wr_addr),
        .wr_data(sram_wr_data_b));

    // =================================================================
    // [ 6 ] READBACK -- rom_sequencer reassembles pixels, pixel FIFO carries
    //       them to the transmit domain
    // =================================================================
    logic        pix_fifo_wr_en;
    logic [23:0] pix_fifo_wr_data;
    logic        pix_fifo_full, pix_fifo_almost_full;
    logic        pix_fifo_empty, pix_fifo_almost_empty;
    logic        pix_fifo_rd_en = 1'b0;
    logic [23:0] pix_fifo_rd_data;
    logic        seq_done;

    rom_sequencer #(.ROM_DEPTH(ROM_WORDS)) u_rom_seq (
        .clk(clk_mem), .rst_n(rst_n),
        .start(read_go),
        .almost_full(pix_fifo_almost_full), .almost_empty(pix_fifo_almost_empty),
        .red_data(red_data), .green_data(green_data), .blue_data(blue_data),
        .rom_addr(rom_addr), .rom_rd_en(rom_rd_en),
        .wr_en(pix_fifo_wr_en), .wr_data(pix_fifo_wr_data),
        .seq_done(seq_done), .busy(rom_seq_busy));

    async_fifo u_pix_fifo (
        .wr_clk(clk_mem), .wr_rst_n(rst_n),
        .wr_en(pix_fifo_wr_en), .wr_data(pix_fifo_wr_data),
        .full(pix_fifo_full), .almost_full(pix_fifo_almost_full),
        .rd_clk(clk_rx), .rd_rst_n(rst_n),
        .rd_en(pix_fifo_rd_en), .rd_data(pix_fifo_rd_data),
        .empty(pix_fifo_empty), .almost_empty(pix_fifo_almost_empty));

    // =================================================================
    // Scoreboard
    // =================================================================
    int    errors = 0;
    int    checks = 0;
    string stage  = "init";

    task automatic chk(input bit cond, input string what);
        checks++;
        if (!cond) begin
            errors++;
            $display("  [%0t] FAIL (%s): %s", $time, stage, what);
        end
    endtask

    task automatic chk_h(input logic [63:0] got, input logic [63:0] exp,
                         input int w, input string what);
        checks++;
        if (got !== exp) begin
            errors++;
            $display("  [%0t] FAIL (%s): %s -- got %0h, expected %0h",
                     $time, stage, what, got, exp);
        end else begin
            $display("        ok  %-42s = %0h", what, got);
        end
    endtask

    // ---- capture the write path as it happens ------------------------
    logic [47:0] cap_cmd_wr, cap_cmd_rd;
    logic [13:0] cap_wr_addr;
    logic [3:0]  cap_wr_be;
    logic [31:0] cap_wr_r, cap_wr_g, cap_wr_b;
    int          n_cmd_wr = 0, n_cmd_rd = 0, n_sram_wr = 0, n_rx_cmd = 0;
    logic [23:0] cap_rx_addr, cap_rx_pixel;

    always @(posedge clk_rx) if (rst_n && rx_cmd_valid) begin
        n_rx_cmd++;  cap_rx_addr = rx_cmd_addr;  cap_rx_pixel = rx_cmd_pixel;
    end
    always @(posedge clk_rx) if (rst_n && cmd_fifo_wr_en) begin
        n_cmd_wr++;  cap_cmd_wr = cmd_fifo_wr_data;
    end
    always @(posedge clk_mem) if (rst_n && cmd_fifo_rd_en && !cmd_fifo_empty) begin
        n_cmd_rd++;  cap_cmd_rd = cmd_fifo_rd_data;
    end
    always @(posedge clk_mem) if (rst_n && sram_wr_en) begin
        n_sram_wr++;
        cap_wr_addr = sram_wr_addr;  cap_wr_be = sram_wr_be;
        cap_wr_r = sram_wr_data_r;   cap_wr_g = sram_wr_data_g;
        cap_wr_b = sram_wr_data_b;
    end

    // ---- readback capture --------------------------------------------
    int          pop_index = 0;
    logic [23:0] pop_1024, pop_1025, pop_1026, pop_1027;
    logic        pop_q;

    always @(posedge clk_rx or negedge rst_n) begin
        if (!rst_n) pop_q <= 1'b0;
        else        pop_q <= pix_fifo_rd_en && !pix_fifo_empty;
    end

    always @(posedge clk_rx) begin
        if (rst_n && pop_q) begin
            case (pop_index)
                1024: pop_1024 = pix_fifo_rd_data;
                1025: pop_1025 = pix_fifo_rd_data;
                1026: pop_1026 = pix_fifo_rd_data;   // <-- the written pixel
                1027: pop_1027 = pix_fifo_rd_data;
                default: ;
            endcase
            pop_index++;
        end
    end

    // Drain the pixel FIFO continuously so rom_sequencer never throttles.
    always @(negedge clk_rx) pix_fifo_rd_en = !pix_fifo_empty;

    // =================================================================
    // Stimulus
    // =================================================================
    task automatic send_byte(input logic [7:0] b);
        @(negedge clk_rx);
        rx_byte = b;  byte_valid = 1'b1;
        @(negedge clk_rx);
        byte_valid = 1'b0;
        repeat (6) @(negedge clk_rx);
    endtask

    // The exact 11 bytes sent from the host.
    task automatic send_pixel_write();
        send_byte(8'h7B);  // '{'
        send_byte(8'h57);  // 'W'
        send_byte(8'h00);  // A2
        send_byte(8'h04);  // A1
        send_byte(8'h02);  // A0   -> 24'h000402 = 1026
        send_byte(8'h2C);  // ','
        send_byte(8'h50);  // 'P'
        send_byte(8'hFF);  // R
        send_byte(8'hA5);  // G
        send_byte(8'h00);  // B
        send_byte(8'h7D);  // '}'
    endtask

    initial begin
        $display("=================================================");
        $display(" Stage 2C -- one pixel, end to end");
        $display("=================================================");
        $display(" frame  : 7B 57 00 04 02 2C 50 FF A5 00 7D");
        $display(" pixel  : index 1026 (row 4, col 2)");
        $display(" colour : (255,165,0) orange");
        $display(" maps to: word %0d, byte lane 2, wr_be 4'b0010", EXP_WORD);
        $display("-------------------------------------------------");

        rst_n = 1'b0;
        repeat (8) @(negedge clk_mem);
        rst_n = 1'b1;
        repeat (8) @(negedge clk_mem);

        // -------------------------------------------------------------
        stage = "0-before";
        // The image as loaded from the .mem files. Pixels 1024..1027 are
        // all white, so word 256 is 0xFFFFFFFF in every channel.
        // -------------------------------------------------------------
        $display("  [0] SRAM word %0d before the write:", EXP_WORD);
        chk_h(u_sram_red.mem[EXP_WORD],   32'hFFFFFFFF, 32, "red   mem[256] initial");
        chk_h(u_sram_green.mem[EXP_WORD], 32'hFFFFFFFF, 32, "green mem[256] initial");
        chk_h(u_sram_blue.mem[EXP_WORD],  32'hFFFFFFFF, 32, "blue  mem[256] initial");

        // -------------------------------------------------------------
        stage = "1-receive";
        // STAGE 1: the 11-byte frame arrives. rx_mac frames it using the
        // length decoded by rx_msg_decode, rx_pixel_wr_parser splits out the
        // address and colour, rx_classifier latches them.
        // -------------------------------------------------------------
        $display("  [1] injecting the UART frame ...");
        send_pixel_write();
        repeat (40) @(negedge clk_rx);

        chk_h(n_rx_cmd, 1, 32, "exactly one command classified");
        chk_h(cap_rx_addr,  PIX_ADDR,   24, "rx_cmd_addr");
        chk_h(cap_rx_pixel, PIX_COLOUR, 24, "rx_cmd_pixel");
        chk(msg_kind_q == MSG_PIX_WRITE, "msg_kind_q == MSG_PIX_WRITE");

        // -------------------------------------------------------------
        stage = "2-cmd-fifo";
        // STAGE 2: the whole 48-bit command crosses 130 -> 100 MHz as one
        // atomic FIFO entry. Address in [47:24], colour in [23:0].
        // -------------------------------------------------------------
        chk_h(n_cmd_wr, 1, 32, "one command FIFO write");
        chk_h(cap_cmd_wr, EXP_CMD, 48, "cmd_fifo_wr_data");
        repeat (40) @(negedge clk_mem);
        chk_h(n_cmd_rd, 1, 32, "one command FIFO read");
        chk_h(cap_cmd_rd, EXP_CMD, 48, "cmd_fifo_rd_data");

        // -------------------------------------------------------------
        stage = "3-write-ctrl";
        // STAGE 3: sram_wr_ctrl maps the raw pixel index onto the memory.
        //   word_addr = 1026 >> 2      = 256
        //   byte_lane = 1026 & 3       = 2
        //   wr_be     = 1 << (3 - 2)   = 4'b0010   <-- REVERSED shift
        // Each channel byte is replicated across all four lanes; wr_be alone
        // decides which one lands.
        // -------------------------------------------------------------
        chk_h(n_sram_wr, 1, 32, "exactly one SRAM write");
        chk_h(cap_wr_addr, EXP_WORD,    14, "sram_wr_addr");
        chk_h(cap_wr_be,   EXP_BE,       4, "sram_wr_be");
        chk_h(cap_wr_r,    EXP_WDATA_R, 32, "sram_wr_data_r");
        chk_h(cap_wr_g,    EXP_WDATA_G, 32, "sram_wr_data_g");
        chk_h(cap_wr_b,    EXP_WDATA_B, 32, "sram_wr_data_b");
        chk(!sram_wr_rejected, "command was not rejected");
        chk(sram_wr_seen,      "wr_seen asserted");

        // -------------------------------------------------------------
        stage = "4-sram";
        // STAGE 4: only byte 1 (bits [15:8]) of word 256 changes, in each
        // channel. R happens to be unchanged because the pixel was already
        // 0xFF there; G and B change visibly.
        // -------------------------------------------------------------
        $display("  [4] SRAM word %0d after the write:", EXP_WORD);
        chk_h(u_sram_red.mem[EXP_WORD],   32'hFFFFFFFF, 32, "red   mem[256] after");
        chk_h(u_sram_green.mem[EXP_WORD], 32'hFFFFA5FF, 32, "green mem[256] after");
        chk_h(u_sram_blue.mem[EXP_WORD],  32'hFFFF00FF, 32, "blue  mem[256] after");

        // The three neighbouring pixels in the same word must be untouched.
        chk(u_sram_green.mem[EXP_WORD][31:24] == 8'hFF, "pixel 1024 green untouched");
        chk(u_sram_green.mem[EXP_WORD][23:16] == 8'hFF, "pixel 1025 green untouched");
        chk(u_sram_green.mem[EXP_WORD][ 7: 0] == 8'hFF, "pixel 1027 green untouched");
        chk(u_sram_blue.mem [EXP_WORD][31:24] == 8'hFF, "pixel 1024 blue untouched");
        chk(u_sram_blue.mem [EXP_WORD][ 7: 0] == 8'hFF, "pixel 1027 blue untouched");

        // An adjacent word must be completely untouched.
        chk_h(u_sram_green.mem[EXP_WORD+1], 32'hFFFFFFFF, 32, "green mem[257] untouched");

        // -------------------------------------------------------------
        stage = "5-readback";
        // STAGE 5: rom_sequencer walks the SRAMs exactly as it does for a
        // real image transfer, reassembling {R,G,B} per pixel in pixels[]
        // and pushing them into the pixel FIFO. The interlock releases the
        // write port to the reader for the duration of the walk.
        // -------------------------------------------------------------
        $display("  [5] starting readback walk (%0d words) ...", ROM_WORDS);
        chk(sram_wr_allowed, "wr_allowed high before the read starts");

        @(negedge clk_mem); start_req = 1'b1;
        @(negedge clk_mem); start_req = 1'b0;

        wait (rom_seq_busy);
        chk(!sram_wr_allowed, "wr_allowed low while rom_sequencer reads");

        wait (seq_done);
        repeat (200) @(negedge clk_rx);

        @(negedge clk_mem); img_done = 1'b1;
        @(negedge clk_mem); img_done = 1'b0;
        repeat (20) @(negedge clk_mem);
        chk(sram_wr_allowed, "wr_allowed returns high after the walk");

        // -------------------------------------------------------------
        stage = "6-pixel-out";
        // STAGE 6: the pixel emerges from the readback path in the transmit
        // domain. This is the same 24-bit value the UART would send.
        // -------------------------------------------------------------
        $display("  [6] pixels popped from the readback FIFO:");
        chk(pop_index >= 1028, "readback reached pixel 1027");
        chk_h(pop_1024, 24'hFFFFFF, 24, "pixel 1024 (neighbour)");
        chk_h(pop_1025, 24'hFFFFFF, 24, "pixel 1025 (neighbour)");
        chk_h(pop_1026, PIX_COLOUR, 24, "pixel 1026 -- THE WRITTEN PIXEL");
        chk_h(pop_1027, 24'hFFFFFF, 24, "pixel 1027 (neighbour)");

        // -------------------------------------------------------------
        $display("-------------------------------------------------");
        $display(" checks executed : %0d", checks);
        $display(" errors          : %0d", errors);
        $display(" RESULT: %s", (errors == 0) ? "PASS" : "FAIL");
        $display("=================================================");
        if (errors != 0) $fatal(1, "tb_stage2c_pipeline FAILED");
        $finish;
    end

    // Safety net -- a hang must not look like a pass.
    initial begin
        #2ms;
        $display(" RESULT: FAIL -- testbench timeout");
        $fatal(1, "tb_stage2c_pipeline timed out");
    end

endmodule : tb_stage2c_pipeline
