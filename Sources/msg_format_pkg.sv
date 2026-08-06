// msg_format_pkg.sv
// -----------------
// SINGLE SOURCE OF PROTOCOL TRUTH.
//
// Everything about the shape of a message on the wire is defined here and
// nowhere else: the ASCII alphabet, the field positions, the frame lengths,
// and the message-kind enumeration. RX, TX and the memory side all read from
// this one file.
//
// Deliberately holds NO implementation detail. FSM state encodings stay with
// the modules that own them (rx_phy_pkg, rx_mac_pkg, burst_state_t), and image
// geometry stays in memory_pkg. Putting either here would defeat the point.
//
// =======================================================================
// THE STRIDE MODEL -- ONE RULE COVERS THE ENTIRE MESSAGE SET
// =======================================================================
// Every message in the Final Project spec (section 11) is built from a fixed
// repeating structure. There is no per-message byte table anywhere:
//
//   byte 0                      '{'
//   group g  (g = 0, 1, 2)      bytes [1 + 5g .. 4 + 5g]
//   delimiter after group g     byte  5 + 5g     ',' if another group
//                                                '}' if this was the last
//
// A group is FOUR bytes: an opcode followed by three payload bytes.
//
//   byte 1 + 5g    opcode        'W' 'R' 'V' 'C' 'P' 'I' 'H'
//   byte 2 + 5g    payload MSB   big-endian, three bytes
//   byte 3 + 5g    payload
//   byte 4 + 5g    payload LSB
//
// Frame length follows directly from the group count:
//
//   1 group   ->  6 bytes   {R<A2,A1,A0>}
//   2 groups  -> 11 bytes   {W<A2,A1,A0>, P<R,G,B>}
//   3 groups  -> 16 bytes   {W<A2,A1,A0>, V<..>, V<..>}
//
//       LEN = 1 + 5 * NGROUPS
//
// Checked against all eight messages the design handles. There are no
// exceptions to the stride.
//
// -----------------------------------------------------------------------
// THE ONE VARIATION: BURST WRITE DATA
// -----------------------------------------------------------------------
// Burst data frames follow the SAME stride -- punctuation at 0, 5, 10, 15,
// three groups of four bytes -- but carry NO opcode. The first byte of each
// group is raw pixel data:
//
//   {<R0,G0,B0,R1>, <G1,B1,R2,G2>, <B2,R3,G3,B3>}
//
// Strip bytes 0, 5, 10 and 15 and the remaining twelve are four consecutive
// RGB triplets in order. The commas land mid-pixel, which is why the spec
// describes it as three 32-bit groups rather than four pixels.
//
// This is what the spec's "enable opcode bypass for the duration of HxW
// pixels" means: for the length of a burst, opcode inspection is suppressed
// entirely and the frame is a fixed 16 bytes. The structure is unchanged --
// only the interpretation of byte 1 + 5g changes.
//
// -----------------------------------------------------------------------
// WHY PAYLOAD BYTES CANNOT BE CONFUSED WITH DELIMITERS
// -----------------------------------------------------------------------
// Payload is RAW BINARY. A pixel value of 123 is 0x7B, which is '{'. A
// dimension byte of 125 is 0x7D, which is '}'. Delimiter-hunting anywhere in
// the frame would therefore corrupt legitimate messages.
//
// The stride is what makes framing safe: delimiters only ever occur at byte
// indices 0, 5, 10 and 15, and payload only ever occurs at indices that are
// NOT those. A '}' at byte 5, 10 or 15 is structurally a delimiter and can
// never be payload. Any framing decision must be POSITIONAL, never a search.

`timescale 1ns/1ps

package msg_format_pkg;

    // -----------------------------------------------------------------
    // ASCII alphabet
    //
    // Frame punctuation plus the seven opcodes. These are the only byte
    // values in the protocol with fixed meaning.
    // -----------------------------------------------------------------
    localparam logic [7:0] CHAR_OPEN_BRACE  = 8'h7B;  // '{'  frame start
    localparam logic [7:0] CHAR_CLOSE_BRACE = 8'h7D;  // '}'  frame end
    localparam logic [7:0] CHAR_COMMA       = 8'h2C;  // ','  group separator

    localparam logic [7:0] CHAR_W           = 8'h57;  // 'W'  write / width
    localparam logic [7:0] CHAR_R           = 8'h52;  // 'R'  read  / row
    localparam logic [7:0] CHAR_V           = 8'h56;  // 'V'  value
    localparam logic [7:0] CHAR_C           = 8'h43;  // 'C'  column
    localparam logic [7:0] CHAR_P           = 8'h50;  // 'P'  pixel
    localparam logic [7:0] CHAR_I           = 8'h49;  // 'I'  image burst write
    localparam logic [7:0] CHAR_H           = 8'h48;  // 'H'  height

    // Retained for the legacy {Rnnn,Cnnn,Vnnn} path, whose payload is three
    // ASCII decimal digits rather than three raw bytes.
    localparam logic [7:0] ASCII_ZERO       = 8'h30;  // '0'

    // -----------------------------------------------------------------
    // Stride model
    // -----------------------------------------------------------------
    localparam int GROUP_STRIDE   = 5;   // bytes from one opcode to the next
    localparam int GROUP_BYTES    = 4;   // opcode + three payload bytes
    localparam int PAYLOAD_BYTES  = 3;   // payload bytes per group
    localparam int PAYLOAD_W      = 8 * PAYLOAD_BYTES;   // 24
    localparam int MAX_GROUPS     = 3;   // longest message carries three

    // -----------------------------------------------------------------
    // Frame lengths, in bytes. Derived from the stride -- not a table.
    // -----------------------------------------------------------------
    localparam int MSG_BYTES_1GRP = 1 + GROUP_STRIDE * 1;   //  6
    localparam int MSG_BYTES_2GRP = 1 + GROUP_STRIDE * 2;   // 11
    localparam int MSG_BYTES_3GRP = 1 + GROUP_STRIDE * 3;   // 16
    localparam int MSG_BYTES_MAX  = MSG_BYTES_3GRP;         // 16

    // Width of the byte counter. 5 bits holds 0..16 inclusive.
    localparam int BYTE_CNT_W     = 5;

    // Frame buffer width, in bits.
    localparam int FRAME_W        = 8 * MSG_BYTES_MAX;      // 128

    // -----------------------------------------------------------------
    // Byte-index helpers.
    //
    // Every positional decision in the design goes through these, so the
    // stride is referenced everywhere and defined once. Bytes are numbered
    // from the START of the frame; see BIT INDEXING below for how that maps
    // onto the frame buffer.
    // -----------------------------------------------------------------

    // Byte index of the opcode of group g. For a burst data frame this is a
    // raw payload byte instead, but the POSITION is the same.
    function automatic int op_byte_idx(input int g);
        return 1 + GROUP_STRIDE * g;
    endfunction

    // Byte index of the first (most significant) payload byte of group g.
    function automatic int payload_byte_idx(input int g);
        return 2 + GROUP_STRIDE * g;
    endfunction

    // Byte index of the delimiter that closes group g -- ',' if another
    // group follows, '}' if this is the last.
    function automatic int delim_byte_idx(input int g);
        return GROUP_STRIDE + GROUP_STRIDE * g;
    endfunction

    // Frame length for a message carrying n groups.
    function automatic int len_for_groups(input int n);
        return 1 + GROUP_STRIDE * n;
    endfunction

    // -----------------------------------------------------------------
    // BIT INDEXING -- byte 0 is the MOST significant byte
    //
    // The frame buffer is filled MSB-first, so byte n occupies
    //
    //     frame[FRAME_W-1 - 8*n -: 8]
    //
    // Byte 0 is frame[127:120]. This matches rx_mac's existing lane
    // ordering exactly and must not be changed independently of it.
    //
    // NOTE: the TX path assembles frames LSB-first (tx_mac packs byte 0 in
    // msg_data[7:0]) -- the OPPOSITE convention. That asymmetry is
    // pre-existing and deliberate; msg_composer already builds to the TX
    // convention. These helpers describe the RECEIVE buffer only.
    // -----------------------------------------------------------------
    function automatic int byte_lsb(input int n);
        return FRAME_W - 8 * (n + 1);
    endfunction

    // -----------------------------------------------------------------
    // Message classification.
    //
    // One member per message the design accepts, plus MSG_UNKNOWN for
    // anything that does not classify.
    // -----------------------------------------------------------------
    typedef enum logic [3:0] {
        MSG_UNKNOWN    = 4'd0,  // unrecognised / malformed
        MSG_LEGACY_RGF = 4'd1,  // {Rnnn,Cnnn,Vnnn}          16, ASCII payload
        MSG_REG_WRITE  = 4'd2,  // {W<A>, V<..>, V<..>}      16
        MSG_REG_READ   = 4'd3,  // {R<A>}                     6
        MSG_PIX_WRITE  = 4'd4,  // {W<A>, P<R,G,B>}          11
        MSG_PIX_READ   = 4'd5,  // {R<..>, C<..>, P<..>}     16
        MSG_BURST_HDR  = 4'd6,  // {I<..>, H<..>, W<..>}     16
        MSG_BURST_READ = 4'd7,  // {R<A>, H<..>, W<..>}      16
        MSG_BURST_DATA = 4'd8   // {<4 px>,<4 px>,<4 px>}    16, no opcodes
    } msg_kind_t;

    // Group count for each kind. This is the ONLY place message length is
    // associated with message type; everything else derives length from the
    // stride. MSG_UNKNOWN defaults to the longest frame, which is the safe
    // direction: guessing too short truncates a valid message, guessing too
    // long merely keeps collecting until a real rule fires.
    function automatic int groups_for_kind(input msg_kind_t k);
        case (k)
            MSG_REG_READ  : return 1;   //  6 bytes
            MSG_PIX_WRITE : return 2;   // 11 bytes
            default       : return 3;   // 16 bytes
        endcase
    endfunction

    function automatic int len_for_kind(input msg_kind_t k);
        return len_for_groups(groups_for_kind(k));
    endfunction

    // =================================================================
    // CROSSING PAYLOAD LAYOUT
    //
    // Every message crosses to the memory domain on ONE shared interface:
    //
    //     { msg_kind[3:0], payload[95:0] }
    //
    // The payload is sized for the widest message -- burst data, four pixels
    // at 24 bits -- and every other kind occupies the low bits with the rest
    // zero. Narrow messages waste wire, not flops: the registers are shared,
    // so a register read costs nothing extra over a burst data frame.
    //
    // WHY ONE CROSSING RATHER THAN ONE PER MESSAGE TYPE
    //   AREA      each separate crossing carried its own data registers on
    //             both sides plus its own synchroniser chain. One shared
    //             crossing reuses the same flops for every kind.
    //   ORDERING  separate paths have no defined order between them. A burst
    //             header and its data could cross out of order, and only
    //             similar latencies made it work. One crossing makes order
    //             structural.
    //   COST      messages serialise instead of crossing in parallel. At
    //             ~2816 clocks between messages against a ~5 clock handshake
    //             the crossing is under 0.2% utilised.
    //
    // STRUCTS, NOT BIT RANGES.
    // These are packed structs so both sides name fields instead of slicing
    // by hand. Hand-written ranges are how a single pixel write ends up
    // reading payload[95:72] -- zeros -- and writing a black pixel with a
    // perfectly correct address and byte enable. Every signal you would
    // normally check looks right. First field declared is the MOST
    // significant.
    // =================================================================

    // Register address width. Kept as a local constant rather than an
    // import so this package stays dependency-free; it must match
    // rgf_pkg::ADDR_WIDTH, and rx_classifier asserts that it does.
    localparam int ADDR_W_RGF = 8;

    localparam int MSG_PAYLOAD_W = 96;

    typedef logic [MSG_PAYLOAD_W-1:0] msg_payload_t;

    // {W<A>, P<R,G,B>}
    typedef struct packed {
        logic [23:0] addr;
        logic [23:0] pixel;
    } pl_pix_write_t;                                    // 48

    // {I<..>, H<..>, W<..>}  -- opens a rectangle
    typedef struct packed {
        logic [9:0]  width;
        logic [9:0]  height;
        logic [23:0] base_addr;
    } pl_burst_hdr_t;                                    // 44

    // Four pixels. px0 is the most significant, matching arrival order.
    typedef struct packed {
        logic [23:0] px0;
        logic [23:0] px1;
        logic [23:0] px2;
        logic [23:0] px3;
    } pl_burst_data_t;                                   // 96, the widest

    // {W<A>, V<..>, V<..>}
    typedef struct packed {
        logic [ADDR_W_RGF-1:0] addr;
        logic [31:0]           data;
    } pl_reg_write_t;                                    // 40

    // {R<A>}
    typedef struct packed {
        logic [ADDR_W_RGF-1:0] addr;
    } pl_reg_read_t;                                     // 8

    // {R<row>, C<col>, P<..>}
    typedef struct packed {
        logic [9:0] row;
        logic [9:0] col;
    } pl_pix_read_t;                                     // 20

    // {R<A>, H<..>, W<..>}
    typedef struct packed {
        logic [9:0]  width;
        logic [9:0]  height;
        logic [23:0] base_addr;
    } pl_burst_read_t;                                   // 44

    // {Rnnn, Cnnn, Vnnn} -- decoded from ASCII by the classifier
    typedef struct packed {
        logic [9:0]  row;
        logic [9:0]  col;
        logic [23:0] pixel;
    } pl_legacy_t;                                       // 44

    // Zero-extend any of the above into the shared payload.
    function automatic msg_payload_t pl_pack(input logic [MSG_PAYLOAD_W-1:0] v,
                                             input int bits);
        msg_payload_t r;
        r = '0;
        for (int i = 0; i < MSG_PAYLOAD_W; i++)
            if (i < bits) r[i] = v[i];
        return r;
    endfunction

    // -----------------------------------------------------------------
    // Burst constants (absorbed from rx_burst_pkg).
    //
    // These are message-format properties, not RX properties -- burst_rd_ctrl
    // and memory_subsystem on the 100 MHz side use them too, which is why
    // they belong in the protocol package rather than an RX-only one.
    // -----------------------------------------------------------------
    localparam int BURST_DIM_W       = PAYLOAD_W;  // H and W, three bytes each
    localparam int BURST_PIX_W       = 24;         // {R,G,B}
    localparam int BURST_ADDR_W      = PAYLOAD_W;  // linear pixel index

    // Pixels carried by one Burst Data message: twelve payload bytes / three
    // bytes per pixel.
    localparam int BURST_PIX_PER_MSG = (MAX_GROUPS * GROUP_BYTES) / 3;   // 4
    localparam int BURST_SLOT_W      = $clog2(BURST_PIX_PER_MSG);        // 2

endpackage : msg_format_pkg
