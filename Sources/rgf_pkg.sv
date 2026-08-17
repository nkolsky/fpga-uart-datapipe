// -----------------------------------------------------------------------------
// rgf_pkg.sv
//
// Register map for the config register file. Addresses are byte-indexed with a
// 32-bit stride, and the IDLE_ADDR value is used when no command is active so
// level-sensitive decoders do not accidentally act on stale data.
//
// The register set is: IMG_STATUS, IMG_TX_MON, IMG_CTRL, FIFO_STATUS,
// CLK_CTRL, and PARITY_FAULT_CNT. IMG_CTRL.start_img_read is gated by the
// complete/error interlock; IMG_TX_MON complete/error bits are read-to-clear.
// -----------------------------------------------------------------------------

`timescale 1ns/1ps

package rgf_pkg;

    // -----------------------------------------------------------------
    // Register addresses
    // -----------------------------------------------------------------
    localparam logic [7:0] IMG_STATUS_ADDR      = 8'h00;
    localparam logic [7:0] IMG_TX_MON_ADDR      = 8'h04;
    localparam logic [7:0] IMG_CTRL_ADDR        = 8'h08;
    localparam logic [7:0] FIFO_STATUS_ADDR     = 8'h0C;
    localparam logic [7:0] CLK_CTRL_ADDR        = 8'h10;
    localparam logic [7:0] PARITY_FAULT_CNT_ADDR = 8'h14;

    // -----------------------------------------------------------------
    // IDLE_ADDR -- the address presented when NO command is being issued.
    //
    // This is a REQUIREMENT of this register file, not a convenience of
    // whoever drives it, which is why it lives here beside the map.
    //
    // IMG_TX_MON's read-to-clear is a LEVEL-SENSITIVE decode:
    //
    //     else if (!pc_wen && pc_sel_img_tx_mon)   // rgf.sv
    //
    // There is no valid qualifier on it. So a driver that HOLDS its last
    // address between commands clears img_send_complete/img_send_error on
    // every idle cycle once that address has been 0x04, and the IMG_CTRL
    // start interlock silently stops working -- no error, no assertion,
    // just a flag that can never latch.
    //
    // The requirement is therefore: park the address here whenever the
    // command is not valid. It must decode to nothing, i.e. match none of
    // 0x00/0x04/0x08/0x0C/0x10/0x14.
    //
    // HISTORY. This used to be enforced structurally by cdc_cmd_sync's
    // IDLE_ADDR parameter, which defaulted to '1 and drove dst_addr to it
    // whenever dst_valid was low. That crossing was removed when register
    // commands became messages routed by mem_msg_router, and the parking
    // went with it -- mem_msg_router assigned rgf_cmd_addr only inside
    // `if (fire && to_rgf)`, so the address held indefinitely. The
    // obligation now belongs to the router, and naming the constant here
    // keeps it defined once rather than as a literal at the driver.
    // -----------------------------------------------------------------
    localparam logic [7:0] IDLE_ADDR = 8'hFF;

    // -----------------------------------------------------------------
    // IMG_STATUS bitfields - RO (continuous passthrough from static inputs)
    //   [9:0]   img_height
    //   [19:10] img_width
    //   [20]    img_ready
    // -----------------------------------------------------------------
    localparam int IMG_STATUS_height_START = 0;
    localparam int IMG_STATUS_height_STOP  = 9;
    localparam int IMG_STATUS_width_START  = 10;
    localparam int IMG_STATUS_width_STOP   = 19;
    localparam int IMG_STATUS_ready_START  = 20;
    localparam int IMG_STATUS_ready_STOP   = 20;

    typedef struct packed {
        logic [10:0] pad;        // [31:21] padding to 32 bits, unused
        logic        img_ready;
        logic [9:0]  img_width;
        logic [9:0]  img_height;
    } img_status_t;

    // -----------------------------------------------------------------
    // IMG_TX_MON bitfields - IW / IW+RC (see package header note above)
    //   [9:0]   row_cnt              (IW    - plain progress counter)
    //   [19:10] col_cnt              (IW    - plain progress counter)
    //   [20]    img_send_complete    (IW+RC - cleared on PC read)
    //   [21]    img_send_error       (IW+RC - cleared on PC read)
    // -----------------------------------------------------------------
    localparam int IMG_TX_MON_row_START      = 0;
    localparam int IMG_TX_MON_row_STOP       = 9;
    localparam int IMG_TX_MON_col_START      = 10;
    localparam int IMG_TX_MON_col_STOP       = 19;
    localparam int IMG_TX_MON_complete_START = 20;
    localparam int IMG_TX_MON_complete_STOP  = 20;
    localparam int IMG_TX_MON_error_START    = 21;
    localparam int IMG_TX_MON_error_STOP     = 21;

    typedef struct packed {
        logic [9:0] pad;         // [31:22] padding to 32 bits, unused
        logic       img_send_error;
        logic       img_send_complete;
        logic [9:0] col_cnt;
        logic [9:0] row_cnt;
    } img_tx_mon_t;

    // -----------------------------------------------------------------
    // IMG_CTRL bitfields - RW
    //   [0] start_img_read - see interlock note in package header above
    // -----------------------------------------------------------------
    localparam int IMG_CTRL_start_START = 0;
    localparam int IMG_CTRL_start_STOP  = 0;

    typedef struct packed {
        logic [30:0] pad;        // [31:1] padding to 32 bits, unused
        logic        start_img_read;
    } img_ctrl_t;

    // -----------------------------------------------------------------
    // FIFO_STATUS bitfields - RO (Lab 10: continuous passthrough from
    // async_fifo, dedicated ports -- not the status_* bus. These are
    // live hardware levels, always current, not one-shot events that
    // need capturing at a specific moment (unlike IMG_TX_MON) -- so
    // there's no snapshot-write mechanism needed here, same reasoning
    // as IMG_STATUS's continuous passthrough.
    //   [0] full
    //   [1] empty
    //   [2] almost_full
    //   [3] almost_empty
    // No occupancy count -- async_fifo doesn't currently expose one (a
    // true count would need CDC-aware pointer-difference logic inside
    // async_fifo.sv itself, out of scope for this pass). Flags only,
    // matching the simpler of two reasonable designs considered.
    // -----------------------------------------------------------------
    localparam int FIFO_STATUS_full_START         = 0;
    localparam int FIFO_STATUS_full_STOP          = 0;
    localparam int FIFO_STATUS_empty_START        = 1;
    localparam int FIFO_STATUS_empty_STOP         = 1;
    localparam int FIFO_STATUS_almost_full_START  = 2;
    localparam int FIFO_STATUS_almost_full_STOP   = 2;
    localparam int FIFO_STATUS_almost_empty_START = 3;
    localparam int FIFO_STATUS_almost_empty_STOP  = 3;

    typedef struct packed {
        logic [27:0] pad;        // [31:4] padding to 32 bits, unused
        logic        almost_empty;
        logic        almost_full;
        logic        empty;
        logic        full;
    } fifo_status_t;

    // -----------------------------------------------------------------
    // Data width for the RGF mem-style interface
    // -----------------------------------------------------------------
    localparam int DATA_WIDTH = 32;
    localparam int ADDR_WIDTH = 8;

    // -----------------------------------------------------------------
    // CLK_CTRL bitfields - RW (Lab 10: clock mux select)
    //   [0] clk_sel - 0=clk0/CLK100MHZ (fallback), 1=clk1/PLL clock
    // Selects the clock for the "counter system that can operate at two
    // different clock speeds" per the spec -- this is the literal
    // control register the spec asks for.
    // No interlock: unlike IMG_CTRL.start, there's no "must be idle"
    // precondition for switching clocks -- glitchless_clk_mux's own
    // break-before-make logic handles a switch arriving at any time.
    // -----------------------------------------------------------------
    localparam int CLK_CTRL_sel_START = 0;
    localparam int CLK_CTRL_sel_STOP  = 0;

    typedef struct packed {
        logic [30:0] pad;        // [31:1] padding to 32 bits, unused
        logic        clk_sel;
    } clk_ctrl_t;

    // -----------------------------------------------------------------
    // PARITY_FAULT_CNT (Lab 10) - RO
    // Plain 32-bit monotonic counter, no bitfields -- the whole register
    // IS the count. Incremented by rx_phy.parity_err_pulse via a
    // dedicated single-bit input (rgf.parity_fault_incr), NOT via the
    // shared status_* bus -- that bus is built for "write this full
    // value" semantics (see IMG_TX_MON/FIFO_STATUS), not "increment by
    // 1," so a dedicated pulse input is the simpler fit here.
    // Not read-to-clear or write-to-clear in this pass -- just counts
    // monotonically from reset. A clear mechanism is a reasonable future
    // addition if you want the PC to be able to reset the count without
    // a full chip reset, deliberately not built now to keep this pass
    // minimal.
    // -----------------------------------------------------------------

endpackage : rgf_pkg