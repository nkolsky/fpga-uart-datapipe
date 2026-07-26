// rgf_pkg.sv
// ----------
// Register map and bitfield definitions for the Config RGF.
// BARE MINIMUM pass: exactly the four registers Lab 8/9 specify, nothing
// added yet. Robustness (invalid-address latching, reserved-bit masking,
// etc.) is a deliberate follow-up pass once this is verified working.
//
// Register access-type glossary:
//   RW    - PC reads and writes directly (mem-style pc_wen/pc_addr/pc_wdata)
//   RO    - PC reads only; continuous passthrough from external/static inputs
//   IW    - Internally-Written: PC reads only; written by internal chip
//           logic (the Sequencer, via status_wen/status_addr/status_wdata)
//           and holds its value between writes
//   IW+RC - Internally-Written AND Read-to-Clear: same as IW, but a PC
//           READ additionally clears specific bits back to 0. Used for
//           IMG_TX_MON.img_send_complete/img_send_error only (see rgf.sv
//           section on the IMG_CTRL interlock for why).
//
// Four registers:
//   IMG_STATUS  (RO)    - static image dimensions + ready flag
//   IMG_TX_MON  (IW/+RC)- live progress of the TX drain; row_cnt/col_cnt
//                          are plain IW, img_send_complete/img_send_error
//                          are IW+RC (see rgf.sv)
//   IMG_CTRL    (RW)    - PC writes bit 0 to trigger an image send
//   FIFO_STATUS (IW)    - async_fifo flag passthrough, for PC-side debug
//                          visibility
//
// -----------------------------------------------------------------------
// IMG_CTRL interlock -- resolved per the LITERAL spec wording this pass:
//   "Initiates the image read process only if image transfer complete
//    and image transfer error are both cleared"
// i.e. accept a '1' write to start only when img_send_complete==0 AND
// img_send_error==0. This is also the natural reset state (both default
// to 0), so no reset-value override is needed to allow the first start --
// unlike an earlier draft that required complete==1, which needed a
// special reset hack to work at all.
//
// This raises the obvious follow-up question: once a transfer finishes
// and img_send_complete gets set to 1, what clears it back to 0 so a
// second transfer can start? Answer: IMG_TX_MON is read-to-clear on
// those two bits specifically. The PC reads IMG_TX_MON once to observe
// the finished/error state; that read clears both bits, which re-arms
// IMG_CTRL.start for the next transfer. row_cnt/col_cnt are NOT cleared
// by a read -- they're live progress counters, not one-shot event flags.
// -----------------------------------------------------------------------

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