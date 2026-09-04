#!/usr/bin/env python3
"""
rgf_parking_test.py -- prove the RGF address parking fix on hardware.

WHAT IS BEING TESTED
--------------------
rgf.sv implements IMG_TX_MON's read-to-clear as a LEVEL-SENSITIVE decode
with no valid qualifier:

    else if (!pc_wen && pc_sel_img_tx_mon)      // rgf.sv

so whatever drives pc_addr must PARK it at an address that decodes to
nothing whenever no command is being issued. That used to be done by
cdc_cmd_sync's IDLE_ADDR parameter. When register commands became messages
the crossing was deleted, and mem_msg_router assigned rgf_cmd_addr only
inside `if (fire && to_rgf)` -- so the address held indefinitely, and
img_send_complete/img_send_error were cleared on every idle cycle from the
first IMG_TX_MON access onward.

WHY THE POLL MUST IGNORE CTS -- THE MISTAKE THIS SCRIPT USED TO MAKE
-------------------------------------------------------------------
The address has to still be 0x04 at the moment tx_img_done fires. Starting
a transfer needs a WRITE to IMG_CTRL (0x08), which moves the address off
0x04, so the read has to land DURING the transfer.

An earlier version of this script polled over a flow-controlled link and
claimed the reads would slip through the gaps between pixel messages. They
do not. tx_sequencer returns to IDLE for exactly ONE cycle per message,
and a 16-byte message at 8.125 Mbaud is 2816 clocks (~21.7 us), so
UART_CTS releases for ~7.7 ns per 21.7 us -- a 0.035% duty cycle the FTDI
cannot use. The polls stayed in the driver buffer, flushed after the
transfer ended, and cleared img_send_complete legitimately. The test then
failed on a CORRECT build, reporting read-to-clear working as a bug.

So the poll here goes out on a link opened with rtscts=False. That is the
only way to deliver a register read into a running transfer, and it is
also the honest statement of the bug's reach: a host that honours RTS/CTS
cannot trigger it. flowtest.py's tests 2, 3, 4 and 6 all run with
rtscts=False, so it is reachable in practice.

ONE POLL, NOT SIX. tx_reply_ctrl holds a single reply slot and drops a
register reply arriving while it is occupied (setting reply_overrun). Six
polls mid-transfer would mean one reply and five drops, which muddies the
byte accounting for no gain. One read is all the address parking needs.

    broken build : complete reads 0 after the polled transfer
    fixed build  : complete reads 1

Both directions are checked -- [3] that the flag survives a polled
transfer, [5] that the interlock still blocks -- because "complete is set"
alone could be luck and "start refused" alone could be an unrelated stall.

NOTE ON THE REPLY STREAM
------------------------
The poll's reply is deferred by tx_reply_ctrl until the transfer ends, so
a polled transfer returns 6 bytes more than an unpolled one. This script
COUNTS bytes and does not decode them; use image_tool.py to judge image
integrity.

USAGE
    python rgf_parking_test.py --port COM5
    python rgf_parking_test.py --port COM5 --width 256 --height 256

Needs board_test.py and flowtest.py (and image_tool.py, which flowtest
imports) beside it. The protocol builders and the non-flow-controlled
Link are imported rather than copied.
"""

import argparse
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("pyserial is required:  pip install pyserial")

try:
    import board_test as BT
except ImportError:
    sys.exit("board_test.py must be in the same directory as this script")

try:
    import flowtest as FT
except ImportError:
    sys.exit("flowtest.py must be in the same directory as this script")

for _m, _a in ((BT, "msg_reg_read"), (BT, "msg_reg_write"),
               (BT, "REG_REPLY_LEN"), (FT, "Link")):
    if not hasattr(_m, _a):
        sys.exit("%s is the wrong version: no %s" % (_m.__name__, _a))

IMG_STATUS = 0x00
IMG_TX_MON = 0x04
IMG_CTRL   = 0x08

COMPLETE_BIT = 20
ERROR_BIT    = 21


class Results:
    def __init__(self):
        self.passed = 0
        self.failed = 0

    def check(self, name, ok, detail=""):
        if ok:
            self.passed += 1
            print("  PASS  %-46s %s" % (name, detail))
        else:
            self.failed += 1
            print("  FAIL  %-46s %s" % (name, detail))
        return ok


def bit(v, n):
    return (v >> n) & 1


def reg_read(link, addr):
    """One register read. Returns the 32-bit value or None."""
    link.discard_input()
    link.send(BT.msg_reg_read(addr))
    link.flush()
    raw = link.read_exact(BT.REG_REPLY_LEN, timeout=1.5)
    if raw is None or raw[0] != BT.OPEN or raw[5] != BT.CLOSE:
        return None
    return (raw[1] << 24) | (raw[2] << 16) | (raw[3] << 8) | raw[4]


def reg_write(link, addr, value):
    link.send(BT.msg_reg_write(addr, value))
    link.flush()
    time.sleep(0.02)


def transfer(link, expect, poll_at=None, settle=0.6, limit=20.0):
    """
    Start a full-image transfer and drain it, optionally issuing ONE
    IMG_TX_MON read once poll_at bytes have arrived.

    Draining continuously matters: the link is opened with rtscts=False,
    so nothing throttles the board and a stalled reader would overrun the
    driver buffer rather than back-pressuring.

    Returns (bytes_received, polled).
    """
    link.discard_input()
    reg_write(link, IMG_CTRL, 0x0000_0001)

    buf = bytearray()
    polled = False
    quiet = 0.0
    deadline = time.time() + limit

    while time.time() < deadline:
        n = link.drain_for(0.02, buf)
        if n:
            quiet = 0.0
        else:
            quiet += 0.02
            if len(buf) > 0 and quiet >= settle:
                break

        if poll_at is not None and not polled and len(buf) >= poll_at:
            # THE POLL. Goes out regardless of the board's CTS, which is
            # the only way it lands while the transfer is still running.
            link.send(BT.msg_reg_read(IMG_TX_MON))
            link.flush()
            polled = True

    reg_write(link, IMG_CTRL, 0x0000_0000)
    link.drain_for(0.3, buf)
    return len(buf), polled


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--baud", type=int, default=8125000)
    ap.add_argument("--timeout", type=float, default=0.5)
    ap.add_argument("--width", type=int, default=256)
    ap.add_argument("--height", type=int, default=256)
    a = ap.parse_args()

    # The FULL-IMAGE stream path is rom_sequencer -> image FIFO ->
    # tx_sequencer -> msg_composer, which emits ONE PIXEL PER 16-BYTE
    # MESSAGE ({R<row>,C<col>,P<R,G,B>}) -- the spec's "single pixel per
    # message read from FPGA". It is NOT the four-pixels-per-frame burst
    # reply format; that belongs to burst_rd_ctrl / burst_msg_composer.
    expect = a.width * a.height * 16
    poll_at = expect // 3

    print("port %s  baud %d  image %dx%d" % (a.port, a.baud, a.width, a.height))
    print("a full transfer is %d bytes (one 16-byte message per pixel)" % expect)
    print("RTS/CTS is OFF for our writes: a flow-controlled host cannot")
    print("deliver a register read into a running transfer at all.")

    link = FT.Link(a.port, a.baud, a.timeout, rtscts=False, write_timeout=5.0)
    link.allow_send(True)          # board may transmit
    res = Results()
    polled_latched = None
    unpolled_latched = None

    try:
        # -----------------------------------------------------------
        print("\n[0] register path is alive")
        v = reg_read(link, IMG_STATUS)
        if not res.check("IMG_STATUS readable", v is not None,
                         "no reply" if v is None else "%08x" % v):
            print("      Registers are not answering. Run board_test.py --reg-only.")
            return 1
        h, w = v & 0x3FF, (v >> 10) & 0x3FF
        res.check("IMG_STATUS geometry matches --width/--height",
                  (h, w) == (a.height, a.width),
                  "board says %dx%d, you said %dx%d" % (w, h, a.width, a.height))

        # -----------------------------------------------------------
        print("\n[1] clear IMG_TX_MON, then confirm it reads back clear")
        reg_read(link, IMG_TX_MON)                 # read-to-clear
        v = reg_read(link, IMG_TX_MON)
        if v is None:
            res.check("IMG_TX_MON readable", False, "no reply")
            return 1
        res.check("complete/error clear before the transfer",
                  bit(v, COMPLETE_BIT) == 0 and bit(v, ERROR_BIT) == 0,
                  "IMG_TX_MON=%08x" % v)

        # -----------------------------------------------------------
        print("\n[2] transfer, with ONE IMG_TX_MON read delivered mid-stream")
        print("      that read is the test: it leaves cmd_addr at 0x04 across")
        print("      tx_img_done on a build without address parking")
        got, polled = transfer(link, expect, poll_at=poll_at)
        print("      %d bytes received, poll issued: %s" % (got, polled))
        res.check("poll was delivered during the transfer", polled, "")
        res.check("transfer produced a substantial stream", got > expect // 2,
                  "%d bytes, expected ~%d + 6 for the deferred reply"
                  % (got, expect))

        # -----------------------------------------------------------
        print("\n[3] THE CHECK -- did img_send_complete survive the poll?")
        v = reg_read(link, IMG_TX_MON)
        if v is None:
            res.check("IMG_TX_MON readable after the transfer", False, "no reply")
            return 1
        polled_latched = bit(v, COMPLETE_BIT) == 1
        if not res.check("img_send_complete latched", polled_latched,
                         "IMG_TX_MON=%08x" % v):
            print("      complete is 0 after a completed transfer. cmd_addr is")
            print("      being held at IMG_TX_MON_ADDR, so the read-to-clear")
            print("      decode fires every idle cycle and the flag can never")
            print("      latch. This is the unparked build.")
        res.check("img_send_error clear", bit(v, ERROR_BIT) == 0,
                  "IMG_TX_MON=%08x" % v)

        v2 = reg_read(link, IMG_TX_MON)
        res.check("read-to-clear cleared complete",
                  v2 is not None and bit(v2, COMPLETE_BIT) == 0,
                  "IMG_TX_MON=%s" % ("none" if v2 is None else "%08x" % v2))

        # -----------------------------------------------------------
        print("\n[4] interlock re-armed by that read -- a start must be accepted")
        got2, _ = transfer(link, expect, poll_at=None)
        res.check("second transfer ran after the clearing read",
                  got2 > expect // 2, "%d bytes" % got2)

        # -----------------------------------------------------------
        print("\n[5] interlock BLOCKS -- start again without clearing first")
        link.discard_input()
        reg_write(link, IMG_CTRL, 0x0000_0001)
        blocked = bytearray()
        link.drain_for(1.0, blocked)
        reg_write(link, IMG_CTRL, 0x0000_0000)
        link.drain_for(0.3, blocked)
        unpolled_latched = len(blocked) < 64
        res.check("start refused while complete is set", unpolled_latched,
                  "%d bytes arrived, expected ~0" % len(blocked))
        if not unpolled_latched:
            print("      The transfer started anyway, so complete was not set.")

        # -----------------------------------------------------------
        print("\n[6] cleanup: clear the flag and confirm the board is re-armed")
        reg_read(link, IMG_TX_MON)
        v = reg_read(link, IMG_TX_MON)
        res.check("board left in a clean, re-armed state",
                  v is not None and bit(v, COMPLETE_BIT) == 0,
                  "IMG_TX_MON=%s" % ("none" if v is None else "%08x" % v))

    finally:
        try:
            link.close()
        except Exception:
            pass

    print("\n" + "=" * 62)
    print("  bytes sent     : %d" % link.sent)
    print("  bytes received : %d" % link.recv)
    print("  passed         : %d" % res.passed)
    print("  failed         : %d" % res.failed)
    if link.recv == 0:
        print("\n  NOTHING WAS RECEIVED. Any pass above would be a bug here.")

    # The two transfers differ ONLY in whether IMG_TX_MON was read while
    # they ran, so comparing them separates "the address is being held"
    # from every other reason the flag might be wrong.
    if polled_latched is False and unpolled_latched is True:
        print("\n  DIAGNOSIS: THE ADDRESS IS NOT BEING PARKED.")
        print("  complete survived the UNPOLLED transfer [4]/[5] but not the")
        print("  POLLED one [2]/[3]. The mid-transfer read is the only")
        print("  difference, and it is what leaves cmd_addr at 0x04 across")
        print("  tx_img_done. Check the bitstream was rebuilt from a")
        print("  mem_msg_router.sv containing rgf_pkg::IDLE_ADDR.")
    elif polled_latched is False and unpolled_latched is False:
        print("\n  DIAGNOSIS: complete never latched at all, polled or not.")
        print("  That is NOT the parking bug. Look at the status_wen path:")
        print("  tx_img_done, its cdc_pulse_sync, and register_subsystem's")
        print("  IMG_TX_MON write decode.")
    elif polled_latched and unpolled_latched:
        print("\n  Address parking is working: complete latched across a")
        print("  transfer that was read mid-stream, and the interlock still")
        print("  refuses a start until IMG_TX_MON is read.")

    print("  RESULT         : %s"
          % ("PASS" if res.failed == 0 and link.recv > 0 else "FAIL"))
    print("=" * 62)
    return 0 if res.failed == 0 and link.recv > 0 else 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except serial.SerialException as e:
        sys.exit("\ncould not open the port: %s" % e)
    except KeyboardInterrupt:
        sys.exit("\ninterrupted")
