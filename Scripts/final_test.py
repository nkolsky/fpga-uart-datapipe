#!/usr/bin/env python3
"""
final_test.py -- one run that exercises every pipeline on the board.

    python final_test.py --port COM5
    python final_test.py --port COM5 --image photo.png
    python final_test.py --port COM5 --image photo.png --keep

WITHOUT THE BOARD THIS SCRIPT DOES NOTHING.
Stage 0 opens the port and requires a well-formed register reply before
anything else runs. No file is written, no byte is written to memory, and
no stage is reported until the board has answered. There is no simulation
mode, no offline mode and no default value anywhere that could stand in
for a reply that never came. If the port is absent or silent the script
prints why and exits 2.

WHAT IT PROVES, IN ORDER
------------------------
  0  the board is there and answering            register read
  1  BASELINE: the whole image, captured         Image Burst Read + address-echo sampling
  2  register pipeline                           reads of four registers
  3  single pixel write pipeline                 write, read back by address, neighbour untouched
  4  burst write pipeline, UNALIGNED             6 wide at column 61: two partial words per row
  5  burst read placement                        burst region vs per-pixel region, compared
  6  load a new image                            optional, --image
  7  flow control, both directions               hold-off, then a lossless/lossy contrast
  8  RESTORE the original image                  load the stage 1 capture back, verify every pixel

WHY THE BASELINE COMES FIRST
----------------------------
The board's initial image comes from the .mem files at CONFIGURATION time.
CPU_RESETN does not reload it, and this script cannot reprogram the FPGA.
So "put it back" can only mean "write back what was there", and that has
to be captured before anything is written.

Stage 1 therefore runs before any write, and if it comes back with missing
pixels or a failed address-echo check, the script STOPS. Writing to a board
whose original contents were not captured cleanly would be unrecoverable
without a reprogram, so a bad baseline is a hard stop rather than a warning.

Restore is on by default. --keep skips it, and says so.

WHAT IT IS NOT
--------------
This is a final acceptance run, not a diagnostic. Every stage checks something
the board must get right, and reports pass or fail. When something fails,
the dedicated scripts are what tell you why:

    board_test.py            protocol correctness, cannot-fake-success
    flowtest.py              RTS/CTS in depth, with a negative control
    rgf_parking_test.py      the RGF address parking regression
    image_tool.py            look at the image, load one, spot-check placement

Needs image_tool.py and flowtest.py beside it.
"""

import argparse
import os
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("pyserial is required:  pip install pyserial")

try:
    import image_tool as IT
except ImportError:
    sys.exit("image_tool.py must be in the same directory as this script")

try:
    import flowtest as FT
except ImportError:
    sys.exit("flowtest.py must be in the same directory as this script")

for _m, _a in ((IT, "Board"), (IT, "read_image_burst"), (IT, "read_image_pixels"),
               (IT, "write_image_burst"), (IT, "verify_pixels"), (IT, "write_png"),
               (IT, "read_png"), (IT, "msg_reg_write"), (IT, "msg_pixel_write"),
               (IT, "msg_pixel_read"), (IT, "parse_pixel_reply"), (FT, "Link")):
    if not hasattr(_m, _a):
        sys.exit("%s is the wrong version: no %s" % (_m.__name__, _a))

IMG_STATUS, IMG_TX_MON, IMG_CTRL, CLK_CTRL = 0x00, 0x04, 0x08, 0x10
REG_REPLY_LEN = 6
OPEN, CLOSE = ord('{'), ord('}')

# Stage 3 and 4 targets. Chosen to sit away from (0,0) and from each other,
# and stage 4 is deliberately unaligned: 6 wide starting at column 61 means
# every row spans two words and ends mid-word, which is the case that needs
# the byte enables and the end-of-row flush to both be right.
PIX_ROW, PIX_COL, PIX_VAL = 9, 5, (0xC0, 0xFF, 0xEE)
RECT_ROW, RECT_COL, RECT_H, RECT_W, RECT_VAL = 61, 61, 10, 6, (0x00, 0xFF, 0xFF)


class Report:
    def __init__(self):
        self.rows = []
        self.failed = 0

    def ok(self, stage, what, detail=""):
        self.rows.append((stage, "PASS", what, detail))
        print("   PASS  %-44s %s" % (what, detail))

    def bad(self, stage, what, detail=""):
        self.rows.append((stage, "FAIL", what, detail))
        self.failed += 1
        print("   FAIL  %-44s %s" % (what, detail))

    def check(self, stage, what, cond, detail=""):
        (self.ok if cond else self.bad)(stage, what, detail)
        return cond


def stage(n, title):
    print("\n" + "=" * 68)
    print("  STAGE %s  %s" % (n, title))
    print("=" * 68)


def reg_read(board, addr):
    """One register read. Returns the 32-bit value, or None."""
    # image_tool has no register READ builder (it only ever writes IMG_CTRL),
    # so the 6-byte frame is built here. Same layout as board_test's
    # msg_reg_read: { R <A2,A1,A0> }
    board.discard_input()
    board.send(bytes([OPEN, ord('R')]) + IT.be24(addr) + bytes([CLOSE]))
    board.flush()
    raw = board.read_exact(REG_REPLY_LEN)
    if raw is None or len(raw) != REG_REPLY_LEN or raw[0] != OPEN or raw[5] != CLOSE:
        return None
    return (raw[1] << 24) | (raw[2] << 16) | (raw[3] << 8) | raw[4]


def read_one_pixel(board, row, col):
    """Single Pixel Read. Returns ((r,g,b), echo_ok) or (None, False)."""
    board.discard_input()
    board.send(IT.msg_pixel_read(row, col))
    board.flush()
    raw = board.read_exact(16)
    if raw is None:
        return None, False
    got = IT.parse_pixel_reply(raw)
    if got is None:
        return None, False
    r, c, px = got
    return px, (r == row and c == col)


def write_one_pixel(board, row, col, width, rgb):
    board.send(IT.msg_pixel_write(row * width + col, *rgb))
    board.flush()
    time.sleep(0.02)


def same(a, b):
    return list(a) == list(b)


# ---------------------------------------------------------------- stage 0
def stage0(board, rep, args):
    stage(0, "is the board there?")
    print("   Nothing below runs, and no file is written, until this passes.")
    v = reg_read(board, IMG_STATUS)
    if v is None:
        rep.bad(0, "IMG_STATUS replied", "no reply -- board silent or not programmed")
        return None
    h, w, ready = v & 0x3FF, (v >> 10) & 0x3FF, (v >> 20) & 1
    rep.ok(0, "IMG_STATUS replied", "%08x" % v)
    if not rep.check(0, "geometry matches --width/--height",
                     (w, h) == (args.width, args.height),
                     "board says %dx%d, you said %dx%d" % (w, h, args.width, args.height)):
        return None
    rep.check(0, "img_ready set", ready == 1, "")
    return (w, h)


# ---------------------------------------------------------------- stage 1
def stage1(board, rep, args, outdir):
    stage(1, "baseline -- capture the original image BEFORE writing anything")
    print("   This capture is the only way back: CPU_RESETN does not reload")
    print("   BRAM, and this script cannot reprogram the FPGA.")
    t0 = time.time()
    pixels, missing = IT.read_image_burst(board, args.width, args.height,
                                          retries=args.retries)
    dt = time.time() - t0
    if not rep.check(1, "every pixel came back", missing == 0,
                     "%d missing of %d, %.1f s" % (missing, args.width * args.height, dt)):
        return None
    bad = IT.verify_pixels(board, pixels, args.width, args.height,
                           0, 0, args.width, args.height, args.verify)
    if not rep.check(1, "placement confirmed by address echo", bad == 0,
                     "%d of %d samples disagreed" % (bad, args.verify)):
        return None
    path = os.path.join(outdir, "00_original.png")
    IT.write_png(path, pixels, args.width, args.height)
    rep.ok(1, "baseline written", path)
    return pixels


# ---------------------------------------------------------------- stage 2
def stage2(board, rep):
    stage(2, "register pipeline")
    names = [(IMG_STATUS, "IMG_STATUS"), (IMG_TX_MON, "IMG_TX_MON"),
             (IMG_CTRL, "IMG_CTRL"), (CLK_CTRL, "CLK_CTRL")]
    for addr, nm in names:
        v = reg_read(board, addr)
        rep.check(2, "read %s (0x%02X)" % (nm, addr), v is not None,
                  "none" if v is None else "%08x" % v)


# ---------------------------------------------------------------- stage 3
def stage3(board, rep, args):
    stage(3, "single pixel write -- one byte lane, no read-modify-write")
    row, col = PIX_ROW, PIX_COL
    nb_col = col + 1                      # same 32-bit word, next lane
    before, echo0 = read_one_pixel(board, row, nb_col)
    if not rep.check(3, "neighbour readable before the write", before is not None and echo0, ""):
        return
    write_one_pixel(board, row, col, args.width, PIX_VAL)
    got, echo = read_one_pixel(board, row, col)
    rep.check(3, "pixel reads back as written",
              got is not None and same(got, PIX_VAL) and echo,
              "wrote %02X%02X%02X, got %s%s" % (
                  PIX_VAL[0], PIX_VAL[1], PIX_VAL[2],
                  "none" if got is None else "%02X%02X%02X" % tuple(got),
                  "" if echo else "  ADDRESS ECHO WRONG"))
    after, echo1 = read_one_pixel(board, row, nb_col)
    rep.check(3, "neighbour lane untouched",
              after is not None and echo1 and same(after, before),
              "was %02X%02X%02X, now %s" % (
                  before[0], before[1], before[2],
                  "none" if after is None else "%02X%02X%02X" % tuple(after)))


# ---------------------------------------------------------------- stage 4
def stage4(board, rep, args):
    stage(4, "burst write, UNALIGNED -- %d wide at column %d" % (RECT_W, RECT_COL))
    print("   Every row of this rectangle spans two words and ends mid-word,")
    print("   so it needs the byte enables and the end-of-row flush.")
    block = [RECT_VAL] * (RECT_W * RECT_H)
    IT.write_image_burst(board, block, args.width, RECT_COL, RECT_ROW, RECT_W, RECT_H)
    time.sleep(0.05)
    # Read back per pixel: every reply carries its own address, so a
    # placement error cannot hide behind the mechanism that wrote it.
    got, missing = IT.read_image_pixels(board, args.width, args.height,
                                        RECT_COL, RECT_ROW, RECT_W, RECT_H)
    if not rep.check(4, "rectangle read back", missing == 0, "%d missing" % missing):
        return
    wrong = sum(1 for p in got if not same(p, RECT_VAL))
    rep.check(4, "every pixel in the rectangle is correct", wrong == 0,
              "%d of %d wrong" % (wrong, RECT_W * RECT_H))
    # And the pixels immediately outside it, which a missing flush corrupts.
    edge_ok = True
    for r in (RECT_ROW, RECT_ROW + RECT_H - 1):
        for c in (RECT_COL - 1, RECT_COL + RECT_W):
            if not (0 <= c < args.width):
                continue
            px, echo = read_one_pixel(board, r, c)
            if px is None or not echo or same(px, RECT_VAL):
                edge_ok = False
    rep.check(4, "pixels either side of the rectangle untouched", edge_ok, "")


# ---------------------------------------------------------------- stage 5
def stage5(board, rep, args):
    stage(5, "burst read placement -- burst vs per-pixel, same region")
    print("   A burst reply carries no address; a single pixel reply does.")
    print("   Reading the same window both ways is the cross-check.")
    x0, y0, w, h = 32, 32, 32, 32
    if x0 + w > args.width or y0 + h > args.height:
        x0 = y0 = 0
        w = min(32, args.width)
        h = min(32, args.height)
    a, ma = IT.read_image_burst(board, args.width, args.height, x0, y0, w, h,
                               retries=args.retries)
    b, mb = IT.read_image_pixels(board, args.width, args.height, x0, y0, w, h)
    if not rep.check(5, "both reads complete", ma == 0 and mb == 0,
                     "burst missing %d, per-pixel missing %d" % (ma, mb)):
        return
    diff = sum(1 for p, q in zip(a, b) if not same(p, q))
    rep.check(5, "burst placement matches per-pixel ground truth", diff == 0,
              "%d of %d pixels differ" % (diff, w * h))


# ---------------------------------------------------------------- stage 6
def stage6(board, rep, args, outdir):
    stage(6, "load a new image")
    if not args.image:
        print("   skipped -- pass --image PATH to exercise this")
        return
    try:
        src, sw, sh = IT.read_png(args.image)
    except Exception as e:
        rep.bad(6, "PNG decoded", str(e))
        return
    rep.ok(6, "PNG decoded", "%s, %dx%d" % (args.image, sw, sh))
    if (sw, sh) != (args.width, args.height):
        src, note = IT.fit_image(src, sw, sh, args.width, args.height, args.fit)
        print("   %s" % note)
    IT.write_image_burst(board, src, args.width, 0, 0, args.width, args.height)
    time.sleep(0.1)
    back, missing = IT.read_image_burst(board, args.width, args.height,
                                        retries=args.retries)
    if not rep.check(6, "image read back", missing == 0, "%d missing" % missing):
        return
    diff = sum(1 for p, q in zip(src, back) if not same(p, q))
    rep.check(6, "every pixel matches what was sent", diff == 0,
              "%d of %d differ" % (diff, args.width * args.height))
    path = os.path.join(outdir, "01_loaded.png")
    IT.write_png(path, back, args.width, args.height)
    rep.ok(6, "read-back written", path)


# ---------------------------------------------------------------- stage 7
def stage7(rep, args):
    stage(7, "flow control, both directions")
    print("   The board must go silent when RTS is deasserted, and must not")
    print("   lose replies when flow control is on. Both are checked here;")
    print("   flowtest.py is the full suite.")

    # (a) HOLD-OFF. Nothing is in flight yet, so there is no FIFO tail to
    #     argue about: after RTS is deasserted the line must be silent.
    link = FT.Link(args.port, args.baud, args.timeout, rtscts=False,
                   write_timeout=5.0)
    try:
        link.allow_send(False)                     # RTS deasserted
        time.sleep(0.05)
        link.discard_input()
        link.send(IT.msg_burst_read(0, min(16, args.height), args.width))
        link.flush()
        buf = bytearray()
        link.drain_for(args.holdoff, buf)
        rep.check(7, "silent while RTS is deasserted", len(buf) == 0,
                  "%d bytes in %.1f s" % (len(buf), args.holdoff))

        link.allow_send(True)                      # RTS asserted
        buf2 = bytearray()
        t0 = time.time()
        while time.time() - t0 < 4.0:
            if link.drain_for(0.05, buf2) == 0 and len(buf2) > 0:
                break
        rep.check(7, "data arrives once RTS is reasserted", len(buf2) > 0,
                  "%d bytes" % len(buf2))
    finally:
        link.close()

    # (b) CONTRAST. The same workload with flow control on and off. If the
    #     lossless run is not lossless the handshake is not working; if the
    #     lossy run loses nothing, the workload never stressed anything and
    #     the first result proves less than it appears to.
    n = args.flood

    def flood(rtscts):
        lk = FT.Link(args.port, args.baud, args.timeout, rtscts=rtscts,
                     write_timeout=5.0)
        try:
            if not rtscts:
                lk.allow_send(True)
            lk.discard_input()
            for i in range(n):
                lk.send(IT.msg_pixel_read((i // args.width) % args.height,
                                          i % args.width))
            lk.flush()
            buf = bytearray()
            t0 = time.time()
            while time.time() - t0 < 20.0:
                if lk.drain_for(0.05, buf) == 0 and len(buf) >= n * 16:
                    break
                if len(buf) >= n * 16:
                    break
            good = 0
            for j in range(0, min(len(buf), n * 16) - 15, 16):
                if IT.parse_pixel_reply(bytes(buf[j:j + 16])) is not None:
                    good += 1
            return good
        finally:
            lk.close()

    with_fc = flood(True)
    time.sleep(0.3)
    without_fc = flood(False)
    lost_with = n - with_fc
    lost_without = n - without_fc

    rep.check(7, "lossless with flow control ON", lost_with == 0,
              "%d of %d replies lost" % (lost_with, n))
    if not rep.check(7, "the negative control actually loses data",
                     lost_without > 0,
                     "%d of %d lost without flow control" % (lost_without, n)):
        print("   INCONCLUSIVE: nothing was lost even with flow control off,")
        print("   so this workload did not stress the link and the result")
        print("   above is unproven by contrast. Raise --flood.")


# ---------------------------------------------------------------- stage 8
def stage8(board, rep, args, baseline, outdir):
    stage(8, "restore the original image")
    if args.keep:
        print("   skipped -- --keep given, the board keeps what this run wrote")
        print("   The original is still saved at %s" %
              os.path.join(outdir, "00_original.png"))
        return
    IT.write_image_burst(board, baseline, args.width, 0, 0, args.width, args.height)
    time.sleep(0.1)
    back, missing = IT.read_image_burst(board, args.width, args.height,
                                        retries=args.retries)
    if not rep.check(8, "image read back after restore", missing == 0,
                     "%d missing" % missing):
        return
    diff = sum(1 for p, q in zip(baseline, back) if not same(p, q))
    rep.check(8, "board matches the stage 1 baseline exactly", diff == 0,
              "%d of %d pixels differ" % (diff, args.width * args.height))
    path = os.path.join(outdir, "02_restored.png")
    IT.write_png(path, back, args.width, args.height)
    rep.ok(8, "restored image written", path)


# ---------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(
        description="One run that exercises every pipeline on the board.")
    ap.add_argument("--port", required=True)
    ap.add_argument("--baud", type=int, default=8125000)
    ap.add_argument("--timeout", type=float, default=0.5)
    ap.add_argument("--width", type=int, default=256)
    ap.add_argument("--height", type=int, default=256)
    ap.add_argument("--image", help="PNG to load in stage 6")
    ap.add_argument("--fit", default="exact", choices=("exact", "scale", "crop"))
    ap.add_argument("--keep", action="store_true",
                    help="skip the restore; leave this run's writes on the board")
    ap.add_argument("--out", default="final_test_out", help="where PNGs go")
    ap.add_argument("--verify", type=int, default=16,
                    help="address-echo samples in stage 1")
    ap.add_argument("--retries", type=int, default=2)
    ap.add_argument("--holdoff", type=float, default=2.0,
                    help="stage 7: seconds of silence required")
    ap.add_argument("--flood", type=int, default=2000,
                    help="stage 7: pixel reads per contrast run")
    ap.add_argument("--skip-flow", action="store_true")
    args = ap.parse_args()

    print("=" * 68)
    print("  ACCEPTANCE RUN  --  port %s, %d baud, image %dx%d"
          % (args.port, args.baud, args.width, args.height))
    print("  Every result below comes from the board. Nothing is simulated,")
    print("  defaulted or composited locally.")
    print("=" * 68)

    try:
        board = IT.Board(args.port, args.baud, args.timeout)
    except serial.SerialException as e:
        print("\n  could not open %s: %s" % (args.port, e))
        print("  NOTHING WAS RUN.")
        return 2

    rep = Report()
    outdir = args.out
    baseline = None

    try:
        if stage0(board, rep, args) is None:
            print("\n  The board did not answer. NOTHING ELSE WAS RUN, no file")
            print("  was written and nothing on the board was modified.")
            print("  Check the bitstream is programmed, then run board_test.py.")
            return 2

        os.makedirs(outdir, exist_ok=True)

        baseline = stage1(board, rep, args, outdir)
        if baseline is None:
            print("\n  The baseline capture failed, so the original image could")
            print("  not be saved. STOPPING BEFORE ANY WRITE -- restoring it")
            print("  afterwards would be impossible without reprogramming.")
            return 1

        stage2(board, rep)
        stage3(board, rep, args)
        stage4(board, rep, args)
        stage5(board, rep, args)
        stage6(board, rep, args, outdir)

        if not args.skip_flow:
            board.close()
            stage7(rep, args)
            board = IT.Board(args.port, args.baud, args.timeout)
        else:
            stage(7, "flow control")
            print("   skipped -- --skip-flow given")

        stage8(board, rep, args, baseline, outdir)

    except KeyboardInterrupt:
        print("\n  interrupted")
        if baseline is not None and not args.keep:
            print("  The board may hold this run's writes. Restore with:")
            print("    python image_tool.py --port %s load %s"
                  % (args.port, os.path.join(outdir, "00_original.png")))
        return 130
    finally:
        try:
            board.close()
        except Exception:
            pass

    print("\n" + "=" * 68)
    by_stage = {}
    for st, res, what, _ in rep.rows:
        d = by_stage.setdefault(st, [0, 0])
        d[0 if res == "PASS" else 1] += 1
    for st in sorted(by_stage):
        p, f = by_stage[st]
        print("  stage %s : %d passed, %d failed" % (st, p, f))
    print("  ----")
    print("  checks  : %d" % len(rep.rows))
    print("  failed  : %d" % rep.failed)
    print("  RESULT  : %s" % ("PASS" if rep.failed == 0 else "FAIL"))
    if not args.keep and baseline is not None:
        print("  The board has been restored to the image captured in stage 1.")
    print("=" * 68)
    return 0 if rep.failed == 0 else 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except serial.SerialException as e:
        sys.exit("\nserial error: %s" % e)
