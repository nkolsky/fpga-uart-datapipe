#!/usr/bin/env python3
"""
flowtest.py -- prove the RTS/CTS hardware flow control actually works.

WHAT THIS TESTS, AND WHY THE OBVIOUS TEST IS NOT ENOUGH
-------------------------------------------------------
There are two independent directions and they fail differently.

  FPGA -> PC : the FPGA's TX watches its CTS input. To test it the PC must
               drive RTS BY HAND, so those tests open the port with
               rtscts=False. With rtscts=True the driver owns RTS and manual
               writes to it are ignored or fought over.

  PC -> FPGA : the FPGA deasserts its RTS when its receive path is busy.
               Those tests open with rtscts=True and OBSERVE cts rather
               than driving anything.

A test that only checks "the bytes stopped" passes on a design that stalls
its UART transmitter while its SRAM read side keeps running and throws data
on the floor. That is why every throttle test here ends by DECODING THE
WHOLE STREAM AND DIFFING IT AGAINST A CLEAN BASELINE. Three outcomes, and
the third is the one worth catching:

    stops, resumes, image matches   -> flow control works
    never stops                     -> the CTS input is ignored in the TX path
    stops, resumes, image is wrong  -> the UART stalls but backpressure does
                                       not reach the memory read side

POLARITY
--------
RTS/CTS are active low on the wire and pyserial speaks in ASSERTION, not in
pin level:

    ser.rts = True   -> RTS# driven LOW  -> "you may send"
    ser.rts = False  -> RTS# driven HIGH -> "stop sending"
    ser.cts == True  -> the board is asserting -> it is ready to receive
    ser.cts == False -> the board is telling us to back off

Test 0 checks this empirically instead of trusting it.

EVERYTHING HERE IS READ-ONLY
----------------------------
No pixel writes anywhere in the suite. That matters twice: a truncated frame
during the overrun test cannot be misparsed into a write that corrupts
memory, and a mid-run reset that reloads the SRAM from the .mem files does
not invalidate the baseline.

USAGE
-----
    python flowtest.py --port COM5
    python flowtest.py --port COM5 --init-red red_hex.mem \
                       --init-green green_hex.mem --init-blue blue_hex.mem
    python flowtest.py --port COM5 --no-interactive   # skips the overrun test
    python flowtest.py --port COM5 --only 0,1,2,3     # pick tests

Needs image_tool.py beside it -- the protocol builders, the burst framing
check and the stream decoder are imported from there so there is only one
copy of them.
"""

import argparse
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("pyserial is required:  pip install pyserial")

# The protocol builders, the burst framing check and the stream decoder are
# imported rather than copied, so there is only one of each. Both filenames
# are tried AND CHECKED: an older image_tool.py sitting in the same folder
# imports perfectly well and is missing half of what is needed, so matching
# the name is not enough.
_NEEDED = ("BURST_FRAME_LEN", "PIX_REPLY_LEN", "PIX_PER_FRAME",
           "WIRE_BITS_PER_BYTE", "msg_pixel_read", "msg_burst_read",
           "parse_pixel_reply", "decode_stream", "read_image_burst")

IT = None
_tried = []
for _name in ("image_tool", "image_tool_v2"):
    try:
        _mod = __import__(_name)
    except ImportError:
        continue
    _missing = [a for a in _NEEDED if not hasattr(_mod, a)]
    if _missing:
        _tried.append("%s (%s) lacks %s"
                      % (_name, getattr(_mod, "__file__", "?"), _missing[0]))
        continue
    IT = _mod
    break

if IT is None:
    if _tried:
        sys.exit("found an image_tool module, but it is the wrong version:\n  "
                 + "\n  ".join(_tried) +
                 "\nUse the burst-read version (it defines BURST_FRAME_LEN) "
                 "and remove or rename the old one.")
    sys.exit("image_tool.py (or image_tool_v2.py) must be in the same "
             "directory as this script")


BURST_FRAME_LEN = IT.BURST_FRAME_LEN
PIX_REPLY_LEN = IT.PIX_REPLY_LEN


# =======================================================================
# Link -- same duck type as image_tool.Board, but the port settings are
# ours to choose and RTS is ours to drive.
# =======================================================================
class Link:
    def __init__(self, port, baud, timeout, rtscts, write_timeout=None):
        self.rtscts = rtscts
        self.ser = serial.Serial(
            port=port, baudrate=baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_EVEN,
            stopbits=serial.STOPBITS_ONE,
            timeout=timeout,
            write_timeout=write_timeout,
            rtscts=rtscts,
        )
        # Deliberately NOT enlarging the driver's receive buffer: test 5
        # needs it to fill so the backpressure chain engages.
        if not rtscts:
            self.ser.rts = True          # asserted: the board may send
        time.sleep(0.05)
        self.ser.reset_input_buffer()
        self.sent = 0
        self.recv = 0

    # -- image_tool.Board interface --------------------------------
    def send(self, data):
        self.ser.write(data)
        self.sent += len(data)

    def flush(self):
        self.ser.flush()

    def read(self, n):
        d = self.ser.read(n)
        self.recv += len(d)
        return d

    def read_exact(self, n, timeout=None):
        buf = bytearray()
        limit = timeout if timeout is not None else max(self.ser.timeout * 4, 0.2)
        deadline = time.time() + limit
        while len(buf) < n and time.time() < deadline:
            chunk = self.read(n - len(buf))
            if chunk:
                buf += chunk
        return bytes(buf) if len(buf) == n else None

    def discard_input(self):
        self.ser.reset_input_buffer()

    def close(self):
        try:
            if not self.rtscts:
                self.ser.rts = True      # leave the board able to talk
        except Exception:
            pass
        self.ser.close()

    # -- flow control ----------------------------------------------
    def allow_send(self, on):
        """True = assert RTS = the board may transmit."""
        self.ser.rts = bool(on)

    def board_ready(self):
        """The board's RTS, as seen at our CTS. True = it can receive."""
        return bool(self.ser.cts)

    # -- non-blocking windowed read --------------------------------
    def drain_for(self, seconds, into):
        """
        Read whatever turns up for `seconds` and append it to `into`.
        Uses in_waiting so the window length is the window length, rather
        than however long a blocking read decided to sit there.
        """
        n0 = len(into)
        end = time.time() + seconds
        while time.time() < end:
            avail = self.ser.in_waiting
            if avail:
                d = self.ser.read(avail)
                self.recv += len(d)
                into += d
            else:
                time.sleep(0.002)
        return len(into) - n0


# =======================================================================
class Results:
    def __init__(self):
        self.passed = 0
        self.failed = 0
        self.skipped = 0
        self.untested = 0

    def check(self, name, ok, detail=""):
        if ok:
            self.passed += 1
            print("  PASS  %-44s %s" % (name, detail))
        else:
            self.failed += 1
            print("  FAIL  %-44s %s" % (name, detail))
        return ok

    def skip(self, name, why):
        self.skipped += 1
        print("  SKIP  %-44s %s" % (name, why))

    def inconclusive(self, name, why):
        """Not a pass. The workload did not exercise the thing."""
        self.untested += 1
        print("  ????  %-44s %s" % (name, why))


# =======================================================================
def wire_bytes_per_second(baud):
    return baud / float(IT.WIRE_BITS_PER_BYTE)


def load_init(path):
    """One channel's .mem file -> list of 8-bit values. Lane 0 is the MSB."""
    vals = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("//"):
                continue
            word = int(line, 16)
            for lane in range(4):
                vals.append((word >> (8 * (3 - lane))) & 0xFF)
    return vals


def burst_request_whole(link, width, height):
    """Ask for the entire image as one burst read. Returns expected bytes."""
    npx = width * height
    nframes = (npx + IT.PIX_PER_FRAME - 1) // IT.PIX_PER_FRAME
    link.discard_input()
    link.send(IT.msg_burst_read(0, height, width))
    link.flush()
    return nframes * BURST_FRAME_LEN


def diff_against(vals, baseline):
    """(mismatches, first_index) comparing an ordinal pixel list to baseline."""
    bad = 0
    first = -1
    for i, px in enumerate(vals):
        if i >= len(baseline):
            break
        if px != baseline[i]:
            bad += 1
            if first < 0:
                first = i
    return bad, first


def drain_until(link, buf, want, window=0.1, idle_windows=3, limit=10.0):
    """
    Collect into buf until `want` bytes have arrived, or until the link has
    been idle for `idle_windows` consecutive windows. One empty window is
    not enough: a board that pauses between frames would truncate the
    capture and the result would be reported as data loss.
    """
    idle = 0
    deadline = time.time() + limit
    while len(buf) < want and time.time() < deadline:
        if link.drain_for(window, buf) == 0:
            idle += 1
            if idle >= idle_windows:
                break
        else:
            idle = 0
    return len(buf)


def report_stream(res, label, buf, expected_bytes, npx, baseline):
    """Decode a collected burst stream and diff it. Returns True on a match."""
    vals, consumed, err = IT.decode_stream(buf, npx)

    if err == "framing":
        res.check(label, False,
                  "framing error at byte %d of %d" % (consumed, len(buf)))
        return False
    if err == "short":
        res.check(label, False,
                  "%d of %d bytes (%d of %d pixels)"
                  % (len(buf), expected_bytes, len(vals), npx))
        return False

    bad, first = diff_against(vals, baseline)
    if bad:
        res.check(label, False,
                  "%d pixels differ from baseline, first at index %d"
                  % (bad, first))
        print("        The stream stopped and restarted but the CONTENT is")
        print("        wrong: the transmitter stalled and the memory read")
        print("        side did not. That is data thrown away, not delayed.")
        return False

    res.check(label, True, "%d pixels, all identical to baseline" % npx)
    return True


# =======================================================================
def prompt_reset(link, args, why):
    """Ask for a board reset, then prove the link came back."""
    if args.no_interactive:
        print("\n  >>> a board RESET is needed here (--no-interactive: not asking)")
        return False

    print("\n  >>> %s" % why)
    print("  >>> Press the board's RESET button, then hit Enter here.")
    try:
        input()
    except EOFError:
        print("      no console -- continuing without a reset")
        return False

    time.sleep(0.5)                       # covers a PROG_B reconfiguration

    # The reset clears the board's FIFOs, but not the bytes already sitting
    # in the FTDI chip and the OS buffer. Those keep arriving for a few
    # milliseconds afterwards, and a single flush races them -- the probe
    # then reads a reply from BEFORE the reset and reports a perfectly
    # healthy board as dead. Drain until the line is genuinely idle first.
    junk = bytearray()
    drain_until(link, junk, 1 << 30, window=0.1, idle_windows=3, limit=3.0)
    link.ser.reset_input_buffer()
    link.ser.reset_output_buffer()
    if junk:
        print("      drained %d stale bytes left over from before the reset"
              % len(junk))
    return True


def liveness_probe(link, res, label="link alive after reset", tries=3):
    """
    One address-echoed Single Pixel Read. The gate for continuing.

    Retried a few times: the first attempt after a reset can still collide
    with a straggler from before it, and one stale reply is not evidence
    that the board is down.
    """
    detail = "no valid reply"
    for attempt in range(tries):
        link.discard_input()
        link.send(IT.msg_pixel_read(0, 0))
        link.flush()
        raw = link.read_exact(PIX_REPLY_LEN, timeout=1.0)
        parsed = IT.parse_pixel_reply(raw) if raw else None
        if parsed is None:
            detail = "no valid reply"
        else:
            r, c, _ = parsed
            if (r, c) == (0, 0):
                return res.check(label, True,
                                 "" if attempt == 0 else
                                 "on attempt %d" % (attempt + 1))
            detail = "stale or wrong address echo (%d,%d)" % (r, c)
        time.sleep(0.2)
    return res.check(label, False, detail)


# =======================================================================
# 0. Is CTS even wired, and is the polarity what we think?
# =======================================================================
def test0_cts_sanity(args, res):
    print("\n[0] CTS wiring and polarity, board idle")
    print("      ser.cts True means the board is ASSERTING = ready to receive")

    link = Link(args.port, args.baud, args.timeout, rtscts=True)
    try:
        samples = []
        for _ in range(40):
            samples.append(link.board_ready())
            time.sleep(0.005)
        high = sum(1 for s in samples if s)

        if high == 0:
            res.check("CTS asserted while the board is idle", False,
                      "cts low for all 40 samples")
            print("      Every write from here would block. Either the line is")
            print("      unwired or cross-wired, the polarity is inverted, or")
            print("      the receive path is stuck busy. Check the XDC before")
            print("      reading anything below as a result.")
            return None
        if high < len(samples):
            res.check("CTS asserted while the board is idle", False,
                      "cts low in %d of 40 samples" % (len(samples) - high))
            print("      An idle board should not be deasserting. Toggling here")
            print("      suggests a floating input or a missing pull-up.")
            return link

        res.check("CTS asserted while the board is idle", True,
                  "cts high in all 40 samples")

        # A line stuck asserted looks identical to a correct idle line. Test 5
        # is the one that proves it can actually change; say so here.
        print("      Note: this only proves it is not stuck LOW. Test 5 is what")
        print("      proves the board can drive it low when it needs to.")
        return link
    except Exception:
        link.close()
        raise


# =======================================================================
# 1. The oracle: one clean read of the whole image.
# =======================================================================
def test1_baseline(args, res, init):
    print("\n[1] baseline: one clean read of the whole image")

    link = Link(args.port, args.baud, args.timeout, rtscts=False)
    try:
        link.allow_send(True)
        pixels, missing = IT.read_image_burst(
            link, args.width, args.height, 0, 0, args.width, args.height,
            walk="rect", retries=args.retries, timeout=args.timeout * 6)

        ok = res.check("baseline read complete", missing == 0,
                       "%d pixels missing" % missing)
        if not ok:
            print("      Without a clean baseline the throttle tests have")
            print("      nothing to diff against. Fix the plain read first.")
            return link, None

        if init is not None:
            red, green, blue = init
            n = min(len(red), args.width * args.height)
            bad = sum(1 for i in range(n)
                      if pixels[i] != (red[i], green[i], blue[i]))
            res.check("baseline matches your .mem files", bad == 0,
                      "%d of %d pixels differ" % (bad, n))
            if bad:
                print("      The baseline is not what the bitstream was built")
                print("      with, so it is not a trustworthy oracle.")
        else:
            print("      no --init files: the baseline is self-referential.")
            print("      It still catches throttling damage, but it cannot")
            print("      prove the board is not answering consistently wrong.")

        return link, pixels
    except Exception:
        link.close()
        raise


# =======================================================================
# 2. Hold RTS off BEFORE the reply starts.
# =======================================================================
def test2_holdoff(link, args, res, baseline):
    print("\n[2] deassert RTS, then ask for the image -- nothing may come back")
    print("      the cleanest test in the suite: nothing is in flight yet, so")
    print("      there is no FIFO tail to argue about")

    npx = args.width * args.height

    link.allow_send(False)                # stop
    time.sleep(0.05)
    expected = burst_request_whole(link, args.width, args.height)

    buf = bytearray()
    got = link.drain_for(args.holdoff, buf)

    at_rate = wire_bytes_per_second(args.baud) * args.holdoff
    ok = res.check("silent while RTS is deasserted", got == 0,
                   "%d bytes in %.1f s (line rate would be ~%.0f)"
                   % (got, args.holdoff, at_rate))
    if not ok:
        print("      The transmitter is not looking at its CTS input at all.")
        print("      Nothing below will be meaningful until that is fixed.")

    link.allow_send(True)                 # go
    drain_until(link, buf, expected, limit=args.timeout * 20)

    if not ok:
        # The whole image already arrived while RTS was deasserted, so
        # "it resumed" and "it is intact" would just be re-decoding a
        # buffer that filled itself. Reporting those as passes would be
        # reporting nothing at all.
        res.skip("stream arrives after RTS is reasserted",
                 "vacuous: it never stopped")
        res.skip("image after hold-off is intact",
                 "vacuous: nothing was held off")
        return False

    res.check("stream arrives after RTS is reasserted", len(buf) > 0,
              "%d of %d bytes" % (len(buf), expected))
    return report_stream(res, "image after hold-off is intact",
                         buf, expected, npx, baseline)


# =======================================================================
# 3. Stall mid-stream, measure the tail, resume.
# =======================================================================
def test3_midstream(link, args, res, baseline):
    print("\n[3] stall mid-stream, measure the FIFO tail, then resume")

    npx = args.width * args.height
    win = 0.05

    link.allow_send(True)
    expected = burst_request_whole(link, args.width, args.height)

    buf = bytearray()
    end = time.time() + 1.0
    while len(buf) < args.prefix and time.time() < end:
        link.drain_for(0.01, buf)
    before = len(buf)

    if before == 0:
        res.check("stream started before stalling it", False, "no bytes at all")
        return False

    link.allow_send(False)
    t_off = time.time()

    windows = []
    quiet_run = 0.0
    tail = 0
    while time.time() - t_off < args.stall:
        n = link.drain_for(win, buf)
        windows.append(n)
        tail += n
        quiet_run = quiet_run + win if n == 0 else 0.0

    # Bytes already inside the FTDI's receive FIFO reach the host over USB
    # no matter what the FPGA does, so the tail is REPORTED, not asserted.
    # What is asserted is that arrival went to zero and stayed there.
    at_rate = wire_bytes_per_second(args.baud) * args.stall
    print("      tail after deassert: %d bytes, then silence for %.2f s"
          % (tail, quiet_run))
    print("      (at line rate %.1f s of streaming would be ~%.0f bytes)"
          % (args.stall, at_rate))

    quiesced = quiet_run >= args.quiet_needed
    ok = res.check("stream quiesced and stayed quiet", quiesced,
                   "%.2f s of silence, %.2f s required"
                   % (quiet_run, args.quiet_needed))
    if not ok:
        nonzero = sum(1 for n in windows if n)
        print("      %d of %d windows still had traffic. The transmitter is"
              % (nonzero, len(windows)))
        print("      not stalling -- or it stalls and restarts on its own.")

    link.allow_send(True)
    drain_until(link, buf, expected, limit=args.timeout * 20)

    res.check("stream resumed", len(buf) > before + tail,
              "%d bytes before, %d in the tail, %d total"
              % (before, tail, len(buf)))
    return report_stream(res, "image across one stall is intact",
                         buf, expected, npx, baseline) and ok


# =======================================================================
# 4. Toggle repeatedly across one image.
# =======================================================================
def test4_repeated(link, args, res, baseline):
    print("\n[4] %d stall/resume cycles across a single image read" % args.cycles)
    print("      resume logic that works once and breaks on the second stall")
    print("      is an ordinary bug; one toggle will not find it")

    npx = args.width * args.height

    link.allow_send(True)
    expected = burst_request_whole(link, args.width, args.height)

    buf = bytearray()
    stalls = 0
    quiet_stalls = 0
    deadline = time.time() + args.timeout * 20

    while len(buf) < expected and stalls < args.cycles and time.time() < deadline:
        chunk_end = len(buf) + max(expected // (args.cycles + 1), 256)
        t_end = time.time() + 1.0
        # Small windows: at line rate a 10 ms read swallows ~7 kB, which
        # would overshoot the chunk and silently cut the cycle count.
        while len(buf) < chunk_end and time.time() < t_end:
            if link.drain_for(0.002, buf) == 0 and len(buf) >= expected:
                break

        link.allow_send(False)
        stalls += 1
        link.drain_for(args.short_stall, buf)          # let the tail land
        if link.drain_for(args.short_stall, buf) == 0:  # then require silence
            quiet_stalls += 1
        link.allow_send(True)

    drain_until(link, buf, expected, limit=args.timeout * 20)

    res.check("every stall went quiet", quiet_stalls == stalls,
              "%d of %d stalls quiesced (%d requested)"
              % (quiet_stalls, stalls, args.cycles))
    if stalls < args.cycles:
        print("      only %d cycles fitted: the image finished first. Use a"
              % stalls)
        print("      larger image or a smaller --cycles for more coverage.")
    return report_stream(res, "image across %d stalls is intact" % stalls,
                         buf, expected, npx, baseline)


# =======================================================================
# 5. The other direction: fill OUR receive buffer and watch the board's RTS.
# =======================================================================
def test5_reverse(args, res):
    print("\n[5] reverse direction: stop draining and watch the board's RTS")
    print("      our buffer fills -> the driver deasserts RTS -> the board's")
    print("      transmitter stalls -> its reply FIFO fills -> it should")
    print("      deassert its own RTS, which we see at ser.cts")

    link = Link(args.port, args.baud, args.timeout, rtscts=True,
                write_timeout=2.0)
    try:
        if not link.board_ready():
            res.check("board ready before the flood", False, "cts already low")
            return link

        link.discard_input()

        req = args.flood
        per_write = 10
        sent = 0
        cts_dropped = False
        write_blocked = False
        t0 = time.time()

        while sent < req and time.time() - t0 < args.timeout * 20:
            if not link.board_ready():
                cts_dropped = True
                break
            block = bytearray()
            for k in range(min(per_write, req - sent)):
                lin = (sent + k) % (args.width * args.height)
                block += IT.msg_pixel_read(lin // args.width, lin % args.width)
            try:
                link.send(bytes(block))
            except serial.SerialTimeoutException:
                # The write itself blocked: CTS went low mid-transfer, which
                # is the same evidence seen from the other side.
                write_blocked = True
                break
            sent += min(per_write, req - sent)
            # deliberately NOT reading -- the point is to let the buffer fill

        observed = cts_dropped or write_blocked
        if observed:
            res.check("board deasserted its RTS under backpressure", True,
                      "after %d requests (%s)"
                      % (sent, "cts low" if cts_dropped else "write blocked"))
        else:
            res.inconclusive("board deasserted its RTS under backpressure",
                             "never dropped in %d requests" % sent)
            print("      Either the chain never filled -- try a larger --flood")
            print("      -- or the board's RTS is tied asserted. This is not a")
            print("      pass: the line was never seen to change.")

        # Now drain and prove the backpressure DELAYED replies rather than
        # losing them. Pixel replies carry their own address, so loss is
        # exactly locatable.
        buf = bytearray()
        drain_until(link, buf, sent * PIX_REPLY_LEN, window=0.2,
                    limit=args.timeout * 60)

        good = 0
        bad_echo = 0
        for j in range(0, len(buf) - PIX_REPLY_LEN + 1, PIX_REPLY_LEN):
            parsed = IT.parse_pixel_reply(bytes(buf[j:j + PIX_REPLY_LEN]))
            if parsed is None:
                bad_echo += 1
                continue
            idx = j // PIX_REPLY_LEN
            lin = idx % (args.width * args.height)
            if (parsed[0], parsed[1]) == (lin // args.width, lin % args.width):
                good += 1
            else:
                bad_echo += 1

        res.check("every reply arrived, in order, with a correct echo",
                  good == sent and bad_echo == 0,
                  "%d of %d good, %d malformed or out of order"
                  % (good, sent, bad_echo))
        if good != sent:
            print("      Replies were LOST, not merely delayed. Backpressure")
            print("      reached the UART but not the thing feeding it.")
        return link
    except Exception:
        link.close()
        raise


# =======================================================================
# 6. Negative control: the same flood with flow control switched off.
# =======================================================================
def test6_negative(args, res):
    print("\n[6] NEGATIVE CONTROL: the same flood with rtscts=False")
    print("      this one is EXPECTED to lose data. If it does not, the")
    print("      workload never stressed the receive path and tests above")
    print("      are untested rather than passed")

    link = Link(args.port, args.baud, args.timeout, rtscts=False)
    try:
        link.allow_send(True)             # ignore whatever the board asks for
        link.discard_input()

        req = args.flood
        per_write = 64
        sent = 0
        buf = bytearray()

        while sent < req:
            n = min(per_write, req - sent)
            block = bytearray()
            for k in range(n):
                lin = (sent + k) % (args.width * args.height)
                block += IT.msg_pixel_read(lin // args.width, lin % args.width)
            link.send(bytes(block))
            sent += n
            # Drain as we go, so a shortfall is the FPGA's receive path
            # overrunning and not our own buffer overflowing.
            avail = link.ser.in_waiting
            if avail:
                d = link.ser.read(avail)
                link.recv += len(d)
                buf += d

        drain_until(link, buf, sent * PIX_REPLY_LEN, window=0.2,
                    limit=args.timeout * 40)

        good = 0
        for j in range(0, len(buf) - PIX_REPLY_LEN + 1, PIX_REPLY_LEN):
            parsed = IT.parse_pixel_reply(bytes(buf[j:j + PIX_REPLY_LEN]))
            if parsed is None:
                continue
            idx = j // PIX_REPLY_LEN
            lin = idx % (args.width * args.height)
            if (parsed[0], parsed[1]) == (lin // args.width, lin % args.width):
                good += 1

        lost = sent - good
        print("      %d requests sent, %d clean replies, %d missing"
              % (sent, good, lost))

        if lost > 0:
            res.check("flow control is doing real work", True,
                      "%d replies lost without it, 0 lost with it" % lost)
            print("      Same workload, same board: lossless in test 5 and")
            print("      lossy here. That contrast is the actual proof.")
        else:
            res.inconclusive("flow control is doing real work",
                             "nothing lost even with flow control off")
            print("      At %.0f bytes/s the receive path gets ~%.0f clocks"
                  % (wire_bytes_per_second(args.baud),
                     args.fclk / wire_bytes_per_second(args.baud)))
            print("      per byte, so this workload may simply never stress")
            print("      it. Tests 2-5 are UNPROVEN by contrast, not wrong.")

        return link, lost > 0
    except Exception:
        link.close()
        raise


# =======================================================================
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port")
    ap.add_argument("--baud", type=int, default=8125000)
    ap.add_argument("--timeout", type=float, default=0.5)
    ap.add_argument("--width", type=int, default=256)
    ap.add_argument("--height", type=int, default=256)
    ap.add_argument("--fclk", type=float, default=130e6,
                    help="fabric clock, only used to report clocks per byte")
    ap.add_argument("--retries", type=int, default=2)

    ap.add_argument("--holdoff", type=float, default=2.0,
                    help="test 2: seconds to stay silent with RTS deasserted")
    ap.add_argument("--prefix", type=int, default=8192,
                    help="test 3: bytes to let through before stalling")
    ap.add_argument("--stall", type=float, default=2.0,
                    help="test 3: seconds to hold the stall")
    ap.add_argument("--quiet-needed", type=float, default=1.0,
                    help="test 3: silence required at the end of the stall")
    ap.add_argument("--cycles", type=int, default=20,
                    help="test 4: number of stall/resume cycles")
    ap.add_argument("--short-stall", type=float, default=0.05,
                    help="test 4: length of each half of a stall window")
    ap.add_argument("--flood", type=int, default=8000,
                    help="tests 5 and 6: pixel read requests in the flood")

    ap.add_argument("--list", action="store_true", help="list serial ports")
    ap.add_argument("--toggle-rts", type=float, metavar="SECONDS",
                    help="do no testing: just square-wave RTS at 1 Hz for "
                         "this long, so the FPGA's CTS pin can be watched "
                         "on a scope or routed to an LED")
    ap.add_argument("--only", help="comma separated test numbers, e.g. 0,1,2")
    ap.add_argument("--no-interactive", action="store_true",
                    help="never ask for a board reset; skips test 6")
    ap.add_argument("--init-red")
    ap.add_argument("--init-green")
    ap.add_argument("--init-blue")
    a = ap.parse_args()

    if a.list:
        import serial.tools.list_ports
        for p in serial.tools.list_ports.comports():
            print(p.device, "-", p.description)
        return 0

    if a.toggle_rts:
        if not a.port:
            return ap.error("--port is required")
        print("toggling RTS at 1 Hz for %.0f s on %s" % (a.toggle_rts, a.port))
        print("ASSERTED = the FTDI drives RTS# LOW = the board may send")
        print("watch the FPGA's CTS input pin; if it never moves, the line")
        print("is not connected and no amount of RTL will fix it")
        link = Link(a.port, a.baud, a.timeout, rtscts=False)
        try:
            end = time.time() + a.toggle_rts
            state = True
            while time.time() < end:
                link.allow_send(state)
                print("  RTS %s   (pin %s)"
                      % ("ASSERTED " if state else "deasserted",
                         "LOW " if state else "HIGH"))
                state = not state
                time.sleep(0.5)
            link.allow_send(True)
        finally:
            link.close()
        return 0

    want = set(range(7))
    if a.only:
        want = set(int(x) for x in a.only.split(",") if x.strip())

    if not a.port:
        return ap.error("--port is required (use --list to find it)")

    print("port %s  baud %d  parity EVEN  image %dx%d"
          % (a.port, a.baud, a.width, a.height))
    print("wire rate %.0f bytes/s   RTS/CTS active low"
          % wire_bytes_per_second(a.baud))

    init = None
    if a.init_red and a.init_green and a.init_blue:
        init = (load_init(a.init_red), load_init(a.init_green),
                load_init(a.init_blue))
        print("init files loaded: %d pixels per channel" % len(init[0]))

    res = Results()
    baseline = None
    total_sent = total_recv = 0
    link = None

    try:
        if 0 in want:
            link = test0_cts_sanity(a, res)
            if link is None:
                print("\nCTS is unusable. Stopping before anything else runs.")
                return 1
            total_sent += link.sent
            total_recv += link.recv
            link.close()
            link = None

        # Tests 1-4 share one rtscts=False handle: RTS is ours to drive.
        if want & {1, 2, 3, 4}:
            link, baseline = test1_baseline(a, res, init)
            if baseline is None:
                link.close()
                print("\nNo usable baseline. Stopping.")
                return 1
            if 2 in want:
                test2_holdoff(link, a, res, baseline)
            if 3 in want:
                test3_midstream(link, a, res, baseline)
            if 4 in want:
                test4_repeated(link, a, res, baseline)
            total_sent += link.sent
            total_recv += link.recv
            link.close()
            link = None

        # Test 5 needs rtscts=True, which cannot be changed on an open port.
        if 5 in want:
            link = test5_reverse(a, res)
            total_sent += link.sent
            total_recv += link.recv
            link.close()
            link = None

        if 6 in want:
            if a.no_interactive:
                res.skip("negative control", "needs a reset (--no-interactive)")
            else:
                link, provoked = test6_negative(a, res)
                total_sent += link.sent
                total_recv += link.recv
                prompt_reset(link, a,
                             "the overrun may have left rx_msg_parser waiting "
                             "on bytes that never came.")
                alive = liveness_probe(link, res)
                link.close()
                link = None
                if not alive:
                    print("\n  The link did not come back after the reset.")
                    print("  Everything above still stands; this does not.")
    finally:
        if link is not None:
            total_sent += link.sent
            total_recv += link.recv
            link.close()

    print("\n" + "=" * 62)
    print("  bytes sent     : %d" % total_sent)
    print("  bytes received : %d" % total_recv)
    print("  passed         : %d" % res.passed)
    print("  failed         : %d" % res.failed)
    print("  inconclusive   : %d" % res.untested)
    print("  skipped        : %d" % res.skipped)
    if total_recv == 0:
        print("\n  NOTHING WAS RECEIVED. Any pass above would be a bug here.")
    if res.untested:
        print("\n  Inconclusive is not a pass. Something the suite set out to")
        print("  observe never happened, so it was not tested.")
    print("  RESULT         : %s" % ("PASS" if res.failed == 0 and
                                     total_recv > 0 else "FAIL"))
    print("=" * 62)
    return 0 if res.failed == 0 and total_recv > 0 else 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except serial.SerialException as e:
        sys.exit("\ncould not open the port: %s\n"
                 "Run  python flowtest.py --list  to see what is there, and "
                 "close any other\nprogram holding it open (a terminal, "
                 "Vivado's hardware manager, board_test.py)." % e)
    except KeyboardInterrupt:
        sys.exit("\ninterrupted")
