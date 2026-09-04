#!/usr/bin/env python3
"""
board_test.py -- talk to the FPGA over UART and PROVE the board answered.

The point of this script is not to produce a pretty image. It is to make it
impossible to mistake a script that works for a design that works.

-----------------------------------------------------------------------
HOW A HOST SCRIPT FAKES SUCCESS
-----------------------------------------------------------------------
The failure mode is quiet: the script keeps a local copy of the image, the
board never replies or replies with rubbish, and the comparison is made
against the local copy rather than against anything that came back. You get
your picture and learn nothing.

Every test here is built so that CANNOT happen:

  1. READ BEFORE WRITE. The first test reads pixels the script has never
     written. The expected values come from the SRAM initialisation files,
     so the only way to produce them is to actually read the board's memory.

  2. NO SILENT FALLBACK. Every read has a timeout. A timeout is a FAILURE,
     never a default value. The script never invents a pixel.

  3. NEGATIVE CONTROL. After writing a pattern, it reads an address it did
     NOT write and checks the value is the OLD one. A script echoing its own
     writes back would fail this -- it would report the pattern everywhere.

  4. BYTE ACCOUNTING. Bytes sent and received are counted and printed. Zero
     bytes received with tests "passing" is impossible: the tests depend on
     received bytes.

  5. ADDRESS ECHO. A pixel reply carries its own row and column. The script
     checks they match what was asked for, so a board replying with a
     constant, or replies arriving out of order, are both caught.

-----------------------------------------------------------------------
FLOW CONTROL
-----------------------------------------------------------------------
RTS/CTS is enabled (rtscts=True). The design deasserts CTS while the receive
path is busy, and this script must honour it -- without flow control the PC
will overrun the FPGA during a burst and the failure looks like data
corruption rather than what it is.

-----------------------------------------------------------------------
USAGE
-----------------------------------------------------------------------
    python board_test.py --port COM4
    python board_test.py --port /dev/ttyUSB1 --baud 8125000
    python board_test.py --port COM4 --skip-burst
    python board_test.py --list

Requires pyserial:  pip install pyserial
"""

import argparse
import sys
import time

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    sys.exit("pyserial is required:  pip install pyserial")


# =======================================================================
# Protocol
# =======================================================================
OPEN, CLOSE, COMMA = ord('{'), ord('}'), ord(',')
CH = {c: ord(c) for c in "WRVCPIH"}

PIX_REPLY_LEN = 16
REG_REPLY_LEN = 6


def be24(v):
    """Three bytes, most significant first -- the protocol's field layout."""
    v &= 0xFFFFFF
    return bytes([(v >> 16) & 0xFF, (v >> 8) & 0xFF, v & 0xFF])


def msg_pixel_write(addr, r, g, b):
    """{W<A2,A1,A0>, P<R,G,B>} -- 11 bytes."""
    return bytes([OPEN, CH['W']]) + be24(addr) + bytes([COMMA, CH['P'], r, g, b, CLOSE])


def msg_pixel_read(row, col):
    """{R<row>, C<col>, P<0,0,0>} -- 16 bytes."""
    return (bytes([OPEN, CH['R']]) + be24(row) +
            bytes([COMMA, CH['C']]) + be24(col) +
            bytes([COMMA, CH['P'], 0, 0, 0, CLOSE]))


def msg_reg_read(addr):
    """{R<A2,A1,A0>} -- 6 bytes."""
    return bytes([OPEN, CH['R']]) + be24(addr) + bytes([CLOSE])


def msg_reg_write(addr, data):
    """{W<A>, V<0,DH1,DH0>, V<0,DL1,DL0>} -- 16 bytes."""
    hi, lo = (data >> 16) & 0xFFFF, data & 0xFFFF
    return (bytes([OPEN, CH['W']]) + be24(addr) +
            bytes([COMMA, CH['V'], 0, (hi >> 8) & 0xFF, hi & 0xFF]) +
            bytes([COMMA, CH['V'], 0, (lo >> 8) & 0xFF, lo & 0xFF]) +
            bytes([CLOSE]))


def msg_burst_header(base, height, width):
    """{I<0,0,0>, H<h>, W<w>} -- 16 bytes."""
    return (bytes([OPEN, CH['I']]) + be24(base) +
            bytes([COMMA, CH['H']]) + be24(height) +
            bytes([COMMA, CH['W']]) + be24(width) +
            bytes([CLOSE]))


def msg_burst_data(px):
    """
    {<R0,G0,B0,R1>, <G1,B1,R2,G2>, <B2,R3,G3,B3>} -- 16 bytes.

    Four pixels, twelve payload bytes, with the delimiters at positions 5 and
    10 falling MID-PIXEL. That is why pixel 1 spans bytes 4, 6 and 7.
    """
    assert len(px) == 4
    flat = []
    for (r, g, b) in px:
        flat += [r, g, b]
    return bytes([OPEN] + flat[0:4] + [COMMA] + flat[4:8] + [COMMA] + flat[8:12] + [CLOSE])


def parse_pixel_reply(buf):
    """{R<row>, C<col>, P<R,G,B>} -> (row, col, (r,g,b)) or None if malformed."""
    if len(buf) != PIX_REPLY_LEN:
        return None
    if buf[0] != OPEN or buf[1] != CH['R'] or buf[5] != COMMA:
        return None
    if buf[6] != CH['C'] or buf[10] != COMMA or buf[11] != CH['P']:
        return None
    if buf[15] != CLOSE:
        return None
    row = (buf[2] << 16) | (buf[3] << 8) | buf[4]
    col = (buf[7] << 16) | (buf[8] << 8) | buf[9]
    return row, col, (buf[12], buf[13], buf[14])


# =======================================================================
# Link
# =======================================================================
class Board:
    def __init__(self, port, baud, timeout, verbose=False):
        # EVEN parity and RTS/CTS are not optional -- they are what the
        # design implements. Without flow control the PC overruns the FPGA
        # during a burst and it looks like data corruption.
        self.ser = serial.Serial(
            port=port, baudrate=baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_EVEN,
            stopbits=serial.STOPBITS_ONE,
            timeout=timeout,
            rtscts=True,
        )
        self.sent = 0
        self.recv = 0
        self.verbose = verbose
        time.sleep(0.05)
        self.ser.reset_input_buffer()

    def send(self, data):
        self.ser.write(data)
        self.ser.flush()
        self.sent += len(data)
        if self.verbose:
            print("    TX", data.hex())

    def read_exact(self, n):
        """
        Read exactly n bytes or return None. NEVER returns a partial or
        invented result -- a timeout is a failure the caller must handle.
        """
        buf = bytearray()
        deadline = time.time() + self.ser.timeout * 4
        while len(buf) < n and time.time() < deadline:
            chunk = self.ser.read(n - len(buf))
            if chunk:
                buf += chunk
        self.recv += len(buf)
        if self.verbose and buf:
            print("    RX", bytes(buf).hex())
        return bytes(buf) if len(buf) == n else None

    def close(self):
        self.ser.close()


# =======================================================================
# Tests
# =======================================================================
class Results:
    def __init__(self):
        self.passed = 0
        self.failed = 0

    def check(self, name, ok, detail=""):
        if ok:
            self.passed += 1
            print("  PASS  %-42s %s" % (name, detail))
        else:
            self.failed += 1
            print("  FAIL  %-42s %s" % (name, detail))
        return ok


def read_pixel(board, row, col, res, label):
    """Read one pixel and validate the reply thoroughly."""
    board.send(msg_pixel_read(row, col))
    raw = board.read_exact(PIX_REPLY_LEN)

    if raw is None:
        res.check(label, False, "NO REPLY -- board did not answer")
        return None

    parsed = parse_pixel_reply(raw)
    if parsed is None:
        res.check(label, False, "malformed reply: %s" % raw.hex())
        return None

    got_row, got_col, pix = parsed

    # The reply carries its own address. Checking it catches a board that
    # answers with a constant, and replies arriving out of order.
    if got_row != row or got_col != col:
        res.check(label, False,
                  "address echo wrong: asked (%d,%d) got (%d,%d)"
                  % (row, col, got_row, got_col))
        return None

    return pix


def load_init(path):
    """
    Read one channel's .mem file into a list of 8-bit pixel values.

    Each line is one 32-bit word holding FOUR pixels, LANE 0 IS THE MOST
    SIGNIFICANT BYTE -- the convention pixel_word_packer and rgb_sram use.
    So word N holds linear pixels 4N..4N+3, most significant byte first.
    """
    pixels = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("//"):
                continue
            word = int(line, 16)
            for lane in range(4):
                pixels.append((word >> (8 * (3 - lane))) & 0xFF)
    return pixels


def test_read_before_write(board, res, width, init):
    """
    THE PROOF THAT THE BOARD IS ANSWERING.

    These pixels have never been written by this script. The expected values
    come from YOUR OWN .mem files -- the ones the bitstream was built with --
    so the only way to produce them is to read the board's memory. A script
    answering from its own state has nothing to answer with.

    Addresses are chosen where the three channels DIFFER, so a channel swap
    or a stuck value is visible. If the image is uniform there, the test says
    so rather than passing on a coincidence.
    """
    print("\n[1] read before write -- values can only come from the board")

    if init is None:
        print("      SKIPPED: pass --init-red/--init-green/--init-blue to run")
        print("      this is the test that proves the board answered at all")
        return

    red, green, blue = init

    # Prefer addresses whose three channels are not all identical -- those
    # catch more. Fall back to fixed positions if the image is uniform.
    candidates = []
    for lin in range(min(len(red), width * width)):
        if not (red[lin] == green[lin] == blue[lin]):
            candidates.append(lin)
        if len(candidates) >= 4:
            break
    if not candidates:
        candidates = [0, 5, width + 2, 2 * width + 7]
        print("      note: init data is uniform across channels at these")
        print("      addresses, so a channel swap would not be visible here")

    for lin in candidates:
        row, col = lin // width, lin % width
        if lin >= len(red):
            continue
        exp = (red[lin], green[lin], blue[lin])
        pix = read_pixel(board, row, col, res,
                         "read (%d,%d) unwritten" % (row, col))
        if pix is not None:
            res.check("  matches your .mem files", pix == exp,
                      "got %02x%02x%02x expected %02x%02x%02x" % (pix + exp))


def test_write_readback(board, res, width):
    """Write a pixel, read it back. The value must round-trip."""
    print("\n[2] write then read back")

    cases = [((3, 1), (0xC0, 0xFF, 0xEE)),
             ((5, 6), (0x12, 0x34, 0x56))]

    for (row, col), val in cases:
        addr = row * width + col
        board.send(msg_pixel_write(addr, *val))
        time.sleep(0.01)
        pix = read_pixel(board, row, col, res, "read back (%d,%d)" % (row, col))
        if pix is not None:
            res.check("  round trip", pix == val,
                      "got %02x%02x%02x wrote %02x%02x%02x" % (pix + val))


def test_negative_control(board, res, width, init):
    """
    NEGATIVE CONTROL.

    Read an address that was NEVER written. It must still hold its init value.
    A script echoing its own writes would report the written pattern here, and
    a board that ignores addresses would return the same pixel for everything.
    """
    print("\n[3] negative control -- an address never written")

    row, col = 6, 3
    lin = row * width + col

    pix = read_pixel(board, row, col, res, "read (%d,%d) never written" % (row, col))
    if pix is None:
        return

    # This half works with or without the init files: whatever is at this
    # address, it must NOT be something this script wrote elsewhere.
    res.check("  not the pattern written in test 2",
              pix != (0xC0, 0xFF, 0xEE) and pix != (0x12, 0x34, 0x56),
              "got %02x%02x%02x" % pix)

    if init is not None:
        red, green, blue = init
        if lin < len(red):
            exp = (red[lin], green[lin], blue[lin])
            res.check("  still the init value", pix == exp,
                      "got %02x%02x%02x expected %02x%02x%02x" % (pix + exp))


def test_burst(board, res, width):
    """
    Burst write, then read the pixels back individually.

    Reading back one at a time is deliberate: it uses a DIFFERENT path from
    the one that wrote them, so a burst that silently did nothing cannot hide.
    """
    print("\n[4] burst write, read back one pixel at a time")

    base_row, base_col = 2, 0
    h, w = 2, 4
    base = base_row * width + base_col

    # A distinctive pattern, derived from position so a mix-up is visible.
    pixels = []
    for r in range(h):
        for c in range(w):
            i = r * w + c
            pixels.append((0xA0 + i, 0xB0 + i, 0xC0 + i))

    board.send(msg_burst_header(base, h, w))
    time.sleep(0.01)
    for i in range(0, len(pixels), 4):
        board.send(msg_burst_data(pixels[i:i + 4]))
        time.sleep(0.01)
    time.sleep(0.05)

    ok = True
    for r in range(h):
        for c in range(w):
            i = r * w + c
            pix = read_pixel(board, base_row + r, base_col + c, res,
                             "burst pixel (%d,%d)" % (base_row + r, base_col + c))
            if pix is None or pix != pixels[i]:
                ok = False
    res.check("burst contents correct", ok, "")


def test_reg_then_pixel(board, res, width):
    """
    DISCRIMINATOR. Send a register read, then a pixel read.

    The pixel read is known to work on its own. So:

      pixel read still works  ->  the register command vanished somewhere
                                  BEFORE tx_reply_ctrl. Nothing is stuck.
      pixel read now fails    ->  tx_reply_ctrl latched the register reply
                                  and never sent it: `pending` is stuck high
                                  and it now blocks every later reply.

    That single bit tells you which half of the path to look at, and needs
    no rebuild to find out.
    """
    print("\n[6] register read, then a pixel read -- is anything stuck?")

    board.ser.reset_input_buffer()
    board.send(msg_reg_read(0x08))
    got_reg = board.read_exact(REG_REPLY_LEN)
    time.sleep(0.05)

    pix = read_pixel(board, 0, 0, res, "pixel read AFTER a register read")

    if got_reg is None and pix is not None:
        print("      -> register command vanished before the reply path.")
        print("         Nothing is stuck: pixel replies still work.")
    elif got_reg is None and pix is None:
        print("      -> the reply path is now JAMMED. tx_reply_ctrl latched")
        print("         the register reply and never sent it.")
    elif got_reg is not None:
        print("      -> register reply arrived this time: %s" % got_reg.hex())


def test_register_write_effect(board, res):
    """
    DISCRIMINATOR: is it REGISTER commands that are broken, or 6-BYTE FRAMES?

    A register read is the ONLY six-byte message in this whole suite --
    everything that passes is 11 or 16 bytes. So a failing register read has
    two possible causes and they need separating.

    This sends a register WRITE, which is SIXTEEN bytes, to IMG_CTRL bit 0:
    start_img_read. If registers work, the board begins streaming the whole
    image and bytes pour in.

      bytes arrive     -> register commands are fine. The fault is specific
                          to the six-byte frame, i.e. rx_mac terminating on
                          '}' at byte 5, or the parser's one-group branch.
      nothing arrives  -> register commands are broken regardless of frame
                          length, and the six-byte frame is a red herring.
    """
    print("\n[7] register WRITE (16 bytes) to IMG_CTRL.start_img_read")
    print("      if registers work at all, the board starts streaming")

    board.ser.reset_input_buffer()
    board.send(msg_reg_write(0x08, 0x0000_0001))

    # Collect for a moment. A full image is far more than this; any
    # substantial burst of bytes is the answer.
    time.sleep(0.30)
    got = board.ser.read(4096)
    board.recv += len(got)

    if len(got) > 32:
        res.check("image stream started", True, "%d bytes in 0.3 s" % len(got))
        print("      -> REGISTER COMMANDS WORK. The fault is specific to the")
        print("         SIX-BYTE frame: rx_mac ending on '}' at byte 5, or")
        print("         rx_msg_parser's one-group branch.")
    else:
        res.check("image stream started", False,
                  "%d bytes -- nothing happened" % len(got))
        print("      -> register commands do not work at ANY frame length.")
        print("         The six-byte frame is not the issue.")

    # Stop it streaming so later runs start clean.
    board.send(msg_reg_write(0x08, 0x0000_0000))
    time.sleep(0.30)
    board.ser.reset_input_buffer()


def test_register(board, res, addrs=(0x08, 0x00, 0x04, 0x10)):
    """
    Register read. A different reply length -- 6 bytes, not 16 -- and a
    different path out of the design, so it is worth several addresses.

    ANYTHING RECEIVED IS INFORMATIVE, even a wrong length: it tells you the
    design answered and only the format is off. Nothing at all points at the
    reply path rather than the decode.
    """
    print("\n[5] register read")

    for a in addrs:
        board.ser.reset_input_buffer()
        board.send(msg_reg_read(a))
        raw = board.read_exact(REG_REPLY_LEN)

        if raw is None:
            # Take whatever did arrive, so a short or long reply is visible
            # rather than being reported as silence.
            partial = board.ser.read(32)
            board.recv += len(partial)
            if partial:
                res.check("reg 0x%02x reply" % a, False,
                          "wrong length: %d bytes, %s"
                          % (len(partial), partial.hex()))
            else:
                res.check("reg 0x%02x reply" % a, False, "NO REPLY at all")
            continue

        ok = raw[0] == OPEN and raw[5] == CLOSE
        value = (raw[1] << 24) | (raw[2] << 16) | (raw[3] << 8) | raw[4]
        res.check("reg 0x%02x reply" % a, ok,
                  "%s  value=%08x" % (raw.hex(), value))


# =======================================================================
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port")
    ap.add_argument("--baud", type=int, default=8125000,
                    help="130 MHz / 16 = 8125000")
    ap.add_argument("--timeout", type=float, default=0.5)
    ap.add_argument("--width", type=int, default=8,
                    help="image width; 8 for a SIMULATION build, else 256")
    ap.add_argument("--skip-burst", action="store_true")
    ap.add_argument("--reg-only", action="store_true",
                    help="register read ONLY, first thing after connecting. "
                         "Separates a broken reply path from an interaction "
                         "with the reads that ran before it.")
    ap.add_argument("--verbose", action="store_true",
                    help="print every byte sent and received")
    ap.add_argument("--list", action="store_true", help="list serial ports")
    ap.add_argument("--init-red",   help="red_hex.mem, as built into the bitstream")
    ap.add_argument("--init-green", help="green_hex.mem")
    ap.add_argument("--init-blue",  help="blue_hex.mem")
    a = ap.parse_args()

    if a.list:
        for p in serial.tools.list_ports.comports():
            print(p.device, "-", p.description)
        return 0

    if not a.port:
        return ap.error("--port is required (use --list to find it)")

    print("port %s  baud %d  parity EVEN  RTS/CTS ON  image %dx%d"
          % (a.port, a.baud, a.width, a.width))

    init = None
    if a.init_red and a.init_green and a.init_blue:
        init = (load_init(a.init_red), load_init(a.init_green), load_init(a.init_blue))
        print("init files loaded: %d pixels per channel" % len(init[0]))
    else:
        print("no init files given -- test 1 will be skipped")

    board = Board(a.port, a.baud, a.timeout, a.verbose)
    res = Results()

    try:
        if a.reg_only:
            test_register(board, res)
            test_reg_then_pixel(board, res, a.width)
            test_register_write_effect(board, res)
        else:
            test_read_before_write(board, res, a.width, init)
            test_write_readback(board, res, a.width)
            test_negative_control(board, res, a.width, init)
            if not a.skip_burst:
                test_burst(board, res, a.width)
            test_register(board, res)
            test_reg_then_pixel(board, res, a.width)
    finally:
        board.close()

    print("\n" + "=" * 60)
    print("  bytes sent     : %d" % board.sent)
    print("  bytes received : %d" % board.recv)
    print("  passed         : %d" % res.passed)
    print("  failed         : %d" % res.failed)

    # The tests all depend on received bytes, so this cannot be reached with
    # passes and no traffic -- but state it explicitly anyway.
    if board.recv == 0:
        print("\n  NOTHING WAS RECEIVED. The board did not answer at all.")
        print("  Any 'pass' above would be a bug in this script.")
    print("  RESULT         : %s" % ("PASS" if res.failed == 0 else "FAIL"))
    print("=" * 60)

    return 0 if res.failed == 0 and board.recv > 0 else 1


if __name__ == "__main__":
    sys.exit(main())
