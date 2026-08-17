#!/usr/bin/env python3
"""
image_tool.py -- read the image OUT OF THE BOARD and render it to PNG.

THE RULE THIS SCRIPT FOLLOWS
----------------------------
Every pixel in every PNG came back over the wire from the FPGA. This script
sends commands and draws what arrives. It NEVER composites, fills, blends or
modifies an image locally, and it keeps no model of what the memory ought to
contain.

Concretely, that means:

  * A modification is a MESSAGE. To change a pixel the script sends a Single
    Pixel Write and then re-reads the image. It does not touch a local array.

  * Any pixel that never arrives stays MAGENTA (255, 0, 255) in the output
    and is counted. There is no interpolation and no default. A hole in the
    picture is a hole in the data.

  * The PNG writer is stdlib zlib. Nothing outside this file decides what a
    pixel is.

WHAT BURST READ COSTS, AND HOW IT IS PAID BACK
----------------------------------------------
The pixel-by-pixel path places every pixel at THE ADDRESS THE BOARD SENT
BACK: a Single Pixel Read reply carries its own row and column, so a board
answering with a constant, or replies arriving out of order, show up as a
wrong image rather than being silently corrected.

A BURST READ REPLY CARRIES NO ADDRESS. Sixteen bytes hold four pixels and
nothing else. So in burst and stream mode a pixel's position comes from its
ORDINAL POSITION IN THE STREAM, counted from the base address in the request
this script sent. That is a real, unavoidable weakening of the rule above,
and it is worth being precise about what it lets through:

  * A slipped byte desynchronises everything after it. Payload bytes are
    unrestricted 8-bit values, so 0x7B/0x7D/0x2C occur inside pixel data and
    the delimiters CANNOT be used to find frame boundaries -- only to check
    them. Framing is by counting 16 from a known start, and the only known
    start is the first byte after the request.

  * Losing a whole 16-byte frame keeps the delimiters aligned and shifts
    every later pixel by four. Nothing in the reply stream can detect this.

Two things pay that back:

  * Every frame's four delimiters are checked. On a framing error the script
    does NOT try to guess how many pixels went missing -- it stops consuming,
    and re-issues a fresh burst read for the rows it has not got, which
    re-anchors the ordinal against a base address it chose itself.

  * --verify N re-reads N random pixels with the SINGLE PIXEL READ, whose
    reply does carry row and column, and compares them against the burst
    image. That is the address echo, applied by sampling. It catches an
    off-by-one base, a transposed walk, a channel swap and a lost frame. It
    is on by default; --verify 0 turns it off.

USAGE
-----
    python image_tool.py --port COM5 snapshot before.png

    python image_tool.py --port COM5 --mode pixel snapshot slow.png
    python image_tool.py --port COM5 --mode stream snapshot whole.png

    python image_tool.py --port COM5 pixel 100 50 FF0000
    python image_tool.py --port COM5 rect 60 60 40 30 00FF00
    python image_tool.py --port COM5 load photo.png
    python image_tool.py --port COM5 load logo.png --at 64 64 --fit scale

    python image_tool.py --port COM5 demo out         # writes out_*.png

A 256x256 snapshot is 65536 pixels. Pixel-by-pixel that is 65536 round trips
and 2097152 bytes on the wire; burst read is one request and 262160 bytes,
about 8x less traffic and no per-pixel stall. --region reads a window.
"""

import argparse
import random
import struct
import sys
import time
import zlib

try:
    import serial
except ImportError:
    sys.exit("pyserial is required:  pip install pyserial")


OPEN, CLOSE, COMMA = ord('{'), ord('}'), ord(',')
CH = {c: ord(c) for c in "WRVCPIH"}
PIX_REPLY_LEN = 16
BURST_FRAME_LEN = 16             # 16 bytes on the wire = 4 pixels
PIX_PER_FRAME = 4
REG_REPLY_LEN = 6
IMG_CTRL = 0x08                  # bit 0 = start_img_read
MISSING = (255, 0, 255)          # magenta: nothing came back for this pixel
WIRE_BITS_PER_BYTE = 11          # start + 8 data + even parity + stop


def be24(v):
    v &= 0xFFFFFF
    return bytes([(v >> 16) & 0xFF, (v >> 8) & 0xFF, v & 0xFF])


def msg_pixel_read(row, col):
    """{R<row>,C<col>,P<0,0,0>} -- 16 bytes."""
    return (bytes([OPEN, CH['R']]) + be24(row) +
            bytes([COMMA, CH['C']]) + be24(col) +
            bytes([COMMA, CH['P'], 0, 0, 0, CLOSE]))


def msg_pixel_write(addr, r, g, b):
    """{W<addr>,P<R,G,B>} -- 11 bytes."""
    return bytes([OPEN, CH['W']]) + be24(addr) + bytes([COMMA, CH['P'], r, g, b, CLOSE])


def msg_burst_header(base, height, width):
    """{I<base>,H<h>,W<w>} -- 16 bytes. Header of an Image Burst WRITE."""
    return (bytes([OPEN, CH['I']]) + be24(base) +
            bytes([COMMA, CH['H']]) + be24(height) +
            bytes([COMMA, CH['W']]) + be24(width) + bytes([CLOSE]))


def msg_burst_read(base, height, width):
    """
    {R<base>,H<h>,W<w>} -- 16 bytes. Image Burst READ request.

    Same shape as the burst write header with the tag changed to R, and the
    same length as a Single Pixel Read; the two are told apart by byte 6,
    which is 'H' here and 'C' there.
    """
    return (bytes([OPEN, CH['R']]) + be24(base) +
            bytes([COMMA, CH['H']]) + be24(height) +
            bytes([COMMA, CH['W']]) + be24(width) + bytes([CLOSE]))


def msg_reg_write(addr, data):
    """{W<addr>,V<0,DH1,DH0>,V<0,DL1,DL0>} -- 16 bytes."""
    hi, lo = (data >> 16) & 0xFFFF, data & 0xFFFF
    return (bytes([OPEN, CH['W']]) + be24(addr) +
            bytes([COMMA, CH['V'], 0, (hi >> 8) & 0xFF, hi & 0xFF]) +
            bytes([COMMA, CH['V'], 0, (lo >> 8) & 0xFF, lo & 0xFF]) +
            bytes([CLOSE]))


def msg_burst_data(px):
    """Four pixels. The delimiters at bytes 5 and 10 fall MID-PIXEL."""
    flat = []
    for (r, g, b) in px:
        flat += [r, g, b]
    while len(flat) < 12:
        flat.append(0)
    return bytes([OPEN] + flat[0:4] + [COMMA] + flat[4:8] + [COMMA] + flat[8:12] + [CLOSE])


def parse_pixel_reply(buf):
    if len(buf) != PIX_REPLY_LEN:
        return None
    if (buf[0] != OPEN or buf[1] != CH['R'] or buf[5] != COMMA or
            buf[6] != CH['C'] or buf[10] != COMMA or buf[11] != CH['P'] or
            buf[15] != CLOSE):
        return None
    row = (buf[2] << 16) | (buf[3] << 8) | buf[4]
    col = (buf[7] << 16) | (buf[8] << 8) | buf[9]
    return row, col, (buf[12], buf[13], buf[14])


# =======================================================================
# Burst reply framing
#
# {<R0,G0,B0,R1>,<G1,B1,R2,G2>,<B2,R3,G3,B3>} -- 16 bytes, 4 pixels.
# Payload occupies bytes 1-4, 6-9 and 11-14; the delimiters at 0, 5, 10 and
# 15 are the only fixed bytes and they are a CHECK, not a search key, since
# a pixel value of 0x2C is a comma and a pixel value of 0x7D is a brace.
# =======================================================================
def frame_ok(buf, p):
    """Do the four delimiters of the frame at offset p look right?"""
    return (p + BURST_FRAME_LEN <= len(buf) and
            buf[p] == OPEN and buf[p + 5] == COMMA and
            buf[p + 10] == COMMA and buf[p + 15] == CLOSE)


def decode_frame(buf, p):
    """The four pixels of the frame at offset p. Assumes frame_ok already."""
    f = bytes(buf[p + 1:p + 5]) + bytes(buf[p + 6:p + 10]) + bytes(buf[p + 11:p + 15])
    return [(f[0], f[1], f[2]), (f[3], f[4], f[5]),
            (f[6], f[7], f[8]), (f[9], f[10], f[11])]


def decode_stream(buf, npixels):
    """
    Decode consecutive frames from the start of buf.

    Returns (pixels, bytes_consumed, error) where error is None, "framing"
    (a frame's delimiters were wrong -- the stream is desynchronised from
    that byte on and nothing after it can be trusted) or "short" (the data
    ran out). Decoding NEVER skips ahead to resynchronise: a byte slip of
    unknown size means an unknown number of lost pixels, and guessing at it
    is exactly the silent correction this tool exists to avoid.
    """
    px = []
    pos = 0
    while len(px) < npixels and pos + BURST_FRAME_LEN <= len(buf):
        if not frame_ok(buf, pos):
            return px, pos, "framing"
        px += decode_frame(buf, pos)
        pos += BURST_FRAME_LEN
    if len(px) < npixels:
        return px, pos, "short"
    return px[:npixels], pos, None


# =======================================================================
def write_png(path, pixels, width, height):
    """
    Minimal PNG, stdlib only. `pixels` is a list of (r,g,b) in row-major
    order and is written through unchanged -- no scaling, no filtering, no
    colour management. What the board sent is what lands in the file.
    """
    raw = bytearray()
    for y in range(height):
        raw.append(0)                       # filter type 0, none
        for x in range(width):
            raw += bytes(pixels[y * width + x])

    def chunk(tag, data):
        c = struct.pack(">I", len(data)) + tag + data
        return c + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)

    png = (b"\x89PNG\r\n\x1a\n" +
           chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0)) +
           chunk(b"IDAT", zlib.compress(bytes(raw), 6)) +
           chunk(b"IEND", b""))
    with open(path, "wb") as f:
        f.write(png)


# =======================================================================
# PNG reading -- stdlib zlib only, same as the writer.
#
# The decoder is deliberately strict. It verifies every chunk CRC and
# refuses anything it cannot decode exactly rather than guessing: a file
# this tool half-understands would put pixels on the board that are not
# the pixels in the file, and the whole point of the read path is that
# what comes back can be trusted to mean something.
# =======================================================================
def _paeth(a, b, c):
    p = a + b - c
    pa, pb, pc = abs(p - a), abs(p - b), abs(p - c)
    if pa <= pb and pa <= pc:
        return a
    return b if pb <= pc else c


def _unfilter(raw, height, stride, bpp):
    """Undo the per-scanline filters. Returns one bytearray of all rows."""
    out = bytearray()
    prev = bytearray(stride)
    pos = 0
    for _ in range(height):
        ft = raw[pos]
        pos += 1
        line = bytearray(raw[pos:pos + stride])
        pos += stride
        if ft == 0:
            pass
        elif ft == 1:
            for i in range(bpp, stride):
                line[i] = (line[i] + line[i - bpp]) & 0xFF
        elif ft == 2:
            for i in range(stride):
                line[i] = (line[i] + prev[i]) & 0xFF
        elif ft == 3:
            for i in range(stride):
                left = line[i - bpp] if i >= bpp else 0
                line[i] = (line[i] + ((left + prev[i]) >> 1)) & 0xFF
        elif ft == 4:
            for i in range(stride):
                left = line[i - bpp] if i >= bpp else 0
                ul = prev[i - bpp] if i >= bpp else 0
                line[i] = (line[i] + _paeth(left, prev[i], ul)) & 0xFF
        else:
            raise ValueError("unknown PNG filter type %d" % ft)
        out += line
        prev = line
    return out


def _samples(line, depth, count):
    """Extract `count` samples of `depth` bits from one unfiltered row."""
    if depth == 8:
        return list(line[:count])
    if depth == 16:
        return [line[2 * i] for i in range(count)]        # high byte only
    out = []
    per = 8 // depth
    mask = (1 << depth) - 1
    for i in range(count):
        b = line[i // per]
        shift = 8 - depth * (i % per + 1)
        out.append((b >> shift) & mask)
    return out


def read_png(path):
    """
    Decode a PNG to (pixels, width, height) with pixels as (r,g,b) tuples.

    Supports colour types 0/2/3/4/6 at bit depths 1/2/4/8/16, which covers
    everything a normal exporter emits. Interlaced files are refused rather
    than mangled. Alpha is DISCARDED, not composited: the board stores three
    channels and blending against an assumed background would be this script
    inventing colour that is not in the file.
    """
    data = open(path, "rb").read()
    if data[:8] != b"\x89PNG\r\n\x1a\n":
        raise ValueError("%s is not a PNG" % path)

    pos = 8
    idat = bytearray()
    plte = None
    hdr = None
    while pos + 8 <= len(data):
        ln = struct.unpack(">I", data[pos:pos + 4])[0]
        tag = data[pos + 4:pos + 8]
        if pos + 12 + ln > len(data):
            raise ValueError("%s: truncated %s chunk -- the file is "
                             "incomplete" % (path, tag.decode("ascii", "replace")))
        body = data[pos + 8:pos + 8 + ln]
        crc = struct.unpack(">I", data[pos + 8 + ln:pos + 12 + ln])[0]
        if zlib.crc32(tag + body) & 0xFFFFFFFF != crc:
            raise ValueError("%s: CRC failure in %s chunk"
                             % (path, tag.decode("ascii", "replace")))
        if tag == b"IHDR":
            hdr = struct.unpack(">IIBBBBB", body)
        elif tag == b"PLTE":
            plte = body
        elif tag == b"IDAT":
            idat += body
        elif tag == b"IEND":
            break
        pos += 12 + ln

    if hdr is None:
        raise ValueError("%s: no IHDR" % path)
    width, height, depth, ctype, comp, filt, interlace = hdr

    if interlace:
        raise ValueError("%s is interlaced (Adam7); save it non-interlaced"
                         % path)
    if comp != 0 or filt != 0:
        raise ValueError("%s uses an unknown compression or filter method"
                         % path)

    nchan = {0: 1, 2: 3, 3: 1, 4: 2, 6: 4}.get(ctype)
    if nchan is None:
        raise ValueError("%s: unsupported colour type %d" % (path, ctype))
    if ctype == 3 and plte is None:
        raise ValueError("%s: palette image with no PLTE chunk" % path)

    bits = nchan * depth
    stride = (width * bits + 7) // 8
    bpp = max(1, bits // 8)

    raw = zlib.decompress(bytes(idat))
    if len(raw) < height * (stride + 1):
        raise ValueError("%s: truncated image data" % path)
    flat = _unfilter(raw, height, stride, bpp)

    maxval = (1 << depth) - 1
    pixels = []
    for y in range(height):
        line = flat[y * stride:(y + 1) * stride]
        s = _samples(line, depth, width * nchan)
        if ctype == 2 or ctype == 6:
            for x in range(width):
                i = x * nchan
                pixels.append((s[i], s[i + 1], s[i + 2]))
        elif ctype == 0 or ctype == 4:
            for x in range(width):
                g = s[x * nchan]
                if depth < 8:
                    g = g * 255 // maxval
                pixels.append((g, g, g))
        else:                                    # palette
            for x in range(width):
                i = s[x] * 3
                if i + 2 >= len(plte):
                    raise ValueError("%s: palette index out of range" % path)
                pixels.append((plte[i], plte[i + 1], plte[i + 2]))

    return pixels, width, height


def fit_image(pixels, sw, sh, dw, dh, mode):
    """
    Make a sw x sh image into a dw x dh one. Returns (pixels, note).

    Every mode here CHANGES THE PIXELS, which is why none of them is the
    default and each one says what it did. This is the write path: the
    values have to come from somewhere. The read path still never invents
    anything.
    """
    if (sw, sh) == (dw, dh):
        return pixels, "exact size"

    if mode == "scale":
        out = []
        for y in range(dh):
            sy = y * sh // dh
            row = sy * sw
            for x in range(dw):
                out.append(pixels[row + x * sw // dw])
        return out, "nearest-neighbour scaled from %dx%d" % (sw, sh)

    if mode == "crop":
        out = []
        for y in range(dh):
            for x in range(dw):
                if y < sh and x < sw:
                    out.append(pixels[y * sw + x])
                else:
                    out.append((0, 0, 0))
        return out, ("cropped from %dx%d (black where the source ran out)"
                     % (sw, sh))

    raise ValueError(
        "image is %dx%d but the target region is %dx%d. Pass --fit scale "
        "or --fit crop to say what should happen, or resize the file."
        % (sw, sh, dw, dh))


# =======================================================================
def write_image_burst(board, pixels, width, x0, y0, w, h, chunk=4096,
                      pace=0.0):
    """
    Send a w x h block of pixels with one Image Burst Write.

    The script chooses the VALUES -- they came from the file -- but it does
    not place them: one header carries the base, height and width, and the
    board walks the rectangle itself. Nothing here tracks what the memory
    ought to contain afterwards; do_load re-reads it.

    There is no artificial pacing by default. RTS/CTS is what stops the PC
    overrunning the receive path, and sleeping instead of trusting it would
    just hide a broken link. --pace exists for bringing up a board whose
    flow control is not working yet.
    """
    base = y0 * width + x0
    total = w * h

    board.send(msg_burst_header(base, h, w))
    board.flush()
    time.sleep(0.02)

    blob = bytearray()
    for i in range(0, total, 4):
        group = pixels[i:i + 4]
        blob += msg_burst_data(list(group) + [(0, 0, 0)] * (4 - len(group)))

    t0 = time.time()
    sent = 0
    while sent < len(blob):
        n = min(chunk, len(blob) - sent)
        board.send(bytes(blob[sent:sent + n]))
        board.flush()
        sent += n
        if pace:
            time.sleep(pace)
        el = max(time.time() - t0, 1e-6)
        sys.stdout.write("\r    %6.1f%%  %d/%d bytes  %.0f kB/s"
                         % (100.0 * sent / len(blob), sent, len(blob),
                            sent / el / 1000.0))
        sys.stdout.flush()
    print()
    time.sleep(0.10)
    return len(blob), (total + 3) // 4


# =======================================================================
class Board:
    def __init__(self, port, baud, timeout):
        self.ser = serial.Serial(
            port=port, baudrate=baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_EVEN,
            stopbits=serial.STOPBITS_ONE,
            timeout=timeout,
            rtscts=True,            # the design deasserts CTS when busy
        )
        # A burst arrives far faster than a Python loop wants to be woken
        # up. On Windows the driver buffer is 4 KB by default, which a
        # 256 KB image stream overruns; ask for more where we can.
        if hasattr(self.ser, "set_buffer_size"):
            try:
                self.ser.set_buffer_size(rx_size=1 << 20, tx_size=1 << 16)
            except Exception:
                pass
        time.sleep(0.05)
        self.ser.reset_input_buffer()
        self.sent = 0
        self.recv = 0

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
        """Exactly n bytes or None. A timeout is a failure, not a default."""
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
        self.ser.close()


def collect(board, nbytes, first_timeout=3.0, idle_timeout=0.5, label=""):
    """
    Drain nbytes from the port, or as many as arrive before the stream goes
    idle. The loop does no per-frame work: at 8.75 Mbaud the wire delivers
    ~739 kB/s and anything clever in here is a chance to fall behind and
    lose the byte that desynchronises the whole picture. Parsing happens
    afterwards, on the complete buffer.
    """
    buf = bytearray()
    t0 = time.time()
    deadline = t0 + first_timeout
    next_report = t0 + 0.2
    while len(buf) < nbytes and time.time() < deadline:
        chunk = board.read(min(1 << 16, nbytes - len(buf)))
        if chunk:
            buf += chunk
            deadline = time.time() + idle_timeout
            now = time.time()
            if now >= next_report:
                next_report = now + 0.2
                el = max(now - t0, 1e-6)
                sys.stdout.write("\r    %s%6.1f%%  %d/%d bytes  %.0f kB/s"
                                 % (label, 100.0 * len(buf) / nbytes,
                                    len(buf), nbytes, len(buf) / el / 1000.0))
                sys.stdout.flush()
    el = max(time.time() - t0, 1e-6)
    sys.stdout.write("\r    %s%6.1f%%  %d/%d bytes  %.0f kB/s\n"
                     % (label, 100.0 * len(buf) / nbytes, len(buf), nbytes,
                        len(buf) / el / 1000.0))
    sys.stdout.flush()
    return buf


# =======================================================================
def read_image_pixels(board, width, height, x0=0, y0=0, w=None, h=None, batch=64):
    """
    Read a region pixel by pixel and return (pixels, missing_count).

    Slow -- one request and one reply per pixel -- but every pixel is placed
    BY THE ROW AND COLUMN IN ITS OWN REPLY, so the picture is assembled from
    what the board said and not from the order the script asked. This is the
    reference path: when a burst image looks wrong, read a window this way
    and compare. Anything that never comes back stays magenta.
    """
    w = w or width
    h = h or height
    pixels = [MISSING] * (width * height)
    got = 0
    asked = 0

    coords = [(y0 + r, x0 + c) for r in range(h) for c in range(w)]
    t0 = time.time()

    for i in range(0, len(coords), batch):
        block = coords[i:i + batch]

        for (row, col) in block:
            board.send(msg_pixel_read(row, col))
        board.flush()
        asked += len(block)

        buf = bytearray()
        want = len(block) * PIX_REPLY_LEN
        deadline = time.time() + 2.0
        while len(buf) < want and time.time() < deadline:
            chunk = board.read(want - len(buf))
            if chunk:
                buf += chunk
                deadline = time.time() + 2.0

        for j in range(0, len(buf) - PIX_REPLY_LEN + 1, PIX_REPLY_LEN):
            parsed = parse_pixel_reply(bytes(buf[j:j + PIX_REPLY_LEN]))
            if parsed is None:
                continue
            r, c, rgb = parsed
            if 0 <= r < height and 0 <= c < width:
                pixels[r * width + c] = rgb
                got += 1

        done = min(i + batch, len(coords))
        pct = 100.0 * done / len(coords)
        rate = done / max(time.time() - t0, 1e-6)
        sys.stdout.write("\r    %6.1f%%  %d/%d pixels  %.0f px/s"
                         % (pct, done, len(coords), rate))
        sys.stdout.flush()

    print()
    return pixels, asked - got


def read_image_burst(board, width, height, x0=0, y0=0, w=None, h=None,
                     walk="rect", retries=2, timeout=3.0):
    """
    Read a region with Image Burst Read and return (pixels, missing_count).

    One 16-byte request covers the whole rectangle; the board walks it and
    replies with 4 pixels per 16 bytes. Placement is by ordinal position
    from the base address this script asked for -- see the module docstring
    for what that gives up and what --verify buys back.

    On a framing error the remaining rows are RE-REQUESTED rather than
    resynchronised, because a fresh request restores a base address the
    script knows. Rows never received stay magenta.
    """
    w = w or width
    h = h or height
    pixels = [MISSING] * (width * height)
    filled = 0

    row = y0
    rows_left = h
    attempt = 0

    while rows_left > 0 and attempt <= retries:
        attempt += 1
        base = row * width + x0
        npx = rows_left * w
        nframes = (npx + PIX_PER_FRAME - 1) // PIX_PER_FRAME
        nbytes = nframes * BURST_FRAME_LEN

        if attempt > 1:
            print("    re-requesting %d rows from row %d (attempt %d)"
                  % (rows_left, row, attempt))

        board.discard_input()
        board.send(msg_burst_read(base, rows_left, w))
        board.flush()
        buf = collect(board, nbytes, first_timeout=timeout, label="")

        vals, consumed, err = decode_stream(buf, npx)

        for i, rgb in enumerate(vals):
            if walk == "linear":
                lin = base + i
                r, c = lin // width, lin % width
            else:
                r, c = row + i // w, x0 + i % w
            if 0 <= r < height and 0 <= c < width:
                if pixels[r * width + c] == MISSING:
                    filled += 1
                pixels[r * width + c] = rgb

        if err is None:
            rows_left = 0
            break

        if err == "framing":
            # Everything before the bad frame is well framed, so those rows
            # are kept and only the rest is re-requested. The partial row at
            # the boundary is re-read too: a misframed reply says nothing
            # about where within the row the good data stopped.
            complete = len(vals) // w
            print("    FRAMING ERROR at byte %d of the reply: the delimiters"
                  % consumed)
            print("    do not line up, so the stream is desynchronised from")
            print("    there on. Nothing after it is placed.")
            if complete == 0:
                print("    not one complete row survived -- giving up")
                break
            row += complete
            rows_left -= complete
        else:
            # A short reply CANNOT BE LOCALISED. A frame lost mid-stream
            # keeps the delimiters aligned and looks exactly like a
            # truncated tail, so no prefix of this attempt is trustworthy
            # and the whole outstanding region goes again from the base
            # this script chose.
            print("    SHORT REPLY: %d of %d bytes arrived. This cannot be"
                  % (len(buf), nbytes))
            print("    localised -- a frame lost mid-stream and a truncated")
            print("    tail are the same thing here -- so the whole region")
            print("    is re-requested rather than patched.")

    if rows_left > 0:
        print("    RETRIES EXHAUSTED with %d rows outstanding. Pixels that"
              % rows_left)
        print("    did land may also be shifted; --verify is the only thing")
        print("    that will tell you.")

    missing = w * h - filled
    return pixels, max(missing, 0)


def read_image_stream(board, width, height, timeout=5.0):
    """
    Read the WHOLE image by setting IMG_CTRL.start_img_read.

    One 16-byte register write and the board streams the entire frame in the
    burst reply format, base address 0, walked linearly. Fewer host-side
    decisions than a burst request -- and correspondingly less to check: the
    only thing anchoring the ordinal is that the first byte after the
    register write is the first byte of pixel 0.

    Requires register writes to work on the design. If nothing arrives, run
    board_test.py --reg-only: it separates a broken register path from a
    broken six-byte frame.
    """
    total = width * height
    nframes = (total + PIX_PER_FRAME - 1) // PIX_PER_FRAME
    nbytes = nframes * BURST_FRAME_LEN

    pixels = [MISSING] * total

    board.discard_input()
    board.send(msg_reg_write(IMG_CTRL, 0x0000_0001))
    board.flush()
    buf = collect(board, nbytes, first_timeout=timeout)

    # Clear the bit and drain whatever was still in flight, so the next
    # command does not start reading the tail of this image.
    board.send(msg_reg_write(IMG_CTRL, 0x0000_0000))
    board.flush()
    time.sleep(0.30)
    board.discard_input()

    vals, consumed, err = decode_stream(buf, total)
    for i, rgb in enumerate(vals):
        pixels[i] = rgb

    if err == "framing":
        print("    FRAMING ERROR at byte %d -- everything after it is left"
              % consumed)
        print("    magenta rather than placed at a guessed offset.")
    elif err == "short":
        print("    SHORT STREAM: %d of %d bytes arrived" % (len(buf), nbytes))

    return pixels, total - len(vals)


# =======================================================================
def verify_pixels(board, pixels, width, height, x0, y0, w, h, n, seed=1):
    """
    THE ADDRESS ECHO, BY SAMPLING.

    Re-read n random pixels with the Single Pixel Read, whose reply carries
    its own row and column, and compare them to what the burst produced. The
    burst reply cannot police its own placement; this can, at the cost of n
    round trips instead of 65536.

    Returns the number of samples that did not match.
    """
    if n <= 0:
        return 0

    rnd = random.Random(seed)
    coords = set()
    guard = 0
    while len(coords) < min(n, w * h) and guard < n * 20:
        guard += 1
        coords.add((y0 + rnd.randrange(h), x0 + rnd.randrange(w)))
    coords = sorted(coords)

    print("verifying %d random pixels with Single Pixel Reads..." % len(coords))

    ok = 0
    bad = []
    noreply = 0
    echo_bad = 0
    truth = []

    for (r, c) in coords:
        board.discard_input()
        board.send(msg_pixel_read(r, c))
        board.flush()
        raw = board.read_exact(PIX_REPLY_LEN)
        parsed = parse_pixel_reply(raw) if raw else None
        if parsed is None:
            noreply += 1
            continue
        gr, gc, rgb = parsed
        if (gr, gc) != (r, c):
            echo_bad += 1
            continue
        truth.append((r, c, rgb))
        if pixels[r * width + c] == rgb:
            ok += 1
        else:
            bad.append((r, c, rgb, pixels[r * width + c]))

    print("    %d matched, %d differed, %d no reply, %d bad address echo"
          % (ok, len(bad), noreply, echo_bad))

    for (r, c, want, got) in bad[:8]:
        print("      (%3d,%3d) board says %02x%02x%02x, burst image has "
              "%02x%02x%02x" % ((r, c) + want + got))
    if len(bad) > 8:
        print("      ... and %d more" % (len(bad) - 8))

    if bad:
        _diagnose_offset(pixels, width, height, truth)
        print("    The burst image is NOT trustworthy. Read the same window")
        print("    with --mode pixel to see what is actually in memory.")
    elif ok:
        print("    every sampled pixel agrees with an address-echoed read")

    return len(bad)


def _diagnose_offset(pixels, width, height, truth):
    """
    A constant ordinal shift is the signature of a lost reply frame or an
    off-by-one base, and it is worth naming because it looks like noise.
    Slide the verified samples against the assembled image and see whether
    some single displacement explains them all.
    """
    if len(truth) < 3:
        return
    best, best_hits = 0, 0
    for d in range(-64, 65):
        hits = 0
        for (r, c, rgb) in truth:
            idx = r * width + c + d
            if 0 <= idx < width * height and pixels[idx] == rgb:
                hits += 1
        if hits > best_hits:
            best_hits, best = hits, d
    if best != 0 and best_hits >= max(3, int(0.8 * len(truth))):
        print("    DIAGNOSIS: %d of %d samples match at an offset of %+d "
              "pixels." % (best_hits, len(truth), best))
        if best % PIX_PER_FRAME == 0:
            print("      %+d is a whole number of 4-pixel frames: a reply "
                  "frame was" % best)
            print("      lost or duplicated, which keeps the delimiters "
                  "aligned and is")
            print("      invisible to the framing check.")
        elif abs(best) % width == 0:
            print("      %+d is a whole number of image rows: the base "
                  "address or the" % best)
            print("      region's origin is off by %d row(s)."
                  % (best // width))
        else:
            print("      Not a multiple of 4, so this is a base address or "
                  "walk-order")
            print("      problem rather than a lost frame -- re-run with the "
                  "other --walk.")


# =======================================================================
def do_snapshot(board, args, path):
    if args.region:
        x0, y0, w, h = args.region
    else:
        x0, y0, w, h = 0, 0, args.width, args.height

    t0 = time.time()

    if args.mode == "stream":
        if args.region:
            print("note: --mode stream reads the WHOLE image; --region is "
                  "ignored")
            x0, y0, w, h = 0, 0, args.width, args.height
        print("streaming %dx%d via IMG_CTRL.start_img_read..."
              % (args.width, args.height))
        pixels, missing = read_image_stream(board, args.width, args.height,
                                            timeout=args.timeout * 10)
    elif args.mode == "burst":
        print("burst reading %dx%d at (%d,%d) from the board..."
              % (w, h, x0, y0))
        pixels, missing = read_image_burst(board, args.width, args.height,
                                           x0, y0, w, h, walk=args.walk,
                                           retries=args.retries,
                                           timeout=args.timeout * 6)
    else:
        print("reading %dx%d at (%d,%d) one pixel at a time..."
              % (w, h, x0, y0))
        pixels, missing = read_image_pixels(board, args.width, args.height,
                                            x0, y0, w, h, batch=args.batch)

    el = time.time() - t0
    write_png(path, pixels, args.width, args.height)

    print("    wrote %s  (%.2f s, %.0f px/s)" % (path, el, w * h / max(el, 1e-6)))
    if missing:
        print("    %d pixels NEVER CAME BACK and are magenta in the image."
              % missing)
        print("    They are not guessed at -- a hole here is a hole in the data.")
    else:
        print("    every requested pixel came back from the board")

    if args.mode in ("burst", "stream") and args.verify > 0:
        verify_pixels(board, pixels, args.width, args.height,
                      x0, y0, w, h, args.verify)

    return missing


def do_load(board, args):
    """
    Send a PNG to the board, then READ IT BACK and compare.

    The comparison is the point. A burst write produces no replies at all,
    so "the bytes went out" is the only thing the write itself can tell
    you -- and that is exactly the kind of evidence this tool exists not to
    accept. The verification re-reads the region and diffs it against what
    was sent, so a write that silently did nothing cannot look like success.
    """
    try:
        pixels, sw, sh = read_png(args.path)
    except (ValueError, OSError, zlib.error) as e:
        sys.exit("cannot read %s: %s" % (args.path, e))
    print("read %s: %dx%d" % (args.path, sw, sh))

    x0, y0 = args.at
    w, h = args.size if args.size else (args.width - x0, args.height - y0)

    if x0 < 0 or y0 < 0 or x0 + w > args.width or y0 + h > args.height:
        sys.exit("region %dx%d at (%d,%d) does not fit in a %dx%d image"
                 % (w, h, x0, y0, args.width, args.height))

    try:
        pixels, note = fit_image(pixels, sw, sh, w, h, args.fit)
    except ValueError as e:
        sys.exit(str(e))
    print("    %s -- writing %dx%d at row %d col %d" % (note, h, w, y0, x0))

    nbytes, nframes = write_image_burst(board, pixels, args.width,
                                        x0, y0, w, h, pace=args.pace)
    print("    %d pixels in %d burst data messages, %d bytes"
          % (w * h, nframes, nbytes + 16))

    if args.no_verify:
        print("    NOT VERIFIED. Nothing has confirmed the board stored any")
        print("    of this; take a snapshot before believing it.")
        return

    print("reading the same region back...")
    got, missing = read_image_burst(board, args.width, args.height,
                                    x0, y0, w, h, walk=args.walk,
                                    retries=args.retries,
                                    timeout=args.timeout * 6)

    bad = 0
    first = None
    for i in range(w * h):
        r, c = y0 + i // w, x0 + i % w
        if got[r * args.width + c] != pixels[i]:
            bad += 1
            if first is None:
                first = (r, c, pixels[i], got[r * args.width + c])

    if missing:
        print("    %d pixels never came back on the read" % missing)
    if bad == 0 and missing == 0:
        print("    VERIFIED: every pixel read back equals the pixel sent")
    else:
        print("    MISMATCH: %d of %d pixels differ" % (bad, w * h))
        if first:
            print("      first at (%d,%d): sent %02x%02x%02x, board has "
                  "%02x%02x%02x"
                  % (first[0], first[1], first[2][0], first[2][1],
                     first[2][2], first[3][0], first[3][1], first[3][2]))
        print("      This is a write fault OR a read fault -- a burst read")
        print("      places pixels by position, not by address. Re-read with")
        print("      --mode pixel to tell the two apart.")


def do_pixel(board, args):
    row, col, rgb = args.row, args.col, args.colour
    addr = row * args.width + col
    print("sending a Single Pixel Write: (%d,%d) <- %02x%02x%02x"
          % (row, col, rgb[0], rgb[1], rgb[2]))
    board.send(msg_pixel_write(addr, *rgb))
    board.flush()
    time.sleep(0.05)
    print("    sent. The board did the write; re-read to see it.")


def do_rect(board, args):
    """
    Fill a rectangle using Image Burst Write.

    The script computes the pixel VALUES to send, which is unavoidable --
    they have to come from somewhere. But it does not place them: the board
    walks the rectangle itself from the header's base, height and width, and
    the result is only ever seen by reading it back.
    """
    y0, x0, h, w, rgb = args.row, args.col, args.height_r, args.width_r, args.colour
    base = y0 * args.width + x0
    total = h * w

    print("sending an Image Burst Write: %dx%d at (%d,%d) <- %02x%02x%02x"
          % (h, w, y0, x0, rgb[0], rgb[1], rgb[2]))

    board.send(msg_burst_header(base, h, w))
    board.flush()
    time.sleep(0.02)

    sent = 0
    while sent < total:
        n = min(4, total - sent)
        board.send(msg_burst_data([rgb] * n + [(0, 0, 0)] * (4 - n)))
        sent += n
        if sent % 256 == 0:
            board.flush()
            time.sleep(0.005)
    board.flush()
    time.sleep(0.10)
    print("    %d pixels sent in %d burst data messages"
          % (total, (total + 3) // 4))


# =======================================================================
def parse_hex_colour(s):
    v = int(s, 16)
    return ((v >> 16) & 0xFF, (v >> 8) & 0xFF, v & 0xFF)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    # 8.75 Mbaud, not 8.125. The RTL clock moved 130 MHz -> 280 MHz and
    # uart_pkg now sets BAUD_RATE = 8_750_000 with DIV_TX = 32. The host's
    # FT2232H divides 12 MHz by 1.375 and delivers 8.727 Mbaud, a 0.26%
    # mismatch -- about 3% of a bit period accumulated over an 11-bit
    # frame, well inside budget. Asking for the OLD 8.125 Mbaud against a
    # board sending 8.75 is a 7.7% mismatch: nearly a full bit of drift by
    # the stop bit, and every byte fails parity or framing.
    ap.add_argument("--baud", type=int, default=8750000)
    ap.add_argument("--timeout", type=float, default=0.5)
    ap.add_argument("--width", type=int, default=256)
    ap.add_argument("--height", type=int, default=256)
    ap.add_argument("--mode", choices=("burst", "stream", "pixel"),
                    default="burst",
                    help="how a snapshot is read: one burst request (fast), "
                         "IMG_CTRL.start_img_read (whole image), or one "
                         "request per pixel (slow, address-echoed)")
    ap.add_argument("--walk", choices=("rect", "linear"), default="rect",
                    help="how the board walks a burst region: as a 2-D "
                         "window with the image width as stride (rect) or "
                         "as a linear run from the base (linear). Identical "
                         "for a full-width region; --verify tells you which "
                         "one your RTL does")
    ap.add_argument("--verify", type=int, default=12,
                    help="re-read this many random pixels with the "
                         "address-echoing Single Pixel Read and compare. 0 "
                         "disables")
    ap.add_argument("--retries", type=int, default=2,
                    help="re-request the outstanding rows this many times "
                         "after a framing error")
    ap.add_argument("--batch", type=int, default=64,
                    help="pixel reads issued before collecting replies "
                         "(--mode pixel only)")
    ap.add_argument("--region", nargs=4, type=int, metavar=("X", "Y", "W", "H"),
                    help="read only this window; the rest stays magenta")

    sub = ap.add_subparsers(dest="cmd", required=True)

    s = sub.add_parser("snapshot", help="read the image and write a PNG")
    s.add_argument("path")

    p = sub.add_parser("pixel", help="Single Pixel Write")
    p.add_argument("row", type=int)
    p.add_argument("col", type=int)
    p.add_argument("colour", type=parse_hex_colour, help="RRGGBB")

    r = sub.add_parser("rect", help="Image Burst Write of a solid rectangle")
    r.add_argument("row", type=int)
    r.add_argument("col", type=int)
    r.add_argument("height_r", type=int)
    r.add_argument("width_r", type=int)
    r.add_argument("colour", type=parse_hex_colour, help="RRGGBB")

    l = sub.add_parser("load", help="send a PNG to the board, then verify it")
    l.add_argument("path")
    l.add_argument("--at", nargs=2, type=int, default=(0, 0),
                   metavar=("X", "Y"), help="top-left corner on the board")
    l.add_argument("--size", nargs=2, type=int, metavar=("W", "H"),
                   help="region size (default: from --at to the edge)")
    l.add_argument("--fit", choices=("exact", "scale", "crop"),
                   default="exact",
                   help="what to do when the file is not the region size. "
                        "exact refuses; scale is nearest-neighbour; crop "
                        "takes the top-left and pads with black")
    l.add_argument("--pace", type=float, default=0.0,
                   help="seconds to sleep between write chunks. Leave at 0 "
                        "unless RTS/CTS is known broken")
    l.add_argument("--no-verify", action="store_true",
                   help="skip the read-back comparison")

    d = sub.add_parser("demo", help="before, modify, after -- three PNGs")
    d.add_argument("prefix")

    a = ap.parse_args()

    board = Board(a.port, a.baud, a.timeout)
    try:
        if a.cmd == "snapshot":
            do_snapshot(board, a, a.path)

        elif a.cmd == "load":
            do_load(board, a)

        elif a.cmd == "pixel":
            do_pixel(board, a)

        elif a.cmd == "rect":
            do_rect(board, a)

        elif a.cmd == "demo":
            print("\n--- 1. the image as it is now ---")
            do_snapshot(board, a, a.prefix + "_1_before.png")

            print("\n--- 2. burst write a rectangle ---")
            a.row, a.col = a.height // 4, a.width // 4
            a.height_r, a.width_r = a.height // 4, a.width // 4
            a.colour = (0x00, 0xC8, 0xFF)
            do_rect(board, a)
            do_snapshot(board, a, a.prefix + "_2_after_rect.png")

            print("\n--- 3. single pixel writes: a diagonal ---")
            for i in range(0, min(a.width, a.height), 8):
                a.row, a.col, a.colour = i, i, (0xFF, 0x00, 0x00)
                board.send(msg_pixel_write(i * a.width + i, 0xFF, 0x00, 0x00))
            board.flush()
            time.sleep(0.1)
            print("    diagonal sent as individual Single Pixel Writes")
            do_snapshot(board, a, a.prefix + "_3_after_pixels.png")

            print("\nCompare the three PNGs. Every pixel in all of them was")
            print("read back from the board -- the rectangle and the diagonal")
            print("appear because the BOARD stored them, not because this")
            print("script drew anything.")
    finally:
        board.close()

    print("\nbytes sent %d   received %d" % (board.sent, board.recv))
    return 0


if __name__ == "__main__":
    sys.exit(main())
