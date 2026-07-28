#!/usr/bin/env python3
"""
test_burst_write.py -- Image Burst Write hardware test.

Sends one 4x4 Image Burst Write to the FPGA and exits. It does NOT read the
image back; run lab10_capture.py afterwards and diff against a capture taken
before this script.

The 16 pixel values are identical to those in tb_stage3_burst_pipeline.sv,
so hardware and simulation are checked against the same expected data.

--------------------------------------------------------------------------
UART CONFIGURATION -- the parity setting is not optional
--------------------------------------------------------------------------
rx_phy checks EVEN parity on every byte and silently drops any frame that
fails. A port opened with PARITY_NONE has no parity bit at all, so every byte
is rejected and nothing reaches the design -- with no error anywhere. This is
the single easiest way to make the whole test look mysteriously dead.

--------------------------------------------------------------------------
PROTOCOL -- both messages are 16 bytes
--------------------------------------------------------------------------

BURST HEADER   {I<0,0,0>, H<H2,H1,H0>, W<W2,W1,W0>}

    byte  0  0x7B '{'          byte  8  H1
    byte  1  0x49 'I'          byte  9  H0
    byte  2  don't care        byte 10  0x2C ','
    byte  3  don't care        byte 11  0x57 'W'
    byte  4  don't care        byte 12  W2
    byte  5  0x2C ','          byte 13  W1
    byte  6  0x48 'H'          byte 14  W0
    byte  7  H2                byte 15  0x7D '}'

    H and W are 24-bit BIG-ENDIAN BINARY, not ASCII digits. Three decimal
    digits would cap a dimension at 999; three raw bytes give 24 bits.

    The header enables "opcode bypass" for H*W pixels: while it is active
    every following frame is treated as burst data regardless of its byte 1,
    which is necessary because burst payload is raw binary and can happen to
    look like any other message.

BURST DATA     {<R0,G0,B0,R1>, <G1,B1,R2,G2>, <B2,R3,G3,B3>}

    byte  0  0x7B '{'          byte  8  R2
    byte  1  R0                byte  9  G2
    byte  2  G0                byte 10  0x2C ','
    byte  3  B0                byte 11  B2
    byte  4  R1                byte 12  R3
    byte  5  0x2C ','          byte 13  G3
    byte  6  G1                byte 14  B3
    byte  7  B1                byte 15  0x7D '}'

    Strip bytes 0, 5, 10 and 15 and the remaining twelve are four
    consecutive RGB triplets in order. The <...> groupings are frame
    punctuation, not a reordering -- the commas simply land mid-pixel, which
    is why pixels 1 and 2 straddle them.

--------------------------------------------------------------------------
GEOMETRY
--------------------------------------------------------------------------
H and W describe a ROW-MAJOR RECTANGLE anchored at pixel 0, not a linear run
of H*W addresses. For a 256-wide image:

    pixel index i  ->  row = i // W,  col = i % W
                   ->  address = row * 256 + col

so a 4x4 burst writes addresses 0..3, 256..259, 512..515, 768..771 --
NOT 0..15.
"""

import argparse
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("pyserial is required:  pip install pyserial")


# --------------------------------------------------------------------------
# Protocol constants. These are the ONLY hardcoded byte values -- everything
# else is computed from the arguments.
# --------------------------------------------------------------------------
CHAR_OPEN_BRACE  = 0x7B   # '{'
CHAR_CLOSE_BRACE = 0x7D   # '}'
CHAR_COMMA       = 0x2C   # ','
CHAR_I           = 0x49   # 'I'  burst-write header opcode
CHAR_H           = 0x48   # 'H'  height field opcode
CHAR_W           = 0x57   # 'W'  width field opcode

BAUD_RATE       = 8_125_000
MSG_BYTES       = 16
PIXELS_PER_FRAME = 4
IMG_WIDTH       = 256      # must match memory_pkg::IMG_WIDTH
IMG_HEIGHT      = 256

# Padding pixels for a final partial frame. The FPGA discards them without
# writing anything, so the value is arbitrary -- a recognisable sentinel just
# makes it obvious in a hex dump which pixels were filler.
PAD_PIXEL = (0xDE, 0xAD, 0x00)


# --------------------------------------------------------------------------
# UART
# --------------------------------------------------------------------------
def open_uart(port: str, timeout: float = 2.0) -> serial.Serial:
    """Open the FPGA UART. PARITY_EVEN is required -- see the module header."""
    ser = serial.Serial(
        port=port,
        baudrate=BAUD_RATE,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_EVEN,
        stopbits=serial.STOPBITS_ONE,
        timeout=timeout,
        rtscts=False,
    )
    ser.rts = True
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    return ser


# --------------------------------------------------------------------------
# Frame construction
# --------------------------------------------------------------------------
def build_burst_header(height: int, width: int) -> bytes:
    """Build the 16-byte Image Burst Write header."""
    if not (1 <= height <= IMG_HEIGHT):
        raise ValueError(f"height {height} outside 1..{IMG_HEIGHT}")
    if not (1 <= width <= IMG_WIDTH):
        raise ValueError(f"width {width} outside 1..{IMG_WIDTH}")

    frame = bytes([
        CHAR_OPEN_BRACE,
        CHAR_I,
        0x00, 0x00, 0x00,                  # don't-care field
        CHAR_COMMA,
        CHAR_H,
        (height >> 16) & 0xFF, (height >> 8) & 0xFF, height & 0xFF,
        CHAR_COMMA,
        CHAR_W,
        (width >> 16) & 0xFF, (width >> 8) & 0xFF, width & 0xFF,
        CHAR_CLOSE_BRACE,
    ])
    assert len(frame) == MSG_BYTES
    return frame


def build_burst_data(p0, p1, p2, p3) -> bytes:
    """Build one 16-byte Burst Data frame from four (R,G,B) tuples."""
    for p in (p0, p1, p2, p3):
        if len(p) != 3 or any(not (0 <= c <= 255) for c in p):
            raise ValueError(f"bad pixel {p}: expected three values 0..255")

    frame = bytes([
        CHAR_OPEN_BRACE,
        p0[0], p0[1], p0[2],               # R0 G0 B0
        p1[0],                             # R1
        CHAR_COMMA,
        p1[1], p1[2],                      # G1 B1
        p2[0], p2[1],                      # R2 G2
        CHAR_COMMA,
        p2[2],                             # B2
        p3[0], p3[1], p3[2],               # R3 G3 B3
        CHAR_CLOSE_BRACE,
    ])
    assert len(frame) == MSG_BYTES
    return frame


def pack_burst_frames(pixels, pad=PAD_PIXEL):
    """
    Convert [(R,G,B), ...] into a list of correctly formatted Burst Data
    frames, four pixels per frame.

    If len(pixels) is not a multiple of four the final frame is padded. The
    FPGA counts real pixels from H and W and discards anything beyond that,
    so the padding never reaches the SRAM.
    """
    padded = list(pixels)
    remainder = len(padded) % PIXELS_PER_FRAME
    n_pad = 0
    if remainder:
        n_pad = PIXELS_PER_FRAME - remainder
        padded.extend([pad] * n_pad)

    frames = [
        build_burst_data(*padded[i:i + PIXELS_PER_FRAME])
        for i in range(0, len(padded), PIXELS_PER_FRAME)
    ]
    return frames, n_pad


# --------------------------------------------------------------------------
# Geometry helper -- mirrors rx_burst_ctrl's address generation
# --------------------------------------------------------------------------
def pixel_address(index: int, width: int) -> int:
    """Linear SRAM pixel address for the index-th pixel of a W-wide burst."""
    return (index // width) * IMG_WIDTH + (index % width)


# --------------------------------------------------------------------------
# Debug output
# --------------------------------------------------------------------------
def hexdump(frame: bytes) -> str:
    return " ".join(f"{b:02X}" for b in frame)


def describe_frame(index, frame, pixels=None, first_pixel=None, width=None):
    line = f"  frame {index:>4}: {hexdump(frame)}"
    print(line)
    if pixels is not None:
        for k, p in enumerate(pixels):
            i = first_pixel + k
            addr = pixel_address(i, width)
            print(f"             pixel {i:>4} -> (row {addr // IMG_WIDTH:>3}, "
                  f"col {addr % IMG_WIDTH:>3})  addr {addr:>6}  RGB{p}")


# --------------------------------------------------------------------------
# Transmit
# --------------------------------------------------------------------------
def send_burst(ser, height, width, pixels, verbose=True):
    """Send a complete Image Burst Write: one header then the data frames."""
    expected = height * width
    if len(pixels) != expected:
        raise ValueError(
            f"{len(pixels)} pixels supplied but {height}x{width} needs {expected}")

    header = build_burst_header(height, width)
    frames, n_pad = pack_burst_frames(pixels)

    total_bytes = MSG_BYTES * (1 + len(frames))
    airtime_s = total_bytes * 11 / BAUD_RATE      # 8E1 = 11 bits per byte

    print(f"Burst Write  {height} x {width}  = {expected} pixels")
    print(f"  {len(frames)} data frame(s), {n_pad} padding pixel(s)")
    print(f"  {total_bytes} bytes, ~{airtime_s * 1e3:.2f} ms on the wire")
    print()

    print("HEADER")
    describe_frame(0, header)
    print()

    print("DATA FRAMES")
    for n, frame in enumerate(frames):
        first = n * PIXELS_PER_FRAME
        group = pixels[first:first + PIXELS_PER_FRAME]
        if verbose:
            describe_frame(n, frame, group, first, width)
        else:
            describe_frame(n, frame)
    print()

    # Send header and data as one contiguous stream. There is no protocol
    # gap requirement between frames -- rx_mac frames purely by byte count.
    payload = header + b"".join(frames)
    ser.write(payload)
    ser.flush()                       # block until the OS has taken it all

    # flush() only guarantees the host buffer is drained, not that the last
    # bit has left the wire. Wait out the airtime plus margin.
    time.sleep(airtime_s + 0.05)

    print(f"Sent {len(payload)} bytes.")
    return frames


# --------------------------------------------------------------------------
# Test data -- identical to tb_stage3_burst_pipeline.sv
# --------------------------------------------------------------------------
def make_test_pixels(count: int):
    """
    Pixel i = (16i+1, 16i+2, 16i+3), masked to 8 bits.

    Every channel of every pixel is distinct, so a mis-ordered, duplicated or
    dropped write is visible in the captured image rather than blending in.
    """
    return [(((16 * i + 1) & 0xFF),
             ((16 * i + 2) & 0xFF),
             ((16 * i + 3) & 0xFF)) for i in range(count)]


def print_expected_map(height, width, pixels):
    print("EXPECTED MODIFIED PIXELS")
    print(f"  {height}x{width} rectangle anchored at pixel 0")
    print()
    print("   idx   row   col     addr    R    G    B")
    print("  ----  ----  ----   ------  ---  ---  ---")
    for i, p in enumerate(pixels):
        addr = pixel_address(i, width)
        print(f"  {i:>4}  {addr // IMG_WIDTH:>4}  {addr % IMG_WIDTH:>4}   "
              f"{addr:>6}  {p[0]:>3}  {p[1]:>3}  {p[2]:>3}")
    print()
    print(f"  Every other pixel in the image must be UNCHANGED.")
    print(f"  Note the addresses are NOT 0..{height * width - 1}: a burst is a")
    print(f"  rectangle, so each row starts {IMG_WIDTH} apart.")


# --------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description="Image Burst Write hardware test")
    ap.add_argument("port", help="serial port, e.g. COM5 or /dev/ttyUSB1")
    ap.add_argument("--height", type=int, default=4)
    ap.add_argument("--width", type=int, default=4)
    ap.add_argument("--quiet", action="store_true",
                    help="hex only, omit the per-pixel breakdown")
    args = ap.parse_args()

    pixels = make_test_pixels(args.height * args.width)

    print("=" * 66)
    print(" Image Burst Write -- hardware test")
    print("=" * 66)
    print()

    ser = open_uart(args.port)
    try:
        send_burst(ser, args.height, args.width, pixels, verbose=not args.quiet)
    finally:
        ser.close()

    print()
    print("=" * 66)
    print_expected_map(args.height, args.width, pixels)
    print("=" * 66)
    print()
    print("Now run lab10_capture.py and diff against a capture taken before")
    print("this script. Exactly the pixels listed above should have changed.")


if __name__ == "__main__":
    main()
