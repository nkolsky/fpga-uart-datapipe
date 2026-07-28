#!/usr/bin/env python3
"""
rotate_and_send.py -- rotate an image and upload it over Image Burst Write.

    python rotate_and_send.py COM5 input.png --rotate 90
    python rotate_and_send.py COM5 input.png --rotate 180 --save rotated.png
    python rotate_and_send.py COM5 input.png --rotate 0

All burst protocol code is reused from test_burst_write.py -- this script
only handles image loading, rotation and geometry. Both files must sit in
the same directory.

--------------------------------------------------------------------------
ROTATION DIRECTION -- the easiest thing to get backwards
--------------------------------------------------------------------------
--rotate takes a CLOCKWISE angle, which is what people mean colloquially.
Pillow's own rotate() and its ROTATE_n transpose constants are
COUNTER-CLOCKWISE, so the mapping is deliberately inverted below:

    clockwise  90  ->  Image.ROTATE_270
    clockwise 180  ->  Image.ROTATE_180
    clockwise 270  ->  Image.ROTATE_90

transpose() is used rather than rotate() because it is exact: no resampling,
no interpolation, no filled corners, and the dimensions swap automatically
for the 90 and 270 cases. rotate() without expand=True would silently CROP a
non-square image.

--------------------------------------------------------------------------
GEOMETRY -- images smaller than the framebuffer
--------------------------------------------------------------------------
A burst write is a rectangle ANCHORED AT PIXEL 0, so a smaller image lands in
the TOP-LEFT corner of the 256x256 framebuffer and everything outside it is
left untouched. The image is NOT padded to fill the frame.

Rotating by 90 or 270 SWAPS the dimensions, so the uploaded rectangle changes
shape: a 64x128 image becomes 128x64. Both must still fit the framebuffer,
which is checked after rotation rather than before.

The origin is fixed at pixel 0 in the current RTL (the spec's burst-write
header carries three don't-care bytes where the burst-read header carries an
address). If that changes, only the header build needs updating.
"""

import argparse
import contextlib
import io
import math
import sys

try:
    from PIL import Image
except ImportError:
    sys.exit("Pillow is required:  pip install pillow")

# Reuse the burst protocol implementation unchanged.
try:
    from test_burst_write import (
        open_uart,
        send_burst,
        pixel_address,
        IMG_WIDTH,
        IMG_HEIGHT,
        BAUD_RATE,
        MSG_BYTES,
        PIXELS_PER_FRAME,
    )
except ImportError:
    sys.exit("test_burst_write.py must be in the same directory as this script")


# --------------------------------------------------------------------------
# Pillow moved the transpose constants into an enum in 9.1. Support both.
# --------------------------------------------------------------------------
try:
    _T = Image.Transpose          # Pillow >= 9.1
except AttributeError:            # pragma: no cover
    _T = Image                    # older Pillow

# Clockwise angle -> Pillow transpose op. Pillow's ROTATE_n is
# COUNTER-clockwise, hence the inversion.
CLOCKWISE_OP = {
    90:  _T.ROTATE_270,
    180: _T.ROTATE_180,
    270: _T.ROTATE_90,
}


def extract_pixels(img):
    """Row-major list of (R,G,B) tuples, across Pillow versions."""
    if hasattr(img, "get_flattened_data"):      # Pillow >= 12
        return img.get_flattened_data()
    return img.getdata()                        # Pillow < 12


def rotate_clockwise(img, degrees):
    """Rotate exactly, without resampling. 0 returns the image unchanged."""
    if degrees == 0:
        return img
    if degrees not in CLOCKWISE_OP:
        raise ValueError(f"rotation must be 0, 90, 180 or 270 (got {degrees})")
    return img.transpose(CLOCKWISE_OP[degrees])


# --------------------------------------------------------------------------
def load_and_rotate(path, degrees):
    """Open, convert to RGB, rotate clockwise, and check it fits."""
    img = Image.open(path)
    orig_mode = img.mode
    orig_w, orig_h = img.size

    img = img.convert("RGB")
    img = rotate_clockwise(img, degrees)
    w, h = img.size

    if h > IMG_HEIGHT or w > IMG_WIDTH:
        raise ValueError(
            f"rotated image is {w}x{h}, which exceeds the "
            f"{IMG_WIDTH}x{IMG_HEIGHT} framebuffer "
            f"(source was {orig_w}x{orig_h}, rotated {degrees} deg clockwise)")

    return img, (orig_w, orig_h, orig_mode)


def report(img, degrees, orig):
    """Print the transfer plan before anything is sent."""
    orig_w, orig_h, orig_mode = orig
    w, h = img.size
    n_pixels = w * h
    n_frames = math.ceil(n_pixels / PIXELS_PER_FRAME)
    n_pad = n_frames * PIXELS_PER_FRAME - n_pixels
    n_bytes = MSG_BYTES * (1 + n_frames)          # header + data frames
    airtime = n_bytes * 11 / BAUD_RATE            # 8E1 = 11 bits per byte

    print("=" * 66)
    print(" Image Burst Write -- upload")
    print("=" * 66)
    print(f"  source            : {orig_w} x {orig_h}, mode {orig_mode}")
    print(f"  rotation          : {degrees} degrees clockwise")
    print(f"  uploaded image    : {w} x {h}  (H={h}, W={w})")
    print(f"  pixels            : {n_pixels}")
    print(f"  burst data frames : {n_frames}"
          + (f"  ({n_pad} padding pixel(s) in the last frame)" if n_pad else ""))
    print(f"  bytes on the wire : {n_bytes}")
    print(f"  estimated airtime : {airtime * 1e3:.2f} ms")
    print()

    if w < IMG_WIDTH or h < IMG_HEIGHT:
        last = pixel_address(n_pixels - 1, w)
        print(f"  The image is smaller than the framebuffer, so it occupies the")
        print(f"  TOP-LEFT {w}x{h} corner: rows 0..{h-1}, columns 0..{w-1}.")
        print(f"  Addresses run from 0 to {last}, but NOT contiguously -- a burst")
        print(f"  is a rectangle, so each row starts {IMG_WIDTH} apart. Every pixel")
        print(f"  outside the rectangle is left UNCHANGED.")
        print()

    return n_frames


# --------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(
        description="Rotate an image and upload it via Image Burst Write")
    ap.add_argument("port", help="serial port, e.g. COM5 or /dev/ttyUSB1")
    ap.add_argument("image", help="input image file")
    ap.add_argument("--rotate", type=int, default=0, choices=[0, 90, 180, 270],
                    help="clockwise rotation in degrees (default 0)")
    ap.add_argument("--save", metavar="PATH",
                    help="save the rotated image before transmitting, so it "
                         "can be diffed directly against the framebuffer capture")
    ap.add_argument("--dump-frames", type=int, default=8, metavar="N",
                    help="show the first N data frames in hex (default 8, "
                         "use -1 for all)")
    args = ap.parse_args()

    img, orig = load_and_rotate(args.image, args.rotate)
    n_frames = report(img, args.rotate, orig)

    # Save BEFORE transmitting: if the upload fails partway, the reference
    # image for comparison still exists.
    if args.save:
        img.save(args.save)
        print(f"  rotated image saved to {args.save}")
        print()

    # Row-major RGB, exactly the order the burst controller expects.
    #
    # getdata() is deprecated from Pillow 12 and removed in 14, replaced by
    # get_flattened_data(). Both return the identical list of (R,G,B) tuples
    # in the same order -- verified -- so this picks whichever exists rather
    # than emitting a DeprecationWarning on newer installs or breaking on
    # older ones.
    pixels = list(extract_pixels(img))
    w, h = img.size
    assert len(pixels) == w * h

    ser = open_uart(args.port)
    try:
        # send_burst() prints one block per frame, which is unusable for a
        # large image. Capture its output and replay a controlled amount --
        # this keeps send_burst() itself untouched.
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            send_burst(ser, h, w, pixels, verbose=False)
        transcript = buf.getvalue().splitlines()
    finally:
        ser.close()

    # Replay: everything up to and including the header, then N data frames,
    # then the trailing summary line.
    frame_lines = [ln for ln in transcript if ln.lstrip().startswith("frame")]
    header_line = frame_lines[0] if frame_lines else ""
    data_lines = frame_lines[1:]

    print("HEADER")
    print(header_line)
    print()
    print("DATA FRAMES")
    show = len(data_lines) if args.dump_frames < 0 else args.dump_frames
    for ln in data_lines[:show]:
        print(ln)
    if show < len(data_lines):
        print(f"  ... {len(data_lines) - show} more frame(s) suppressed "
              f"(--dump-frames -1 to show all)")
    print()

    for ln in transcript:
        if ln.startswith("Sent "):
            print(ln)

    print()
    print("=" * 66)
    print(" Verification")
    print("=" * 66)
    print(f"  1. Run lab10_capture.py to capture the framebuffer.")
    if args.save:
        print(f"  2. Compare the top-left {w}x{h} region of the capture")
        print(f"     against {args.save} -- they should match exactly.")
    else:
        print(f"  2. The top-left {w}x{h} region should match the rotated")
        print(f"     source image. Re-run with --save to get a reference file.")
    print(f"  3. Every pixel outside that region must be unchanged.")
    print()
    print("  A numpy comparison:")
    print("     import numpy as np; from PIL import Image")
    print(f"     cap = np.array(Image.open('capture.png').convert('RGB'))")
    print(f"     ref = np.array(Image.open('{args.save or 'rotated.png'}').convert('RGB'))")
    print(f"     print(np.array_equal(cap[:{h}, :{w}], ref))")


if __name__ == "__main__":
    main()
