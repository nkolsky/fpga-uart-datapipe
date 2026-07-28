"""
Final project -- maximum-throughput bulk image capture and interlock regression.

Formerly Lab10_capture_fast_bulk.py. Renamed because it is now the standing
read-path regression for the final project, run after every RTL change and
between hardware experiments.

REUSABLE WITHOUT A BOARD RESET. A completed capture leaves
IMG_TX_MON.complete set, and the RGF interlock then rejects the next
IMG_CTRL.start. Previously that meant pressing Reset before every run. A
pre-run clear now handles it automatically; --skip-preclear disables it for
testing genuine power-on behaviour.

This version keeps the FPGA protocol and UART configuration unchanged:
- COM5
- 8,125,000 baud
- 8 data bits
- even parity
- 1 stop bit
- manual RTS assertion

The receive loop performs large blocking reads with no parsing, image updates,
progress printing, in_waiting polling, or deliberate sleeps while the FPGA is
streaming. Parsing happens only after all expected bytes have been received.
"""

from __future__ import annotations

import argparse
import time
from pathlib import Path

import serial
from PIL import Image


PORT = "COM5"
BAUD_RATE = 8_125_000

IMG_WIDTH = 256
IMG_HEIGHT = 256
PACKET_BYTES = 16
TOTAL_PACKETS = IMG_WIDTH * IMG_HEIGHT
TOTAL_IMAGE_BYTES = TOTAL_PACKETS * PACKET_BYTES

# Large read requests reduce Python/driver overhead. The Windows FTDI driver may
# return smaller chunks internally; that is fine.
READ_SIZE = 65_536

# A full transfer normally takes about 1.5 seconds. This permits ample margin.
FIRST_BYTE_TIMEOUT_S = 2.0
STALL_TIMEOUT_S = 2.0
TOTAL_TIMEOUT_S = 6.0

RGF_IMG_TX_MON = 1
RGF_IMG_CTRL = 2

# Output filenames. Kept as a prefix so a renamed script does not scatter
# differently-named artefacts around the working directory.
OUT_PREFIX = "fp_capture"

# Quiet period after the pre-run clear, before flushing the input buffer.
# The command is 16 bytes = 21.7 us on the wire at 8.125 Mbaud; 20 ms is
# three orders of magnitude beyond that and covers host buffering latency.
PRECLEAR_QUIET_S = 0.020


def open_uart(port: str = PORT) -> serial.Serial:
    ser = serial.Serial(
        port=port,
        baudrate=BAUD_RATE,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_EVEN,
        stopbits=serial.STOPBITS_ONE,
        # A short timeout lets us detect a stalled stream without adding sleeps.
        timeout=0.05,
        write_timeout=2.0,
        xonxoff=False,
        rtscts=False,
        dsrdtr=False,
    )

    time.sleep(0.2)
    ser.rts = True

    print(f"Opened {port} at {BAUD_RATE} baud")
    print(f"CTS: {ser.cts}")
    print(f"RTS: {ser.rts}")
    return ser


def send_command(ser: serial.Serial, cmd: str) -> None:
    if len(cmd) != 16 or not cmd.startswith("{") or not cmd.endswith("}"):
        raise ValueError(
            f"Command must be exactly 16 characters wrapped in braces: {cmd!r}"
        )

    print(f"Sending command: {cmd}")
    payload = cmd.encode("ascii")
    written = ser.write(payload)
    ser.flush()

    if written != len(payload):
        raise IOError(f"Expected to write {len(payload)} bytes, wrote {written}")


def build_rgf_write_command(reg_index: int, value: int) -> str:
    if not 0 <= reg_index <= 999:
        raise ValueError("reg_index must fit in three decimal digits")
    if not 0 <= value <= 999:
        raise ValueError("value must fit in three decimal digits")
    return f"{{R{reg_index:03d},C000,V{value:03d}}}"


def build_rgf_read_command(reg_index: int) -> str:
    if not 0 <= reg_index <= 999:
        raise ValueError("reg_index must fit in three decimal digits")
    return f"{{R{reg_index:03d},C001,V000}}"


def receive_exact_stream(
    ser: serial.Serial,
    expected_bytes: int,
    *,
    expect_transfer: bool,
) -> bytes | None:
    """
    Drain the UART as aggressively as possible with large blocking reads.

    Returns:
        bytes: all received bytes, possibly short if the stream stalls
        None: no transfer began within FIRST_BYTE_TIMEOUT_S
    """
    raw = bytearray()
    start = time.perf_counter()
    first_byte_deadline = start + FIRST_BYTE_TIMEOUT_S
    total_deadline = start + TOTAL_TIMEOUT_S
    last_progress = start

    # First-byte phase.
    while not raw:
        chunk = ser.read(min(READ_SIZE, expected_bytes))
        now = time.perf_counter()

        if chunk:
            raw.extend(chunk)
            last_progress = now
            break

        if now >= first_byte_deadline:
            if expect_transfer:
                print("No image received -- UNEXPECTED.")
            else:
                print("No image received -- expected interlock rejection.")
            return None

    # Continuous bulk-drain phase. No printing or sleeping in this loop.
    while len(raw) < expected_bytes:
        remaining = expected_bytes - len(raw)
        chunk = ser.read(min(READ_SIZE, remaining))
        now = time.perf_counter()

        if chunk:
            raw.extend(chunk)
            last_progress = now
            continue

        if now - last_progress >= STALL_TIMEOUT_S or now >= total_deadline:
            break

    elapsed = time.perf_counter() - start
    print(f"Bulk receive elapsed: {elapsed:.3f} s")
    print(f"Received total: {len(raw)}/{expected_bytes} bytes")
    return bytes(raw)


def parse_pixel_packet(packet: bytes) -> tuple[int, int, int, int, int]:
    if len(packet) != PACKET_BYTES:
        raise ValueError(f"Packet length {len(packet)} is not 16")

    if not (
        packet[0] == 0x7B              # {
        and packet[1] == ord("R")
        and packet[5] == ord(",")
        and packet[6] == ord("C")
        and packet[10] == ord(",")
        and packet[11] == ord("P")
        and packet[15] == 0x7D         # }
    ):
        raise ValueError(f"Bad framing: {packet.hex(' ')}")

    row = ((packet[3] & 0x03) << 8) | packet[4]
    col = ((packet[8] & 0x03) << 8) | packet[9]
    r, g, b = packet[12], packet[13], packet[14]
    return row, col, r, g, b


def parse_and_save(raw: bytes, output_file: str) -> bool:
    raw_path = Path(output_file).with_suffix(".raw.bin")

    if len(raw) != TOTAL_IMAGE_BYTES:
        raw_path.write_bytes(raw)
        missing = TOTAL_IMAGE_BYTES - len(raw)
        print(f"Capture is short by {missing} bytes.")
        print(f"Saved incomplete raw stream to: {raw_path}")
        return False

    image = Image.new("RGB", (IMG_WIDTH, IMG_HEIGHT))
    errors = 0

    for packet_index in range(TOTAL_PACKETS):
        offset = packet_index * PACKET_BYTES
        packet = raw[offset : offset + PACKET_BYTES]

        try:
            row, col, r, g, b = parse_pixel_packet(packet)
        except ValueError as exc:
            errors += 1
            if errors <= 20:
                print(f"[{packet_index}] {exc}")
            continue

        expected_row = packet_index // IMG_WIDTH
        expected_col = packet_index % IMG_WIDTH

        if row != expected_row or col != expected_col:
            errors += 1
            if errors <= 20:
                print(
                    f"Coordinate mismatch at packet {packet_index}: "
                    f"expected ({expected_row}, {expected_col}), "
                    f"received ({row}, {col})"
                )

        if 0 <= row < IMG_HEIGHT and 0 <= col < IMG_WIDTH:
            image.putpixel((col, row), (r, g, b))
        else:
            errors += 1
            if errors <= 20:
                print(f"Out-of-range coordinate: ({row}, {col})")

    image.save(output_file)
    print(f"Image saved to: {output_file}")
    print(f"Parse errors: {errors}")

    if errors:
        raw_path.write_bytes(raw)
        print(f"Saved raw stream to: {raw_path}")

    return errors == 0


def preclear_status(ser: serial.Serial) -> None:
    """
    Clear any IMG_TX_MON completion/error state left by a previous run.

    After a successful capture IMG_TX_MON.complete stays set, and the RGF
    interlock then rejects the next IMG_CTRL.start. Without this the script
    needs a board reset between runs.

    WHAT THIS TOUCHES, traced against rgf.sv. The command is
    {R001,C001,V000}: row 001 selects IMG_TX_MON (pc_addr 0x04) and col 001
    is odd, which the design decodes as a READ (pc_wen = 0). There are
    exactly three pc_addr decodes in the register file:

        pc_sel_img_tx_mon = (pc_addr == 0x04)  used with !pc_wen  <- fires
        pc_sel_img_ctrl   = (pc_addr == 0x08)  used with  pc_wen  <- no
        pc_sel_clk_ctrl   = (pc_addr == 0x10)  used with  pc_wen  <- no

    IMG_CTRL and CLK_CTRL both require pc_wen HIGH, so a read cannot reach
    either. IMG_STATUS, FIFO_STATUS and PARITY_FAULT_CNT have no pc_addr
    decode at all. Within IMG_TX_MON the read-to-clear touches only
    img_send_complete and img_send_error; row_cnt and col_cnt are preserved.

    So this cannot start a transfer, change the clock select, or disturb the
    monitor's coordinate fields.

    WHAT IT CANNOT FIX: mem_interlock holds its own img_in_flight flag, set
    by read_go and cleared only by tx_img_done or reset. It is not
    host-accessible. After a normal capture it is already clear. After an
    INTERRUPTED capture it can be stuck, and no command will clear it --
    that still needs Reset.

    Fire-and-forget: the design has no read-reply path, so there is nothing
    to verify. Stage 1 succeeding is the confirmation.
    """
    print("==== Pre-run: clear stale IMG_TX_MON status ====")

    # Discard both directions first. A previous run interrupted mid-image can
    # leave the host buffer full of pixel data that would otherwise be
    # mistaken for Stage 1 output.
    ser.reset_output_buffer()
    ser.reset_input_buffer()

    send_command(ser, build_rgf_read_command(RGF_IMG_TX_MON))

    # Quiet period, then flush again. The FPGA sends nothing in reply to a
    # register read, but stale bytes still in flight from before the clear
    # would corrupt Stage 1 framing.
    time.sleep(PRECLEAR_QUIET_S)
    ser.reset_input_buffer()

    print("IMG_TX_MON cleared, input buffer flushed.")


def capture_once(
    ser: serial.Serial,
    output_file: str,
    *,
    expect_transfer: bool,
) -> bool:
    ser.reset_input_buffer()
    ser.reset_output_buffer()

    send_command(ser, build_rgf_write_command(RGF_IMG_CTRL, 1))
    print(f"Receiving {TOTAL_IMAGE_BYTES} bytes with large blocking reads...")

    raw = receive_exact_stream(
        ser,
        TOTAL_IMAGE_BYTES,
        expect_transfer=expect_transfer,
    )

    if raw is None:
        return False

    if not expect_transfer:
        print("A transfer occurred although the interlock should have rejected it.")
        parse_and_save(raw, output_file)
        return True

    return parse_and_save(raw, output_file)


def main() -> None:
    ap = argparse.ArgumentParser(
        description="Final project image capture and interlock regression"
    )
    ap.add_argument("--port", default=PORT,
                    help=f"serial port (default {PORT})")
    ap.add_argument("--skip-preclear", action="store_true",
                    help="do NOT clear IMG_TX_MON before Stage 1. Only for "
                         "testing genuine power-on/reset behaviour, where the "
                         "monitor is expected to be clear already.")
    args = ap.parse_args()

    print("=" * 55)
    print(" Final project -- capture and interlock regression")
    print("=" * 55)
    print("Program the FPGA and press Reset once after loading a new bitstream.")
    print("For later runs, the script clears IMG_TX_MON automatically.")

    ser = open_uart(args.port)

    try:
        # ---- Pre-run cleanup, before Stage 1 ------------------------------
        # Separate from, and in addition to, the Stage 3 clear. It makes the
        # script reusable after a previous capture, a Single Pixel Write or a
        # Burst Write without pressing Reset.
        print()
        if args.skip_preclear:
            print("==== Pre-run clear SKIPPED (--skip-preclear) ====")
            print("Expecting a freshly reset board with IMG_TX_MON clear.")
        else:
            preclear_status(ser)

        print("\n==== Stage 1: initial transfer ====")
        stage1 = capture_once(
            ser,
            f"{OUT_PREFIX}_run1.png",
            expect_transfer=True,
        )

        if not stage1:
            print("Stage 1: FAIL")
            if not args.skip_preclear:
                print()
                print("Stage 1 failed even after the pre-run clear. If no image")
                print("started at all, the block is mem_interlock's img_in_flight")
                print("flag, not the RGF -- that is cleared only by a completed")
                print("transfer or by Reset, and is not reachable from the host.")
                print("Press Reset and try again.")
            return

        print("Stage 1: PASS")

        print("\n==== Stage 2: immediate retry should be rejected ====")
        time.sleep(0.5)
        stage2_transfer_occurred = capture_once(
            ser,
            f"{OUT_PREFIX}_run2_should_not_exist.png",
            expect_transfer=False,
        )

        if stage2_transfer_occurred:
            print("Stage 2: FAIL")
        else:
            print("Stage 2: PASS")

        print("\n==== Stage 3: read IMG_TX_MON to clear ====")
        send_command(ser, build_rgf_read_command(RGF_IMG_TX_MON))
        time.sleep(0.1)

        print("\n==== Stage 4: transfer after clear ====")
        stage4 = capture_once(
            ser,
            f"{OUT_PREFIX}_run3.png",
            expect_transfer=True,
        )

        print(f"Stage 4: {'PASS' if stage4 else 'FAIL'}")

        print("\n=========================================")
        print(" SUMMARY")
        print(f"  Pre-clear: {'skipped' if args.skip_preclear else 'performed'}")
        print(f"  Stage 1: {'PASS' if stage1 else 'FAIL'}")
        print(
            "  Stage 2: "
            f"{'PASS' if not stage2_transfer_occurred else 'FAIL'}"
        )
        print(f"  Stage 4: {'PASS' if stage4 else 'FAIL'}")
        print("=========================================")

    finally:
        ser.close()
        print("UART closed")


if __name__ == "__main__":
    main()
