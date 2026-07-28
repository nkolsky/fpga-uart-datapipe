"""
lab10_capture.py
-----------------
Same protocol as Lab 9 (rx_parser.sv / msg_composer.sv / rgf.sv), plus a
helper for exercising Lab 10's new demo clock mux register.

  row = RGF register INDEX (rgf_pkg.sv address map):
      row=0 -> IMG_STATUS   (0x00)
      row=1 -> IMG_TX_MON   (0x04)
      row=2 -> IMG_CTRL     (0x08)
      row=3 -> FIFO_STATUS  (0x0C)
      row=4 -> CLK_CTRL     (0x10)  -- Lab 10: demo clock mux select
  col = read/write opcode: EVEN = write, ODD = read.
  val = write data.

  Demo clock mux (Lab 10): {R004,C000,V001} selects the PLL clock (LED[11]
  lights, LED[12] heartbeat blinks faster); {R004,C000,V000} selects
  CLK100MHZ (both revert). This does NOT affect UART timing -- the demo
  mux only drives a standalone counter, never the operational clock. See
  chip_top.sv's PLL/clock mux status block for why those are kept separate.

  UPDATE: parity is now ENABLED (EVEN). rx_phy.sv now correctly checks
  parity and gates byte_valid on it, matching tx_phy.sv's framing -- both
  directions need this now, not just reading replies. (Earlier versions
  of this script deliberately left parity off, back when only tx_phy.sv
  had parity support and rx_phy.sv still expected old 10-bit frames --
  that asymmetry is resolved now that both sides match.)

  UART baud rate is now 8,125,000. tx_phy/rx_phy HAVE been moved onto the
  fixed 130 MHz PLL clock (pll_clk_out) -- the whole UART + FIFO-read domain
  runs there now, while only the demo heartbeat counter remains on the muxed
  sys_clk. With uart_pkg DIV_TX = 130 MHz / 8.125 Mbaud = 16 exactly, the
  divisor drift is zero. This baud MUST match uart_pkg.BAUD_RATE and the PC
  port setting below.
"""

import serial
import time
from PIL import Image


# ============================================================
# User configuration
# ============================================================

PORT = "COM5"
BAUD_RATE = 8_125_000   # FPGA UART is now on the 130 MHz PLL clock:
                         # DIV_TX = 130 MHz / 8.125 Mbaud = 16 exactly (zero
                         # divisor drift). MUST match uart_pkg.BAUD_RATE. If
                         # your PC's FTDI bridge can't sustain 8.125 M (>5 M
                         # async is at the edge of what some USB-UART bridges
                         # do reliably -- exactly the case the Lab 10 spec
                         # flags), drop BOTH this and uart_pkg.BAUD_RATE to a
                         # rate that still divides 130 MHz cleanly, e.g.
                         # 130M/26 = 5.00 M or 130M/20 = 6.50 M.

IMG_WIDTH = 256
IMG_HEIGHT = 256
PACKET_BYTES = 16

# RGF register indices (see rgf_pkg.sv)
RGF_IMG_STATUS  = 0
RGF_IMG_TX_MON  = 1
RGF_IMG_CTRL    = 2
RGF_FIFO_STATUS = 3
RGF_CLK_CTRL    = 4   # Lab 10: demo clock mux select


def open_uart() -> serial.Serial:
    ser = serial.Serial(
        port=PORT,
        baudrate=BAUD_RATE,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_EVEN,   # rx_phy.sv now gates byte_valid on
                                      # parity passing -- commands sent
                                      # without a real parity bit are
                                      # silently rejected. tx_phy.sv has
                                      # sent parity-framed bytes for a
                                      # while; rx_phy.sv catching up is
                                      # what makes this necessary now on
                                      # BOTH directions, not just reads.
        stopbits=serial.STOPBITS_ONE,
        timeout=2.0,
        write_timeout=2.0,
        xonxoff=False,
        rtscts=False,      # NOTE: automatic hardware flow control via this
                            # flag depends on the OS/FTDI VCP driver actually
                            # implementing it correctly, which is unreliable
                            # on Windows generic FTDI drivers. We drive RTS
                            # manually instead (below) for a deterministic,
                            # verifiable signal rather than trusting driver
                            # auto-negotiation we can't directly observe.
        dsrdtr=False,
    )

    time.sleep(0.2)

    # Manually drive RTS high (logic level, per pyserial semantics --
    # True typically asserts the pin). This feeds the FPGA's UART_CTS
    # input. Check LED[10] (raw UART_CTS passthrough) on the board to see
    # what level the FPGA is actually receiving for this -- don't trust
    # ser.cts/ser.rts readback alone, confirm against the LED.
    ser.rts = True

    print(f"Opened {PORT} at {BAUD_RATE} baud")
    print(f"CTS: {ser.cts}")
    print(f"RTS: {ser.rts}")

    return ser


def send_command(ser: serial.Serial, cmd: str) -> None:
    if len(cmd) != 16 or not cmd.startswith("{") or not cmd.endswith("}"):
        raise ValueError(f"Command must be exactly 16 chars, wrapped in {{}}: got {cmd!r}")

    print(f"Sending command: {cmd}")
    ser.write(cmd.encode("ascii"))
    ser.flush()


def build_rgf_write_command(reg_index: int, value: int) -> str:
    """Build a {R###,C###,V###} command that WRITES `value` to `reg_index`."""
    if not (0 <= reg_index <= 999):
        raise ValueError("reg_index must fit in 3 ASCII digits (0-999)")
    if not (0 <= value <= 999):
        raise ValueError("value must fit in 3 ASCII digits (0-999) -- "
                          "this helper only covers the ASCII decimal wire format")
    col = 0  # even = write
    return f"{{R{reg_index:03d},C{col:03d},V{value:03d}}}"


def build_rgf_read_command(reg_index: int) -> str:
    """Build a {R###,C###,V###} command that READS (and read-to-clears,
    for IMG_TX_MON) `reg_index`. val is unused on reads."""
    if not (0 <= reg_index <= 999):
        raise ValueError("reg_index must fit in 3 ASCII digits (0-999)")
    col = 1  # odd = read
    return f"{{R{reg_index:03d},C{col:03d},V000}}"


def read_exact(ser: serial.Serial, n: int) -> bytes:
    data = ser.read(n)
    if len(data) != n:
        raise TimeoutError(f"Expected {n} bytes, got {len(data)} bytes: {data.hex(' ')}")
    return data


def resync_and_read_packet(ser: serial.Serial) -> bytes:
    """
    Read one 16-byte packet, resyncing on '{' first. Mirrors the same
    resync-on-frame-marker logic used in tb_chip_top.sv, for the same
    reason: the very first byte read after opening the port (or after a
    rejected/no-op command) can land mid-message, or never arrive at all.
    """
    b = ser.read(1)
    while b and b != b"{":
        b = ser.read(1)
    if not b:
        raise TimeoutError("Timed out waiting for '{' frame start")

    rest = read_exact(ser, PACKET_BYTES - 1)
    return b + rest


def parse_pixel_packet(packet: bytes) -> tuple[int, int, int, int, int]:
    """Returns (row, col, r, g, b). Raises ValueError if framing is wrong."""
    if not (packet[0:1] == b"{" and packet[1:2] == b"R" and packet[5:6] == b","
            and packet[6:7] == b"C" and packet[10:11] == b"," and packet[11:12] == b"P"
            and packet[15:16] == b"}"):
        raise ValueError(f"Bad framing: {packet.hex(' ')}")

    row = ((packet[3] & 0x03) << 8) | packet[4]
    col = ((packet[8] & 0x03) << 8) | packet[9]
    r, g, b = packet[12], packet[13], packet[14]
    return row, col, r, g, b


def capture_image(ser: serial.Serial, out_file: str, expect_transfer: bool = True) -> bool:
    """
    Sends the start command and attempts to capture a full image.
    Returns True if a full image was captured, False if the interlock
    rejected the start (no '{' ever arrived) or the port timed out.
    Raises nothing for the "expected rejection" case -- that's a normal,
    valid outcome this function is meant to report, not an exception.
    """
    ser.reset_input_buffer()
    ser.reset_output_buffer()

    start_cmd = build_rgf_write_command(RGF_IMG_CTRL, 1)
    send_command(ser, start_cmd)

    img = Image.new("RGB", (IMG_WIDTH, IMG_HEIGHT))
    expected_row = 0
    expected_col = 0
    error_count = 0
    total_packets = IMG_WIDTH * IMG_HEIGHT

    print(f"Capturing up to {total_packets} packets...")
    start_time = time.time()

    for i in range(total_packets):
        try:
            packet = resync_and_read_packet(ser)
        except TimeoutError:
            if i == 0:
                # No transfer ever started -- either the interlock
                # rejected the write (expected, if expect_transfer=False)
                # or something is actually broken (unexpected, if
                # expect_transfer=True).
                if expect_transfer:
                    print("No image received -- UNEXPECTED (a transfer was expected here).")
                else:
                    print("No image received -- as expected: the interlock rejected the write.")
                return False
            else:
                print(f"Timed out mid-transfer after {i}/{total_packets} packets -- unexpected.")
                return False

        try:
            row, col, r, g, b = parse_pixel_packet(packet)
        except ValueError as e:
            error_count += 1
            if error_count <= 20:
                print(f"[{i}] {e}")
            continue

        if row != expected_row or col != expected_col:
            error_count += 1
            if error_count <= 20:
                print(f"Coordinate mismatch at packet {i}: "
                      f"expected row={expected_row}, col={expected_col}, got row={row}, col={col}")

        if 0 <= row < IMG_HEIGHT and 0 <= col < IMG_WIDTH:
            img.putpixel((col, row), (r, g, b))
        else:
            error_count += 1
            if error_count <= 20:
                print(f"Out-of-range coordinate: row={row}, col={col}")

        expected_col += 1
        if expected_col == IMG_WIDTH:
            expected_col = 0
            expected_row += 1

        if (i + 1) % 16384 == 0:
            percent = 100.0 * (i + 1) / total_packets
            print(f"Captured {i + 1}/{total_packets} packets ({percent:.1f}%)")

    elapsed = time.time() - start_time
    img.save(out_file)

    print(f"Capture complete! Saved to: {out_file}")
    print(f"Elapsed time: {elapsed:.2f} seconds, errors: {error_count}")
    return True


def main() -> None:
    ser = open_uart()

    try:
        # ---- Stage 1: normal transfer ----
        print("\n==== Stage 1: start a transfer, expect success ====")
        ok1 = capture_image(ser, "lab10_capture_run1.png", expect_transfer=True)
        if not ok1:
            print("STAGE 1 FAILED -- stopping here, nothing downstream will make sense.")
            return
        print("Stage 1: PASS")

        # ---- Stage 2: immediate re-trigger, no reset -- expect rejection ----
        print("\n==== Stage 2: start again with NO reset -- expect interlock REJECTION ====")
        time.sleep(0.5)
        ok2 = capture_image(ser, "lab10_capture_run2_should_not_exist.png", expect_transfer=False)
        if ok2:
            print("STAGE 2 UNEXPECTED: a second transfer happened -- the interlock did NOT "
                  "reject it. This means IMG_TX_MON.complete either isn't being set, or "
                  "isn't being checked correctly. Worth investigating before trusting stage 3.")
        else:
            print("Stage 2: PASS (interlock correctly rejected the second start)")

        # ---- Stage 3: read-to-clear, then retry ----
        print("\n==== Stage 3: read IMG_TX_MON to clear/re-arm, then retry ====")
        clear_cmd = build_rgf_read_command(RGF_IMG_TX_MON)
        send_command(ser, clear_cmd)
        time.sleep(0.1)

        print("\n==== Stage 4: start again after clear -- expect success ====")
        ok3 = capture_image(ser, "lab10_capture_run3.png", expect_transfer=True)
        if ok3:
            print("Stage 4: PASS (read-to-clear successfully re-armed the interlock)")
        else:
            print("Stage 4 FAILED: transfer still didn't happen after the read-clear.")

        print("\n=========================================")
        print(" SUMMARY")
        print(f"  Stage 1 (initial transfer):         {'PASS' if ok1 else 'FAIL'}")
        print(f"  Stage 2 (interlock rejects retry):  {'PASS' if not ok2 else 'FAIL'}")
        print(f"  Stage 4 (re-armed after read-clear): {'PASS' if ok3 else 'FAIL'}")
        print("=========================================")

        # ---- Stage 5: Lab 10 demo clock mux ----
        # No UART reply to check here -- this is purely an LED-observation
        # test. demo_clk_sel only drives a standalone counter, never the
        # operational clock, so there's nothing else on the wire to verify.
        print("\n==== Stage 5: Lab 10 demo clock mux (watch LEDs, no auto-check) ====")
        print("Sending clk_sel=1 (select PLL clock)...")
        send_command(ser, build_rgf_write_command(RGF_CLK_CTRL, 1))
        print("  Expect: LED[11] on, LED[12] heartbeat blinking ~1.3x faster")
        time.sleep(3)

        print("Sending clk_sel=0 (select CLK100MHZ fallback)...")
        send_command(ser, build_rgf_write_command(RGF_CLK_CTRL, 0))
        print("  Expect: LED[11] off, LED[12] heartbeat back to the slower rate")
        time.sleep(3)

    finally:
        ser.close()
        print("UART closed")


if __name__ == "__main__":
    main()