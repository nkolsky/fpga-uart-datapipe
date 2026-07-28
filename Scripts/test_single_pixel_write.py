import time
import serial

PORT = "COM5"
BAUD = 8_125_000

# {W<A2,A1,A0>,P<R,G,B>}
frame = bytes.fromhex(
    "7B 57 00 04 02 2C 50 FF A5 00 7D"
)

print("Frame:", frame.hex(" "))

with serial.Serial(
    port=PORT,
    baudrate=BAUD,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_EVEN,
    stopbits=serial.STOPBITS_ONE,
    timeout=1,
    write_timeout=2,
    rtscts=True,
) as ser:
    time.sleep(0.5)

    print("CTS:", ser.cts)
    print("RTS:", ser.rts)

    written = ser.write(frame)
    ser.flush()

    print(f"Wrote {written} bytes")

    # Leave the port open briefly so transmission completes.
    time.sleep(1.0)

print("Done.")
print("Expected: LED[15] should latch ON.")