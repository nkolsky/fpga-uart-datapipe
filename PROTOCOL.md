# UART Image-Transfer Protocol

## 1. UART configuration

The FPGA receive path expects:

| Setting | Value |
|---|---|
| Baud rate | 8,125,000 |
| Data bits | 8 |
| Parity | Even |
| Stop bits | 1 |
| Hardware flow control | Disabled in the current host utilities |

Even parity is required. A byte with invalid parity is rejected by the receive physical layer before message decoding.

## 2. General framing

Image commands use binary message fields surrounded by fixed delimiter bytes:

| Symbol | Hex |
|---|---:|
| `{` | `0x7B` |
| `}` | `0x7D` |
| `,` | `0x2C` |

Multi-byte numeric fields are encoded as unsigned big-endian binary values, not ASCII decimal text.

## 3. Single Pixel Write

A Single Pixel Write is an 11-byte frame:

```text
{ W A2 A1 A0 , P R G B }
```

| Byte | Meaning |
|---:|---|
| 0 | `{` |
| 1 | `W` opcode |
| 2–4 | 24-bit pixel index, big-endian |
| 5 | `,` |
| 6 | `P` field tag |
| 7 | Red |
| 8 | Green |
| 9 | Blue |
| 10 | `}` |

Example: write orange `(255,165,0)` to pixel index 1026 (`0x000402`):

```text
7B 57 00 04 02 2C 50 FF A5 00 7D
```

Valid pixel indices are `0` through `65535` for the current 256 × 256 framebuffer. Out-of-range commands are rejected.

## 4. Burst Write header

A Burst Write begins with one 16-byte header:

```text
{ I Ø Ø Ø , H H2 H1 H0 , W W2 W1 W0 }
```

| Byte | Value / field |
|---:|---|
| 0 | `{` |
| 1 | `I` Burst Write opcode |
| 2–4 | Don't-care bytes |
| 5 | `,` |
| 6 | `H` |
| 7–9 | 24-bit height, big-endian |
| 10 | `,` |
| 11 | `W` |
| 12–14 | 24-bit width, big-endian |
| 15 | `}` |

Example: 4 × 4 transfer:

```text
7B 49 00 00 00 2C 48 00 00 04 2C 57 00 00 04 7D
```

### Header rules

- `1 ≤ H ≤ 256`
- `1 ≤ W ≤ 256`
- the three don't-care bytes are accepted at any value;
- the current implementation starts at framebuffer coordinate `(0,0)`;
- the FPGA generates SRAM addresses internally.

## 5. Burst Data frame

Each 16-byte Burst Data frame carries four consecutive RGB pixels:

```text
{ R0 G0 B0 R1 , G1 B1 R2 G2 , B2 R3 G3 B3 }
```

| Byte | Field |
|---:|---|
| 0 | `{` |
| 1 | R0 |
| 2 | G0 |
| 3 | B0 |
| 4 | R1 |
| 5 | `,` |
| 6 | G1 |
| 7 | B1 |
| 8 | R2 |
| 9 | G2 |
| 10 | `,` |
| 11 | B2 |
| 12 | R3 |
| 13 | G3 |
| 14 | B3 |
| 15 | `}` |

The delimiters split pixels 1 and 2 across byte groups; removing bytes 0, 5, 10, and 15 yields twelve bytes forming four RGB triplets in order.

Example first data frame for the colors `(1,2,3)`, `(17,18,19)`, `(33,34,35)`, `(49,50,51)`:

```text
7B 01 02 03 11 2C 12 13 21 22 2C 23 31 32 33 7D
```

## 6. Burst addressing

The Burst Write controller interprets H and W as a row-major rectangle. For pixel `i` within a burst:

```text
burst_row = i // W
burst_col = i % W
framebuffer_address = burst_row × 256 + burst_col
```

A 2 × 3 burst therefore writes:

```text
(row 0) addresses   0,   1,   2
(row 1) addresses 256, 257, 258
```

It does not write the linear address sequence `0..5`.

## 7. Final partial frame

The number of required data frames is:

```text
ceil(H × W / 4)
```

When `H × W` is not divisible by four, the host fills the unused slots of the final frame with arbitrary padding pixels. The FPGA ignores all slots after the final real pixel and does not write or advance the address for them.

## 8. Burst mode and opcode bypass

Burst payload is arbitrary binary data. A red channel byte, for example, may equal the ASCII value of `W`, `R`, `P`, `{`, `}`, or `,`.

After accepting a Burst Write header, the decoder treats subsequent 16-byte messages as Burst Data frames without inspecting byte 1 as an opcode. The normal classifier is also gated during the burst, preventing payload bytes from triggering Register File or Single Pixel Write operations.

## 9. Command acceptance and backpressure

The Burst Write controller exposes a ready/valid command interface to the asynchronous command FIFO.

- A command is accepted only when `cmd_valid && cmd_ready` is true at a clock edge.
- While `cmd_ready` is low, address and pixel outputs remain stable.
- Burst position advances only after acceptance.
- The burst completes only after the final real command is accepted.

The current host utilities do not enable CTS hardware flow control. Burst uploads should not be started while framebuffer readback is actively using the SRAM ports.

## 10. Framebuffer readback

Framebuffer capture is initiated through the Register File command interface. The FPGA serializes the complete 256 × 256 framebuffer and the Python capture utility reconstructs a PNG.

The read interlock rejects a duplicate request while a previous image transfer remains marked in flight. Reading the image-transfer monitor register clears the completion state and permits the next capture. The current capture utility performs a pre-run clear automatically for repeated testing.

## 11. Host utility examples

### Capture

```powershell
python Scripts/lab10_capture.py
```

### Single Pixel Write

```powershell
python Scripts/test_single_pixel_write.py
```

### 4 × 4 test burst

```powershell
python Scripts/test_burst_write.py COM5
```

### Upload an image as-is

```powershell
python Scripts/rotate_and_send.py COM5 image.png
```

### Rotate 90° clockwise and upload

```powershell
python Scripts/rotate_and_send.py COM5 image.png --rotate 90 --save rotated_90.png
```

Images larger than 256 × 256 must be resized or cropped on the host before upload. Smaller images replace only the corresponding top-left rectangle of the framebuffer.

## 12. Error and status behavior

The design reports or latches status for conditions including:

- malformed or unsupported messages;
- command FIFO overflow;
- out-of-range pixel writes;
- duplicate framebuffer-read requests;
- image-transfer completion state.

Exact register addresses and board LED mappings are defined by the RTL packages and top-level integration and may be changed without altering the image-transfer frame formats documented here.
