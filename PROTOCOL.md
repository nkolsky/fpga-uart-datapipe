# UART Image-Transfer Protocol

## 1. UART configuration


The FPGA receive path expects:

| Setting | Value |
|---|---|
| Baud rate | 8,000,000 |
| Data bits | 8 |
| Parity | Even |
| Stop bits | 1 |
| Hardware flow control | RTS/CTS, active in both directions |

The baud rate follows from the clocking: a 256 MHz UART domain with a
divide-by-32 bit period gives exactly 8 Mbaud. Host utilities must be given
`--baud 8000000` explicitly; several still default to an older value.

Even parity is required. A byte with invalid parity is rejected by the receive
physical layer before message decoding, and increments `PARITY_FAULT_CNT`.

**The wire format has not changed.** The design now carries register traffic
over APB and image memory over AHB-Lite internally, but both buses sit behind
the message decoder — nothing about the framing, opcodes or field layout below
is affected by them.

## 2. General framing


Image commands use binary message fields surrounded by fixed delimiter bytes:

| Symbol | Hex |
|---|---:|
| `{` | `0x7B` |
| `}` | `0x7D` |
| `,` | `0x2C` |

Multi-byte numeric fields are encoded as unsigned big-endian binary values, not ASCII decimal text.

## 3. Message summary


Every message is `1 + 5n` bytes: an opening brace, then `n` groups of
`opcode + 3 payload bytes + delimiter`. The final delimiter is `}`.

| Kind | Frame | Bytes |
|---|---|---:|
| Register write | `{ W<A> , V<..> , V<..> }` | 16 |
| Register read | `{ R<A> }` | 6 |
| Register read *reply* | `{ v3 v2 v1 v0 }` — no opcode | 6 |
| Single pixel write | `{ W<A> , P<R,G,B> }` | 11 |
| Single pixel read | `{ R<row> , C<col> , P<..> }` | 16 |
| Burst write header | `{ I<..> , H<..> , W<..> }` | 16 |
| Burst read | `{ R<A> , H<..> , W<..> }` | 16 |
| Burst data | four pixels, **no opcodes** | 16 |

Opcode bytes: `W` `0x57`, `R` `0x52`, `V` `0x56`, `C` `0x43`, `P` `0x50`,
`I` `0x49`, `H` `0x48`.

Burst data is the exception — it carries no opcode bytes at all, which is why
the decoder must be told to expect it (see the opcode bypass section).

## 4. Single Pixel Write


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

## 5. Single Pixel Read


```text
{ R r2 r1 r0 , C c2 c1 c0 , P x x x }
```

The `P` group is a placeholder in the request; the FPGA replies with a frame of
the same shape carrying the pixel.

### Reply, byte for byte

| Byte | Value |
|---:|---|
| 0 | `{` `0x7B` |
| 1 | `R` `0x52` |
| 2 | `0x00` |
| 3 | `{6'b0, row[9:8]}` |
| 4 | `row[7:0]` |
| 5 | `,` `0x2C` |
| 6 | `C` `0x43` |
| 7 | `0x00` |
| 8 | `{6'b0, col[9:8]}` |
| 9 | `col[7:0]` |
| 10 | `,` `0x2C` |
| 11 | `P` `0x50` |
| 12 | red |
| 13 | green |
| 14 | blue |
| 15 | `}` `0x7D` |

Row and column are 10-bit values, big-endian across bytes 3–4 and 8–9 with the
top six bits of each pair zero. Byte 2 and byte 7 are always zero — the third
payload byte of each group is unused at this geometry.

## 6. Register write and read


```text
{ W 00 00 A , V v3 v2 v1 , V v0 x x }      register write, 16 bytes
{ R 00 00 A }                              register read,  6 bytes
```

The register address is 8 bits, carried in the low byte of the first group's
payload. A write's 32-bit value is split across the two `V` groups.

### Register read reply

The reply is 6 bytes and does **not** follow the `1 + 5n` request shape — there
is no opcode byte, just the 32-bit value big-endian between the braces:

| Byte | Value |
|---:|---|
| 0 | `{` `0x7B` |
| 1 | `value[31:24]` |
| 2 | `value[23:16]` |
| 3 | `value[15:8]` |
| 4 | `value[7:0]` |
| 5 | `}` `0x7D` |

Observed reading `IMG_STATUS` on a 256 x 256 build:

```text
7B 00 14 01 00 7D      ->  0x00140100
```

which decodes as height 256 in bits [9:0] and width 256 in bits [19:10].

| Offset | Register | Notes |
|---|---|---|
| `0x00` | `IMG_STATUS` | height [9:0], width [19:10], ready |
| `0x04` | `IMG_TX_MON` | complete/error, row, col — **read to clear** |
| `0x08` | `IMG_CTRL` | `start_img_read` |
| `0x0C` | `FIFO_STATUS` | full / empty / almost flags |
| `0x10` | `CLK_CTRL` | `clk_sel` |
| `0x14` | `PARITY_FAULT_CNT` | RX parity faults |

Registers are 4 bytes apart. The decode is by exact address, so an offset that
is not one of these reads back zero rather than aliasing onto a neighbour.

## 7. Burst Write header


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

## 8. Burst Data frame


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

## 9. Burst Read


```text
{ R a2 a1 a0 , H h2 h1 h0 , W w2 w1 w0 }
```

Requests an H×W rectangle starting at linear pixel address A. The FPGA replies
with `ceil(H*W/4)` burst-data frames in the same format as a burst write's
payload.

## 10. Burst addressing


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

## 11. Final partial frame


The number of required data frames is:

```text
ceil(H × W / 4)
```

When `H × W` is not divisible by four, the host fills the unused slots of the final frame with arbitrary padding pixels. The FPGA ignores all slots after the final real pixel and does not write or advance the address for them.

## 12. Burst mode and opcode bypass


Burst payload is arbitrary binary data. A red channel byte, for example, may equal the ASCII value of `W`, `R`, `P`, `{`, `}`, or `,`.

After accepting a Burst Write header, the decoder treats subsequent 16-byte messages as Burst Data frames without inspecting byte 1 as an opcode. The normal classifier is also gated during the burst, preventing payload bytes from triggering Register File or Single Pixel Write operations.

## 13. Command acceptance and backpressure


The Burst Write controller exposes a ready/valid command interface to the asynchronous command FIFO.

- A command is accepted only when `cmd_valid && cmd_ready` is true at a clock edge.
- While `cmd_ready` is low, address and pixel outputs remain stable.
- Burst position advances only after acceptance.
- The burst completes only after the final real command is accepted.

### Flow control

RTS/CTS is active in both directions and is exercised end to end.

- **Host to FPGA** — when an internal destination is busy, back-pressure
  propagates up the receive chain to `UART_CTS`, and the host stops sending.
- **FPGA to host** — when the host stops draining, the board's reply path fills
  and it deasserts its own RTS.

`flowtest.py` proves both, and includes a negative control: the same workload
repeated with flow control disabled, which is expected to lose data. If it
loses nothing, the workload never stressed the receive path.

Because back-pressure is honoured throughout, a burst upload during framebuffer
readback is no longer a hazard — the interlock defers whichever arrives
second.

## 14. Framebuffer readback


Framebuffer capture is initiated through the Register File command interface.
The FPGA serialises the complete 256 × 256 framebuffer and `image_tool.py`
reconstructs a PNG.

The read interlock rejects a duplicate request while a previous image transfer
remains marked in flight. Reading `IMG_TX_MON` (offset `0x04`) clears the
completion state and permits the next capture.

That read-to-clear is qualified by a genuine one-cycle read strobe. An earlier
design decoded it from the address alone, which forced whichever block drove
the address to park it at an unused value between accesses; that workaround has
been removed.

Internally the readback now uses AHB-Lite INCR4 bursts, four words of one
colour channel at a time, buffered per channel. None of that is visible on the
wire — the reply stream is unchanged.

## 15. Host utility examples


Every command needs the baud rate passed explicitly.

### Capture the framebuffer

```powershell
python image_tool.py --port COM5 --baud 8000000 snapshot out.png
```

### Upload a full image

```powershell
python image_tool.py --port COM5 --baud 8000000 load image.png
```

### Write a rectangle

```powershell
python image_tool.py --port COM5 --baud 8000000 rect 61 61 10 6 FF0000
```

Arguments are `row col height width colour`, the colour as hex RGB.

### Regression tests

```powershell
python board_test.py --port COM5 --baud 8000000 --width 256 \
       --init-red red_hex.mem --init-green green_hex.mem --init-blue blue_hex.mem

python flowtest.py  --port COM5 --baud 8000000 \
       --init-red red_hex.mem --init-green green_hex.mem --init-blue blue_hex.mem

python rgf_parking_test.py --port COM5 --baud 8000000 --width 256 --height 256
```

`board_test` and `flowtest` compare what they read against the `.mem` files, so
they must run **before** anything that writes to the framebuffer. Note that
memory contents come from the bitstream at configuration — pressing reset does
not restore them, only reprogramming does.

Images larger than 256 × 256 must be resized or cropped on the host before
upload. Smaller images replace only the corresponding top-left rectangle.

## 16. Error and status behavior


The design reports or latches status for conditions including:

- malformed or unsupported messages;
- command FIFO overflow;
- out-of-range pixel writes;
- duplicate framebuffer-read requests;
- image-transfer completion state.

Exact register addresses and board LED mappings are defined by the RTL packages and top-level integration and may be changed without altering the image-transfer frame formats documented here.
