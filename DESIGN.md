# Design notes

Why this design looks the way it does. The wire protocol is in
[PROTOCOL.md](PROTOCOL.md); the module inventory is in [README.md](README.md).
This file is the reasoning.

---

## 1. Two clock domains

| Domain | Clock | Owns |
|---|---|---|
| 256 MHz | `pll_clk_out` | UART protocol — framing, parity, classification |
| 100 MHz | `CLK100MHZ` | image geometry, addresses, words |

`clk_wiz_0` derives 256 MHz from the board's 100 MHz with D=5, M=48 and a
CLKOUT0 divide of 3.75. A divide-by-32 bit period then gives exactly 8 Mbaud.

The 3.75 divide is **fractional**, which only CLKOUT0 supports and which costs
221 ps of peak-to-peak jitter and 302 ps of phase error. Vivado accounts for
both, so they are part of why the 256 MHz timing margin is thin. There is no
clean integer path to 256 MHz; there is one to 280 (M=42, D=5, divide 3), which
matters if that frequency is ever attempted.

### Every crossing is at chip_top

A clock crossing is interconnect *between* two subsystems, so it belongs with
the interconnect rather than inside either side. All of them are instantiated
at `chip_top`:

| Crossing | Carries |
|---|---|
| `cdc_msg_sync` | whole messages, 256 → 100, two-phase req/ack |
| `async_fifo` ×3 | image words, 100 → 256, one per colour |
| `cdc_cmd_sync` ×3 | pixel, burst and register replies |
| `cdc_level_sync` ×3 | FIFO flags, `burst_active` |
| `cdc_pulse_sync` ×4 | accepts, `tx_img_done`, parity fault |

Both clocks come from the same MMCM, so Vivado treats them as *related* and
will time every crossing synchronously unless told otherwise. At the 64:25
ratio the tightest launch/capture relationship is 0.156 ns — unachievable. The
XDC therefore declares them asynchronous, and every crossing is handled
structurally instead.

That constraint is load-bearing. It is written **without `-quiet`**: if a
lookup fails, the group silently empties and every crossing goes back to being
timed, producing a wall of impossible violations with nothing pointing at the
cause. A loud failure is better. It is also resolved off the clk_wiz wrapper
*pin* rather than by clock name, because the generated clock does not exist
when the XDC is parsed.

---

## 2. Two buses, two address spaces

Register traffic runs over **APB**; image memory over **AHB-Lite**. They are
independent — a message reaches one or the other, chosen by kind, so the two
spaces never collide.

```
APB   PADDR[15:8]  aperture       RGF at 0x00
      PADDR[7:0]   register offset

AHB   HADDR[17:16] channel        R 0x0_0000  G 0x1_0000  B 0x2_0000
      HADDR[15:2]  word index     16384 per channel
```

### Where each fabric is instantiated

`apb_master` and `apb_bar` are at `chip_top`, beside the CDC primitives, for
the same reason those are: they are interconnect. Only `apb_slave_rgf` sits
inside `register_subsystem`, because it is the part that translates to `rgf`'s
private port.

The AHB fabric is entirely inside `memory_subsystem`, because every one of its
clients and all three slaves already live there. Lifting it out would gain
nothing.

### Default responders

Both decoders answer an unmapped address rather than ignoring it. Without a
default responder no slave drives ready, the master waits forever, and a single
bad address stalls the whole link permanently. The AHB version implements the
protocol-required **two-cycle** error: `HRESP = ERROR` with `HREADY` low, then
`HRESP = ERROR` with `HREADY` high.

---

## 3. The read path

`img_burst_reader` drains the three SRAMs into three channel FIFOs using
per-channel INCR4 bursts, round-robin.

```
burst 1   R SRAM, words N..N+3   ->  4 entries into the R FIFO
burst 2   G SRAM, words N..N+3   ->  4 entries into the G FIFO
burst 3   B SRAM, words N..N+3   ->  4 entries into the B FIFO
then N += 4
```

### Why three FIFOs

This is the question the design answers most directly. A single-manager bus
cannot read three SRAMs at once — accesses serialise. An INCR4 therefore covers
four words of **one** channel, so red runs up to two bursts ahead of blue and
each colour needs its own buffer.

The predecessor read all three SRAMs in parallel at one address and needed only
one 24-bit FIFO. Measured skew under the current design is **4 entries**, a
full burst, which is the empirical proof that one FIFO would no longer suffice.

### Why an arbiter and not a counter

R, G, B could be a fixed rotation. It is a `round_robin_arbiter` because a
channel whose FIFO is nearly full must be able to **drop out** while the others
continue; otherwise one slow channel stalls the whole drain. A channel requests
only when its FIFO has room for a whole burst, and the arbiter's lock holds the
grant so a burst is never cut in half.

### The n = 3 pointer trap

`PTR_W` is `$clog2(N)`, so N=3 gives a 2-bit pointer with range 0..3. Left to
roll over naturally it would visit 3, and for a 3-bit vector `{2{req}} >> 3` is
identical to `>> 0` — requester 0 would get two turns in every four while 1 and
2 got one each. The pointer therefore wraps at **N-1**, not at its natural
maximum. This only matters when N is not a power of two, which is exactly this
case.

---

## 4. The write path

### Full-image writes only

One burst message carries 4 pixels, which de-interleave into exactly **one word
per channel**. An INCR4 needs four consecutive words of one channel, so
`img_burst_writer` gathers four messages before issuing anything.

Two things make full-image the only safe case:

- **AHB-Lite has no byte strobes.** A partial word cannot be expressed at all.
- **`pixel_word_packer` emits partial words** at the end of every row of a
  narrower rectangle, because the linear address jumps by `IMG_WIDTH` between
  rows. Only a full-width rectangle produces complete words at consecutive
  addresses.

`burst_mode` is decided once, before the rectangle starts, from the header
geometry: base 0, height and width equal to the image. Everything else — single
pixels, offset or short rectangles — keeps the direct port and its byte
enables.

### One master, two engines

`mem_interlock` already makes reads and writes mutually exclusive in both
directions:

```systemverilog
wr_port_grant = !read_active && !pix_rd_active
read_go       = start_pending && !wr_pending && ...
```

so the read and write burst engines share one `ahb_master` through a plain mux.
No second arbiter is needed. This was verified in the RTL rather than assumed.

---

## 5. The IDLE_ADDR hazard, and its removal

`rgf`'s `IMG_TX_MON` read-to-clear was originally a level decode with no access
qualifier: any block driving that address cleared the register, whether or not
a read was happening. Whoever drove the address therefore had to *park* it at
an unused value between accesses.

That obligation had already migrated between modules once. `apb_slave_rgf`
supplies a genuine one-cycle `pc_ren`, so the hazard is **deleted rather than
relocated** and the parking is gone.

The regression test for it now passes for a different reason than it was
written for. It was built to prove the parking worked; it now proves the
parking is unnecessary. Same green result, different mechanism.

---

## 6. PPA: what the buses cost, and what they buy

Stated plainly because the honest answer is not the flattering one.

**The write path is not throughput-bound.** A burst message arrives every
22 µs; an INCR4 takes about 50 ns. The bus is roughly **0.2% utilised**. No bus
choice can buy throughput here.

| Option | Extra flops | Throughput |
|---|---:|---|
| Direct writes, no bus | 0 | baseline |
| AHB `SINGLE` transfers | ~45 | unchanged |
| AHB `INCR4` with gather | ~95 | unchanged |

`SINGLE` is equally protocol-conformant and about 50 flops cheaper, because it
needs no gather. INCR4 was implemented because the specification asks for it on
the full-image path. That is a legitimate reason; it is not a performance one.

Where the design *is* efficient:

- The **read** slaves cost zero flops. Their only registers are the write-side
  `wr_pending` and `wr_addr_q`, which tie off when the write port is unused, so
  Vivado absorbs what remains into the surrounding mux.
- One shared master rather than two.
- One shared write-data bus with three enables, rather than three buses.
- The rotate-mask arbiter scales with N; a unique-case version needs a
  hand-written priority chain per pointer value.

### Measured cost

| Module | Predicted | Actual |
|---|---:|---:|
| `ahb_master` | ~25 FF | 27 FF |
| `ahb_decoder` | 5 FF | 5 FF |
| `img_burst_writer` | ~25 FF + buffers | 25 FF + 64 LUTRAM |

The 384-bit gather went to LUTRAM rather than flops. The three channel FIFOs
also moved to LUTRAM when their depth dropped to 16, freeing the `RAMB18` the
single 64-deep FIFO had used.

---

## 7. Timing

| | |
|---|---|
| WNS | 0.022 ns |
| WHS | 0.024 ns |
| Failed routes | 0 |
| Total power | 0.274 W |
| Critical path | `rx_classifier` burst-extent compare, 256 MHz |

The critical path has been in `rx_classifier` throughout — a wide comparison
computed combinationally in one stage, five logic levels with two carry chains.
Splitting it across another pipeline stage is the fix, and it is also what
stands between this design and 280 MHz.

### A measured trade-off worth recording

`ASYNC_REG` marks a synchroniser chain so the placer keeps its flops adjacent
and synthesis will not retime or merge them. It costs no resources — but it is
not free.

Adding it to `rx_phy`'s three-stage input synchroniser moved WNS from
**+0.078 ns to −0.052 ns**. Pinning three flops removed placement freedom in
`rx_subsystem`, which is where the critical path lives.

It was therefore **left off that one chain** and applied to the other five. The
reasoning: `rx_phy`'s chain has three stages where the others have two, so it
has the most inherent settling margin and is the cheapest to leave unpinned.
Vivado reports it as CDC-2; that is accepted, not overlooked, and the RTL says
so at the declaration.

---

## 8. Known synthesis reports

**`SYNTH-15` — byte-wide write enable not inferred, ×48.** Informational.
Vivado declines the BRAM's dedicated byte-write pins because the address width
of 14 exceeds its threshold of 12, and falls back to one write enable per RAM.
Byte lanes remain independently writable — an unaligned rectangle write
(`rect 61 61 10 6`, two partial words per row) reads back correctly on
hardware. `ram_decomp = power` is the documented override if the dedicated
pins are ever wanted.

**`TIMING-4` / `TIMING-27`, 2 critical warnings.** The Clocking Wizard's
in-context XDC defines a clock on the IP's own `clk_in1` pin. The design's
`sys_clk_pin` overrides it; both describe the same 100 MHz net at the same
period, so the resolution is correct.

**`CDC-1` / `CDC-4`, 52 of 53 criticals.** All are `heartbeat_cnt` inside
`clocking_subsystem` — bit-to-bit paths *within one counter* that is clocked
from the `BUFGCTRL` mux output, which is reachable from both clocks. The
counter is single-domain at any instant; static analysis cannot express that.
Its only load is an LED.

**`CDC-13`, the remaining critical.** `clk_sel` into the BUFGCTRL select. The
primitive exists to accept an asynchronous select and switch break-before-make.

---

## 9. Verification

Ten self-checking testbenches, **187 checks**. Key paths are mutation-tested: a
deliberate bug is injected and the suite must fail. A test that passes against
a mutant is not testing what it claims to.

Two mutation results worth keeping:

- Reverting the router's `dest_ready` to always-high does not merely fail a
  check — it trips `apb_master`'s own assertion, *"request issued while the
  master was busy"*. That is the silent message loss the back-pressure
  prevents, caught by the design's own guard.
- Removing the decoder's default responder causes a simulation **timeout**, not
  a wrong answer. That is the deadlock, demonstrated.

### Things the testbenches got wrong first

Recorded because they generalise:

- An assertion that held only at zero wait states passed every test until a
  slow slave was modelled. `$past(haddr)` is the *previous cycle's* address,
  not the previous *beat's* — under back-pressure they differ.
- Counting events proves completion; protocol compliance often lives in the
  waveform *shape*. A one-cycle error response completes with the same pulse
  count as a legal two-cycle one, so only counting `HREADY`-low cycles
  distinguishes them.
- A path that is never stressed is never tested. A mutation ignoring
  back-pressure escaped until the consumer model stalled long enough for
  `almost_full` to actually assert.

### Hardware

| Test | Covers |
|---|---|
| `board_test` | protocol, memory contents against `.mem` |
| `flowtest` | RTS/CTS under stall, with a negative control |
| `image_tool snapshot` / `load` | full read and write paths |
| `image_tool rect 61 61 10 6` | partial-word writes via the direct port |
| `rgf_parking_test` | register path and the `IMG_TX_MON` interlock |

`flowtest`'s negative control repeats the same workload with flow control
disabled and is *expected* to lose data. If it loses nothing, the workload
never stressed the receive path and the tests above it are untested rather than
passed.

---

## 10. Limitations

- AHB bursts carry full-image transfers only; smaller rectangles and single
  pixels use the direct port
- A full-image read always begins at (0,0)
- No burst timeout or abort
- Geometry is fixed at synthesis
- `image_tool --mode stream` is not functional

## 11. Future work

- Another pipeline stage in `rx_classifier`, which buys timing margin and is
  the prerequisite for 280 MHz
- Configurable geometry
- Burst recovery and timeout
- A shared host-side config module, so the baud rate is not passed by hand
- Consider the shared-pointer FIFO: three memories with one commit pointer
  would save roughly 80 control flops over three independent instances. Not
  done, because three instantiations of an already hardware-validated module
  need no new verification.
