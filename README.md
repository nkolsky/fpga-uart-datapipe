# FPGA RGB Framebuffer over UART

> SystemVerilog implementation of a 256×256 RGB framebuffer on a Nexys A7-100T,
> exchanged with a host over UART with end-to-end flow control. Register access
> runs over **APB**; image memory runs over **AHB-Lite** with INCR4 bursts.
> Verified by unit tests, mutation testing, and hardware validation.
>
> [PROTOCOL.md](PROTOCOL.md) documents the wire format;
> [DESIGN.md](DESIGN.md) explains the design decisions.

## Project history

Built across a two-semester digital design course. The commit history starts
partway through, at the APB and AHB-Lite work — version control was introduced
later in the course, so everything before that point was developed without it.

What predates the first commit: the UART physical and MAC layers, the message
protocol and its classifier, both clock-domain crossings, the SRAM write path
with byte enables, the framebuffer readback, and the register file. What the
history covers: the register path moving to APB, image memory moving to
AHB-Lite with INCR4 bursts, the three-FIFO restructure that followed from it,
and the documentation and diagrams.

## Features

- 256×256 RGB framebuffer, one SRAM per colour channel
- UART at 8 Mbaud, 8 data bits, even parity, 1 stop, RTS/CTS both directions
- Single pixel write and read
- Burst write and burst read over an arbitrary H×W rectangle
- Full-image write and full-image read
- Register file over APB
- Image memory over AHB-Lite, INCR4 bursts, round-robin across R/G/B
- Two clock domains with every crossing handled explicitly
- Python host tools for upload, capture and regression testing

## Clocking

| Domain | Clock | Owns |
|---|---|---|
| 256 MHz | `pll_clk_out` | UART protocol — framing, parity, classification |
| 100 MHz | `CLK100MHZ` | image geometry, addresses, words |

`clk_wiz_0` generates 256 MHz from the board's 100 MHz: D=5, M=48, CLKOUT0
divide 3.75. At 256 MHz with a divide-by-32 bit period that gives exactly
**8 Mbaud**.

Every clock crossing is instantiated at `chip_top`, not inside a subsystem.
[DESIGN.md](DESIGN.md) lists them and explains why.

## Buses

Two independent buses with separate address spaces. A message reaches one or
the other, chosen by kind, so the two never collide.

**APB — register file.**

```
PADDR[15:8]  aperture      RGF at 0x00
PADDR[7:0]   register offset
```

**AHB-Lite — image memory.**

```
HADDR[17:16]  channel      R 0x00000   G 0x10000   B 0x20000
HADDR[15:2]   word index   16384 per channel
```

Unmapped addresses are answered by a default responder rather than left to
hang. [DESIGN.md](DESIGN.md) covers where each fabric is instantiated and why.

## Architecture

![Data pipe flow](data_pipe_flow.png)

Left is the UART side at 256 MHz, right is the memory side at 100 MHz, and the
purple blocks in the middle are the clock crossing. Data enters at `rx_phy`,
is framed into whole messages, crosses once, and `mem_msg_router` dispatches
each message by kind — write path, a reader, or the register path.

`tx_sequencer` and `tx_reply_ctrl` are **parallel** sources into one mux
(`reply_drives_mac`); neither passes through the other. Image pixels return
through the three channel FIFOs, replies through a separate narrower crossing.

![Block diagram](block_diagram.png)

The same design by module hierarchy. Worth noting where each bus sits: the APB
master and BAR are at `chip_top` next to the CDC primitives, and only
`apb_slave_rgf` is inside `register_subsystem`, because it is the only part
that touches `rgf`'s private port. The AHB fabric is entirely inside
`memory_subsystem`, where all of its clients already live.

## Register file

| Offset | Register | Notes |
|---|---|---|
| `0x00` | `IMG_STATUS` | geometry, ready |
| `0x04` | `IMG_TX_MON` | complete/error flags, row/col — **read to clear** |
| `0x08` | `IMG_CTRL` | `start_img_read` |
| `0x0C` | `FIFO_STATUS` | full / empty / almost flags |
| `0x10` | `CLK_CTRL` | `clk_sel` |
| `0x14` | `PARITY_FAULT_CNT` | RX parity faults |

`IMG_TX_MON`'s read-to-clear is qualified by a one-cycle `pc_ren` from
`apb_slave_rgf`. An earlier design decoded it from the address alone, which
forced the message router to park at an unused address between accesses; that
workaround is gone.

## Repository layout

```text
Constraints/            Nexys-A7-100T-Master.xdc
Scripts/                Python host tools
Sources/                SystemVerilog RTL
testbenches/            sixteen that build and pass; legacy/ holds earlier ones
block_diagram.drawio    module hierarchy, by clock domain
block_diagram.png       exported, embedded in this README
data_pipe_flow.drawio   how data moves, and where it crosses clocks
data_pipe_flow.png      exported, embedded in this README
chip_top.bit
README.md
DESIGN.md               why the design is shaped the way it is
PROTOCOL.md             the wire format, message by message
```

The `.drawio` files are the editable sources; the `.png` files beside them are
what the README embeds above. Re-export both after changing either diagram, or
the README will quietly go stale.

## Simulation

Sixteen self-checking testbenches, **361 checks**, all passing. Key paths are
mutation-tested: a deliberate bug is injected and the suite must fail.

| Testbench | Checks | Covers |
|---|---:|---|
| `tb_apb_slave_rgf` | 18 | APB slave, `pc_ren` strobe |
| `tb_apb_master` | 21 | APB FSM, SETUP/ACCESS phases |
| `tb_apb_bar` | 28 | decode, default responder, deadlock guard |
| `tb_apb_integration` | 17 | router → master → BAR → slave → rgf |
| `tb_ahb_slave_sram` | 16 | two-phase write capture, BUSY handling |
| `tb_ahb_master` | 25 | INCR4 sequencing, wait states, HRESP |
| `tb_ahb_decoder` | 24 | data-phase response mux, two-cycle error |
| `tb_round_robin_arbiter` | 16 | fairness at N=3, lock, no preemption |
| `tb_burst_read_path` | 13 | full read path, back-pressure, channel skew |
| `tb_burst_write_path` | 9 | full write path, gather, equivalence |
| `tb_mem_write_path` | 44 | write subsystem, byte enables, rectangles |
| `tb_rx_mac` | 42 | frame assembly, byte indexing, recovery |
| `tb_pixel_word_packer` | 33 | 4 px → 1 word, partial-word flush |
| `tb_mem_msg_writer` | 28 | message → packer geometry |
| `tb_mem_interlock` | 21 | read/write exclusion, request/grant |
| `tb_cdc_msg_sync` | 6 | 256 → 100 message crossing |

`tb_burst_write_path` must be built with **`-DSIMULATION`** — `memory_pkg`
switches to an 8×8 image under that define, and without it the packer strides a
full 256-pixel row between rectangle rows.

### testbenches/legacy

Fourteen earlier testbenches sit in `testbenches/legacy/`. They were written
against interfaces that have since changed — the message types and
`rx_msg_parser`'s port list were reworked when the RX path was restructured, so
they no longer compile. Reviving them would mean rewriting against the current
types rather than patching, so they are kept for reference, not as regression.

### What is covered how

Every module is exercised by the hardware regression on every run. Coverage
differs in kind, not in whether it exists:

| Level | Modules |
|---|---|
| **Unit**, directed simulation | both bus fabrics, the arbiter, `mem_interlock`, `mem_msg_writer`, `pixel_word_packer`, `rx_mac`, `cdc_msg_sync` |
| **Path**, simulation | `async_fifo` ×3, both burst engines, `mem_write_subsystem`, `mem_msg_router`, `rgb_sram` |
| **System**, hardware | the whole design — RX and TX chains, direct read controllers, all ten remaining CDC primitives |

Blocks are driven through the chain they belong to rather than in isolation.
`tb_burst_read_path` runs all three channel FIFOs across the clock crossing
under back-pressure and measures the 4-entry skew between them — a property of
the read path as a whole, which no single-module test would see.

The hardware regression runs millions of cycles at real timing. `flowtest`'s
negative control confirms the workload stresses the link: 7,488 of 8,000
replies lost with flow control disabled.

The next step is a UVM environment, adding functional coverage and
constrained-random stimulus on top of the directed tests here.

## Hardware workflow

**Memory contents come from the bitstream, not from reset.** `INIT_FILE`
values are loaded at configuration; `CPU_RESETN` resets logic only. Any test
that compares against the `.mem` files must run before anything that writes, or
after reprogramming.

### One command

```powershell
cd Scripts
python final_test.py --port COM5 --image photo.png
```

Nine stages covering every pipeline, and it **restores the image it started
with** — stage 1 captures the framebuffer before writing anything and stage 8
loads it back, verifying every pixel. That makes it safe to run repeatedly
without reprogramming.

Pass `--image` to exercise stage 6, the full-image write. Without it that stage
is skipped, and the AHB INCR4 burst write path is not covered — stage 3 tests
single pixels and stage 4 tests partial words, both of which use the direct
port. Any 256 × 256 PNG will do.

It pauses once, in stage 7: the negative control deliberately overruns the
receive path, so the board needs a reset before the restore. Press RESET, then
Enter.

### Or stage by stage

1. Program the FPGA
2. Press reset
3. `board_test` — protocol and memory, compares against `.mem`
4. `flowtest` — flow control under stall, compares against `.mem`
5. `image_tool snapshot` — full read path
6. `image_tool load` — full write path
7. `image_tool rect` — partial-word writes via the direct port
8. `rgf_parking_test` — register path

Steps 3–8 are what `final_test` runs in one pass.

## Python utilities

`final_test.py` defaults to the correct baud rate; every other script needs it
passed explicitly.

```bash
python final_test.py       --port COM5 --image photo.png
python board_test.py       --port COM5 --baud 8000000 --width 256 \
                           --init-red red_hex.mem --init-green green_hex.mem \
                           --init-blue blue_hex.mem
python flowtest.py         --port COM5 --baud 8000000 \
                           --init-red red_hex.mem --init-green green_hex.mem \
                           --init-blue blue_hex.mem
python image_tool.py       --port COM5 --baud 8000000 snapshot out.png
python image_tool.py       --port COM5 --baud 8000000 load photo.png
python image_tool.py       --port COM5 --baud 8000000 rect 61 61 10 6 FF0000
python rgf_parking_test.py --port COM5 --baud 8000000 --width 256 --height 256
```

| Script | Purpose |
|---|---|
| `final_test.py` | all nine stages in one run, restores the image afterwards; needs `--image` for stage 6 |
| `image_tool.py` | snapshot, load, single pixel, rectangle |
| `board_test.py` | protocol and memory regression |
| `flowtest.py` | RTS/CTS under stall, with a negative control |
| `rgf_parking_test.py` | register path and the `IMG_TX_MON` interlock |

`final_test.py` imports `flowtest.py` as a module, so both must sit in the same
directory. Run it from `Scripts/`.

`flowtest` test 6 is a **negative control**: it repeats the same workload with
flow control disabled and is expected to lose data. If it loses nothing, the
workload never stressed the receive path and the tests above it are untested
rather than passed.

## Implementation results

| | |
|---|---|
| Device | xc7a100t-csg324-1 |
| WNS | 0.131 ns |
| WHS | 0.022 ns |
| Failed routes | 0 |
| Total power | 0.275 W |
| Critical path | `rx_classifier` burst-extent compare, 256 MHz domain |

### Implementation directives are required

The design does **not** close timing with Vivado's defaults. The same RTL gives
WNS −0.064 ns with default directives and +0.131 ns with these:

```tcl
set_property STEPS.PLACE_DESIGN.ARGS.DIRECTIVE ExtraTimingOpt [get_runs impl_1]
set_property STEPS.POST_ROUTE_PHYS_OPT_DESIGN.IS_ENABLED true [get_runs impl_1]
set_property STEPS.POST_ROUTE_PHYS_OPT_DESIGN.ARGS.DIRECTIVE AggressiveExplore [get_runs impl_1]
```

The critical path is `rx_classifier`'s burst-extent compare;
[DESIGN.md](DESIGN.md) has the analysis. These settings live in the `.xpr`
rather than in this repository, so recreating the project loses them.

Two critical warnings remain, both from the Clocking Wizard's in-context XDC
defining a clock on its own input pin. The design's `sys_clk_pin` overrides it;
both describe the same 100 MHz net at the same period.

`SYNTH-15` (byte-wide write enable not inferred) is informational — byte lanes
remain independently writable, which the unaligned-rectangle test confirms.

## Current limitations

- AHB bursts are used for full-image transfers only; smaller rectangles and
  single pixels use the direct port
- Burst origin for a full-image read is fixed at (0,0)
- No burst timeout or abort
- Fixed 256×256 geometry, set at synthesis
- `image_tool --mode stream` is not functional; the decoder assumes burst format

## Future work

- Configurable framebuffer geometry
- Burst recovery and timeout
- 280 MHz UART domain — attempted and measured: WNS −0.227 ns, three failing
  paths, all in `rx_classifier`. Pipelining it is the fix, but `rx_mac` sits
  only 30 ps behind and would need the same treatment
- A shared `board_config.py` so the baud rate is not passed by hand