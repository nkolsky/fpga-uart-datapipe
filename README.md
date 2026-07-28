# FPGA RGB Framebuffer with UART Burst Write

> SystemVerilog implementation of an FPGA-based RGB framebuffer supporting **Single Pixel Write**, **Burst Write**, **image readback**, and **legacy register commands**, verified through unit tests, end-to-end simulation, and hardware validation.

## Features

- RGB framebuffer (256×256)
- UART command protocol (8E1)
- Single Pixel Write
- Burst Write (arbitrary H×W up to 256×256)
- Legacy RGF command path
- Image readback
- Asynchronous command FIFO
- Write/read interlock
- Python utilities for upload, capture, and image rotation

## Architecture

```text
               UART RX
                  │
               rx_phy
                  │
           rx_msg_decode
                  │
        ┌─────────┴─────────┐
        │                   │
  rx_classifier      Burst Write
        │                   │
        └─────────┬─────────┘
                  │
          Command Arbitration
                  │
            Async Command FIFO
                  │
            SRAM Write Control
                  │
         RGB Framebuffer SRAM
                  │
            ROM Sequencer
                  │
               UART TX
```

## Repository Layout

```text
Constraints/
Scripts/
Sources/
testbenches/
chip_top.bit
README.md
DESIGN.md
```

## Simulation

Major regressions:

- Burst header parser
- Burst data parser
- Burst controller
- RX classifier / pipeline
- Memory interlock
- Stage 2C end-to-end pipeline
- Stage 3 Burst Write integration

All regressions passed before hardware validation.

## Hardware Workflow

1. Program FPGA.
2. Reset once.
3. Run framebuffer capture.
4. Test Single Pixel Write.
5. Capture and verify.
6. Upload Burst Write image.
7. Capture and compare.

## Python Utilities

| Script | Purpose |
|---------|---------|
| `lab10_capture.py` | Capture framebuffer |
| `test_single_pixel_write.py` | Write a single RGB pixel |
| `test_burst_write.py` | Upload an arbitrary H×W image |
| `rotate_and_send.py` | Rotate then upload an image |

## Burst Write

- Framebuffer size: **256×256**
- Burst origin: **(0,0)** (managed internally by FPGA)
- Header supplies **H** and **W**
- Rectangle is written row-major into SRAM.

## Current Limitations

- Fixed framebuffer geometry
- Burst origin fixed at (0,0)
- CTS backpressure not yet implemented
- Burst timeout/abort not implemented
- Full image readback always begins at (0,0)

## Future Work

- Configurable burst origin
- Automatic host-side image scaling
- CTS flow control
- Configurable framebuffer geometry
- Burst recovery/timeout

