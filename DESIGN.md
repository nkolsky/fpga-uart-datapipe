# DESIGN.md

# FPGA RGB Framebuffer Design Notes

## Project Overview

The project implements a complete FPGA image pipeline capable of receiving RGB pixel data over UART, storing it in an SRAM-backed framebuffer, and transmitting the framebuffer back to a host.

## Major Design Decisions

### Fixed framebuffer, variable transfers

The framebuffer is physically fixed at **256×256** pixels.

Burst Write supports variable image dimensions through the H and W header fields. The transfer always begins at the internally managed origin `(0,0)`.

This intentionally separates:

- physical framebuffer geometry
- transfer size
- host-side image preprocessing

### Valid-driven command arbitration

Command arbitration selects using command valid signals rather than a mode signal.

Benefits:

- avoids withdrawing a pending command
- preserves ready/valid semantics
- prevents races discovered during controller verification

### Burst controller

The controller guarantees:

- row-major traversal
- exactly one command per accepted pixel
- padding suppression
- stable outputs while stalled by backpressure

### Memory interlock

Reads are deferred until:

- no pending writes
- no burst active
- no write controller busy

This prevents SRAM read/write conflicts.

## Verification Strategy

The design was verified in three stages:

1. Unit-level module regressions
2. End-to-end simulation
3. FPGA hardware validation

Regression coverage included:

- parser correctness
- malformed packets
- FIFO backpressure
- burst completion
- command arbitration
- integration
- hardware readback

## Hardware Validation

Hardware testing confirmed:

- Single Pixel Write
- Burst Write
- Image readback
- Legacy register commands
- Repeated capture without reprogramming

## Repository Structure

```text
Sources/
    RTL

testbenches/
    Simulation

Scripts/
    Host-side utilities

Constraints/
    FPGA constraints
```

## Suggested Future Extensions

- Arbitrary burst origin
- Dynamic host-side resizing
- CTS flow control
- Configurable framebuffer dimensions
- Timeout / abort support
- Additional diagnostics

