# Three-Body-Problem ASIC

A digital ASIC implementing a real-time three-body gravitational simulation with 640×480 VGA output, designed in Verilog for the [Tiny Tapeout](https://tinytapeout.com/) shuttle.

The design simulates three planets (A, B, C) attracting each other through a simplified gravity model and renders them in real time on a VGA display, complete with a perspective grid, collision detection, and pseudo-random respawn.

## Authors

- **Jan Rapp**
- **Henri Schulz**

## Features

- **640×480 @ 60 Hz VGA output** following the Tiny Tapeout VGA Pmod pinout
- **Three-body gravitational simulation** with Manhattan-distance gravity model
- **Collision detection** with flash animation and LFSR-based pseudo-random respawn
- **Perspective grid background** with depth-based planet sizing (4 size zones)
- **Wrap-around boundaries** on all four edges of the screen
- **Depth-sorted rendering** of the three bodies (back-to-front)

## Architecture

The design is split into three main modules:

| Module | Purpose |
|---|---|
| `hvsync_generator` | VGA timing — horizontal/vertical sync and active-video signals |
| `gravity` | Physics integration, collision detection, LFSR, edge wrapping |
| `renderer` | Pixel pipeline — grid lines, planet hit-tests, depth-sorted color mux |

Physics calculations are performed **during VGA blanking intervals** using a time-multiplexed micro-scheduler. A single 4-bit `step` counter sequences through all six pair interactions (A↔B, A↔C, B↔C) and both axes (X, Y), enabling gate-shared computation of distance, gravity, and collision per pair.

## Design Decisions & Optimizations

The chip targets a single Tiny Tapeout tile, so area and gate count were the dominant constraints. Key optimizations:

- **Fixed-point arithmetic** throughout — no floating-point hardware
- **Manhattan distance** instead of Euclidean, avoiding multipliers and square roots
- **Shift-add multiplications** for the perspective grid offsets (e.g. `d*180`, `d*327`, `d*480`, `d*608`) instead of using `*`
- **Time-multiplexed scheduler** — one physics engine shared across all three bodies and both axes, run during blanking
- **Bit-slice-derived indexing** for the pair scheduler — replaces a mux chain with direct combinational logic on the step counter
- **XOR overflow detection** for saturating velocity addition
- **Narrow-bit comparators** for distance thresholds (e.g. `manh < 12` decoded from upper bits + nibble pattern instead of a full 11-bit comparator)

## Inputs / Outputs

| Signal | Pin | Description |
|---|---|---|
| `clk` | — | 25.175 MHz clock (standard Tiny Tapeout VGA clock) |
| `rst_n` | — | Active-low reset |
| `uo_out[0]` | R[1] | VGA red bit 1 |
| `uo_out[1]` | G[1] | VGA green bit 1 |
| `uo_out[2]` | B[1] | VGA blue bit 1 |
| `uo_out[3]` | vsync | VGA vertical sync |
| `uo_out[4]` | R[0] | VGA red bit 0 |
| `uo_out[5]` | G[0] | VGA green bit 0 |
| `uo_out[6]` | B[0] | VGA blue bit 0 |
| `uo_out[7]` | hsync | VGA horizontal sync |

Compatible with the [Tiny Tapeout VGA Pmod](https://github.com/mole99/tiny-vga).

## How to Run

1. Connect a VGA Pmod to the Tiny Tapeout demo board
2. Provide a 25.175 MHz clock on `clk`
3. Pulse `rst_n` low to initialize
4. Three planets appear on screen and start orbiting under mutual gravity
5. On collision, the screen flashes and the bodies respawn with pseudo-random positions and velocities

## Status

Design submitted to the Tiny Tapeout shuttle — **currently in fabrication**.

## License

Apache-2.0. See the SPDX header in each source file.

The project is based on the [Tiny Tapeout template](https://github.com/TinyTapeout/tt_um_template) by Uri Shaked. All Verilog logic in `tt_um_Jan_three_body_solution`, `gravity`, and `renderer` was written by the authors listed above.
