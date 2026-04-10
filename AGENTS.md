# Repository Guidelines

## Project Structure & Module Organization
This repository contains bare-metal firmware for the Nuvoton M2003 motor controller. Keep application code in `src/` and public headers in `include/`, usually as matched pairs such as `src/protocol.c` and `include/protocol.h`. Vendor and platform support live in `CMSIS/`, `Library/StdDriver/`, and `RTT/`; treat those as third-party code unless a hardware fix requires a targeted patch. Use `scripts/` for host-side utilities such as `build-jlink.ps1`, `flash-jlink.ps1`, `ring_tool.py`, and `tune_tool.py`. Keep reference material and protocol notes in `docs/` and `protocol.md`.

## Build, Test, and Development Commands
Use GNU Make with the ARM GCC toolchain installed in `PATH`.

- `make`: build `build/m2003-motor.bin` and print image size.
- `make build-jlink`: build firmware and generate `build/m2003-motor.jlink`.
- `powershell -File scripts/build-jlink.ps1`: Windows-friendly wrapper around `make build-jlink`.
- `powershell -File scripts/flash-jlink.ps1`: flash the target with SEGGER J-Link and verify readback when available.
- `make clean`: remove the `build/` directory.

## Coding Style & Naming Conventions
Follow the existing C style: 4-space indentation, opening braces on the next line for functions and control blocks, and concise block comments only where the control flow is non-obvious. Use `lower_snake_case` for functions and file names, `UPPER_SNAKE_CASE` for macros and register-related constants, and keep module state `static` unless it is part of a deliberate interface. Prefer adding functionality to the existing module boundary instead of growing `main.c`.

## Testing Guidelines
There is no committed unit-test framework yet, so a clean `make` build is the minimum gate for every change. For protocol, control-loop, or tuning work, run bench validation against hardware with `scripts/ring_tool.py` or `scripts/tune_tool.py` and record the command used, for example `python scripts/tune_tool.py -p COM7 0 --velocity 500 --csv step.csv`. Include any relevant UART logs, plots, or measured results in the change notes.

## Commit & Pull Request Guidelines
Match the existing commit history: short, imperative summaries such as `fix direction change bug` or `persistent parameter store`. Keep each commit focused on one firmware concern. Pull requests should state the hardware used, list build and bench-test steps, call out protocol or flash-layout changes, and attach evidence for motor-behavior changes such as captures, CSV output, or scope screenshots.
