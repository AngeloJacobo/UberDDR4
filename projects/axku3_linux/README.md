# Linux on AXKU3 with UberDDR4

A small RISC-V Linux system with UberDDR4 as its main-memory controller.
VexRiscv executes the software, LiteX supplies the SoC interconnect and
peripherals, and UberDDR4 drives the board's external DDR4.

The demonstrated configuration is DDR4-2400, a 300 MHz CPU, and 1 GiB of
CPU-visible RAM. It boots to a Buildroot shell over USB-UART. No Ethernet,
SD card or additional board hardware is needed.

**Status:** based on a working hardware demonstration. This cleaned-up version
uses unchanged controller RTL and still needs a fresh bitstream/hardware check.
Startup BIST is disabled; earlier BIST failures and one Linux file-hash mismatch
remain unresolved. See [RESULTS.md](RESULTS.md) before relying on it.

## Where to start reading

The hardware is mainly in **axku3_uberddr4.py**; most other files automate builds
and tests. You do not need to understand all of them to use the example.

| Files | What they do |
| --- | --- |
| `axku3_uberddr4.py` | Clocks/reset, CPU, memory map and Wishbone connection to UberDDR4 |
| `axku3_platform.py`, `constraints/*.xdc` | AXKU3 pins, I/O standards and board constraints |
| `setup.ps1`, `build.ps1`, `resume_implementation.ps1` | User commands for setup, generation, synthesis and routing |
| `prepare_linux_payload.ps1`, `boot.ps1`, `console.ps1` | Prepare images, load/test Linux, then type into its shell |
| `environment.ps1`, `local.example.ps1`, `check_environment.ps1` | Local paths and prerequisite checks |
| `clean.ps1` | Remove local generated files; preview with `-WhatIf` |
| `dependencies.json`, `setup_dependencies.py`, `prepare_cpu.py` | Pinned sources/tools and isolated CPU source preparation |
| `prepare_linux_payload.py`, `linux_hardware_trials.py`, `.tcl`, `serial_trace.py` | Internals of payload preparation and the JTAG/UART boot runner |
| `validate_*.py`, `test.ps1`, `test_*.py` | Build acceptance checks and offline regression tests |

The `.ps1` files are Windows entry points; the Python helpers do the main work.
`validate_generated.py` serves the retained non-Linux bring-up mode; the normal
Linux path uses `validate_linux_generated.py`. Generated Verilog, binaries and
logs stay in the Git-ignored `build/` subfolder, separate from the source files.

## Requirements

- ALINX AXKU3 (XCKU3P-FFVB676-2-I), external power, JTAG USB and CP210x UART USB.
- Windows, Python **3.12** with pip, Git for Windows, and Vivado **2022.2** with
  XSim/XSDB and the board's approved USB drivers.
- Run the commands below in PowerShell from `projects/axku3_linux`.

Use Windows PowerShell 5.1 or PowerShell 7, including in Windows Terminal or
VS Code. Batch output is handled by the scripts; no manual piping is needed.
If your terminal opens Command Prompt or Git Bash, enter `powershell` first,
then use the same commands below.

Python must be available as `python.exe`; Vivado defaults to
`C:\Xilinx\Vivado\2022.2`. To change paths, copy `local.example.ps1` to
`local.ps1` and edit it. Local settings are ignored by Git.
The USB drivers let Windows recognize the JTAG programmer and CP210x UART
adapter. They are Windows prerequisites, not downloaded by these scripts.

Use a checkout outside OneDrive with no spaces in its full path. Git-ignore
does not prevent OneDrive syncing. If necessary, set `CacheRoot` in `local.ps1`
to a nonsynced path without spaces; it overrides the project-local default.

## Build

Run `setup.ps1` once to download the dependency versions in `dependencies.json`.
Everything it downloads stays under this project's Git-ignored `build/` folder;
LiteX is not installed globally. Vivado, host Python, Git and USB drivers remain
separately installed prerequisites.

```text
build/
  dependencies/  LiteX, Migen, CPU/software sources and Python packages
  tools/         RISC-V compiler, build helpers and download archives
  linux/         Linux-capable CPU package and Linux image inputs
  output/        Generated sources, BIOS, bitstreams, boot payloads and test logs
```

```powershell
.\setup.ps1
.\test.ps1
.\build.ps1
.\prepare_linux_payload.ps1
```

Run each command separately and stop on error. `build.ps1` without switches
generates the SoC and compiles its BIOS; it does not run synthesis. The Linux
kernel, OpenSBI and initramfs are pinned prebuilt images, and the device tree
is generated for this exact SoC. No LiteDRAM project is needed.

After those checks pass, synthesize, then place/route and write the bitstream:

```powershell
.\build.ps1 -SynthesizeOnly
.\resume_implementation.ps1
```

These are the long steps. The scripts check synthesis and routed timing/DRC
before hardware use. Do not rebuild just to reconnect or reboot.

## Boot and use Linux

Turn on the board and connect both USB cables. Close any program using the
UART port. Replace COM6 with the CP210x port shown in Device Manager:

```powershell
.\boot.ps1 -Port COM6
.\console.ps1 -Port COM6
```

Run the console command only after boot finishes. Boot programs the FPGA,
checks RAM, uploads and verifies the Linux images, then runs userspace checks.
A complete run took about **six minutes** on the test laptop. Success ends
with `HARDWARE_TRIALS_PASS=1`.

In the console, press Enter. At `buildroot login:`, enter `root` (no password).
If already logged in, expect `root@buildroot:~#`. Try:

```sh
uname -a
free -m
cat /proc/cpuinfo
```

Type normally; avoid large pastes because this kernel's UART driver polls a
small receive FIFO. The console uses 1,000,000 baud, 8N1, no flow control.
**Ctrl+]** closes it. A blank terminal may simply need Enter; echoed commands
that do not execute usually lack a newline.

## Repeat

Linux and its files live in RAM. Reset/power loss does not reload them, and
Linux `reboot` is not supported by this minimal reset implementation. Close
the console and rerun `boot.ps1`; it reuses the bitstream. For ten independent
program/boot/test cycles:

```powershell
.\boot.ps1 -Port COM6 -Trials 10
```

Generated files and prepared boot payloads are in `build\output`; downloaded
Linux inputs are in `build\linux`. Each boot saves its transcript and summary
under `build\output\hardware`. A `CacheRoot` override relocates all four folders.
The scripts do not program boot flash. JTAG alone cannot provide this image's
Linux console.

## Cleanup

Close active builds/terminals and save any bitstreams or hardware logs you want.
From this project directory:

```powershell
.\clean.ps1 -WhatIf  # Preview; delete nothing
.\clean.ps1          # Delete build/output; keep dependencies and tools
.\clean.ps1 -All     # Delete all of build; setup.ps1 is needed again
```

Deletion asks for confirmation and is permanent, not a move to the Recycle Bin.
`-All -WhatIf` previews a full cleanup. Only this project's local `build/` is
eligible; external cache/path overrides are not followed, and linked directories
are refused. Source files and `local.ps1` are kept.

## Sources

This example uses [LiteX](https://github.com/enjoy-digital/litex),
[VexRiscv](https://github.com/SpinalHDL/VexRiscv), and the
[Linux-on-LiteX-VexRiscv](https://github.com/litex-hub/linux-on-litex-vexriscv)
software bundle. Source revisions and archive checksums are in
`dependencies.json`; dependencies retain their upstream licenses.
Vivado and its device libraries remain required for the FPGA build.

The CPU snapshot explicitly prepends `` `define SYNTHESIS `` to the pinned Verilog
to omit simulation-only constructs, matching the demonstrated build. The
source cache is not patched. UberDDR4 RTL is taken from this repository's
`rtl` directory; no controller binaries are hidden in the example.
