# Linux on AXKU3 with UberDDR4

A small RISC-V Linux system with UberDDR4 as its main-memory controller.
VexRiscv executes the software, LiteX supplies the SoC interconnect and
peripherals, and UberDDR4 drives the board's external DDR4.

The demonstrated configuration is DDR4-2400, a 300 MHz CPU, and 1 GiB of
CPU-visible RAM. It boots to a Buildroot shell over USB-UART. No Ethernet,
SD card or additional board hardware is needed.

**Status:** the tested bitstream uses the master controller RTL and passes
routed timing/DRC checks. A reproduced zero-file failure was traced to the
pinned RV32 kernel allocating its unsafe final virtual page. Payload generation
now reserves that 4 KiB page; this correction passed ten consecutive hardware
program/upload/boot/test cycles and a further known-pattern check. Startup BIST
remains disabled and its earlier failures remain open. The later banner payload,
paced console and serial-timeout recovery passed two full hardware boots,
including deliberate host pauses during upload, plus a live console check.
See [RESULTS.md](RESULTS.md) for recorded results and limits.

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
kernel and OpenSBI use pinned prebuilt images. Payload preparation verifies
the pinned base initramfs, adds the Linux banner files, and generates a device
tree covering the resulting image for this exact SoC. No LiteDRAM project is
needed.

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

If you prepared the payload before the RV32 last-page fix or the built-in
banner update, first run `.\prepare_linux_payload.ps1`. This refreshes the
initramfs and device tree without synthesis or routing. Boot rejects a device
tree missing the last-page reservation before programming; an older payload
with that reservation can still boot but will lack `uber-banner`.

Run the console command only after boot finishes. Boot programs the FPGA,
checks RAM, uploads and verifies the Linux images, then runs userspace checks.
A complete run took about **six minutes** on the test laptop. Success ends
with `HARDWARE_TRIALS_PASS=1`.

The default uploader sends 251-byte packets with one packet outstanding. The
BIOS can report a timeout after 250 ms of idle time, so a host pause can leave
an old timeout reply ahead of the next packet acknowledgement. The host handles
this with a bounded wait for a positive acknowledgement and records recoveries
in the trial's `.sfl.json` trace. A timeout alone never counts as success;
CRC errors, missing acknowledgements and full-image CRC mismatches still fail
or enter the existing bounded retry path. If an older runner prints `Retrying
with length 64`, let that attempt finish before starting another command.

In the console, press Enter. At `buildroot login:`, enter `root` (no password).
If already logged in, expect `root@buildroot:~#`. Try:

```sh
uname -a
free -m
cat /proc/cpuinfo
uber-banner
```

The boot checks already log in as root, so the automatic banner may appear
earlier in the boot transcript. `uber-banner` displays it again in your console.

Type normally or paste shell commands. `console.ps1` spaces transmitted bytes
by 5 ms (up to 200 characters/second) because this kernel's UART driver polls a
small receive FIFO. Output remains at full speed. Reopen the console after
updating the script to enable pacing; an already open console keeps its old
behavior. The UART remains at 1,000,000 baud, 8N1, no flow control. Changing only
the PC baud rate would mismatch the FPGA's configured rate.
**Ctrl+]** closes it. A blank terminal may simply need Enter; echoed commands
that do not execute usually lack a newline. If an earlier unpaced paste lost
characters and left an unfinished command at a `>` prompt, press **Ctrl+C**,
close/reopen the updated console, and paste the command again. Keep individual
commands well below 1 KiB: this image's shell line editor truncates overlong
lines even with pacing. Paste a batch of shorter commands for larger inputs.

## Demonstrate LiteX and UberDDR4 from Linux

The current bitstream already has a LiteX identifier ROM and UberDDR4's debug
registers. No FPGA rebuild is needed for the following presentation. Run these in
the board's root shell through the paced `console.ps1` described above. The
hardware commands only read registers.

First show Linux's CPU and physical RAM map:

```sh
uname -a
cat /proc/cpuinfo
cat /proc/iomem
```

For the corrected payload, System RAM ends at `0x7fffefff`; the last 4 KiB is
reserved. The hardware RAM window starts at `0x40000000`.

Read the LiteX identification ROM. In this generated CSR map its base is
`0xf0000800`, with one character in the low byte of each 32-bit CSR word.
The bounded loop reads the ROM contents rather than printing a supplied label:

```sh
ident=
i=0
while [ "$i" -lt 256 ]; do
    addr=$(printf '0xf000%04x' "$((2048 + 4*i))")
    word=$(/sbin/devmem "$addr" 32) || break
    code=$((word & 255))
    [ "$code" -eq 0 ] && break
    ident="$ident$(printf "\\$(printf '%03o' "$code")")"
    i=$((i + 1))
done
printf '%s\n' "$ident"
```

The identifier includes `LiteX VexRiscv + UberDDR4 on ALINX AXKU3` (and may
include a build timestamp). The addresses are specific to this example;
`csr.json` records them for each generated build.

Next read the actual UberDDR4 register interface:

```sh
/sbin/devmem 0xf100002c 32
/sbin/devmem 0xf1000028 32
/sbin/devmem 0xf1000014 32
/sbin/devmem 0xf1000008 32
/sbin/devmem 0xf1000018 32
/sbin/devmem 0xf100001c 32
```

| Address | Register | Interpretation for the demonstrated build |
| --- | --- | --- |
| `0xf100002c` | VERSION | `0x00000001`: IP version 0.1 |
| `0xf1000028` | CONFIG | `0x00000040`: four byte lanes (x32 DDR), startup BIST disabled |
| `0xf1000014` | BIST_STATUS | `0x00000040`: initialization done, no initialization failure; this is not a BIST pass |
| `0xf1000008` | TRAIN_FAIL | `0x00000000`: no reported lane-training failure or calibration retry |
| `0xf1000018` | LANE0_TRAINING | Packed lane-0 delay, write-leveling and bitslip results; value varies |
| `0xf100001c` | LANE1_TRAINING | Packed lane-1 training results; value varies |

For the memory-path evidence, show the `main_ram` slave and the `ddr4_top`
instance in [axku3_uberddr4.py](axku3_uberddr4.py). The main-memory Wishbone
signals connect to that instance, and its DDR ports connect to the board pins.
The resulting path is:

```text
Linux -> VexRiscv caches/MMU -> LiteX Wishbone interconnect
      -> 32-to-256-bit conversion -> UberDDR4 ddr4_top -> external DDR4
```

A hardware identifier is a build label, not cryptographic proof. Reading the
UberDDR4 CSRs demonstrates its exposed interface, but by itself does not prove
that every memory transaction uses it. Pair the live Linux output with the
source/generated-netlist wiring and the manifest for the bitstream actually
programmed. The existing boot runner records that bitstream hash and checks
RAM and Linux file contents; see [RESULTS.md](RESULTS.md).

For a future demonstration of workload activity, read/write transaction
counters can be added at the actual UberDDR4 main-memory handshake and sampled
before and after a large Linux memory workload. That requires a new bitstream.
The existing BIST correct/error counters do not count normal Linux traffic.

### Built-in LiteX and UberDDR4 banner

Prepared Linux payloads include both ASCII logos in `/etc/uberddr4-banner`.
The banner appears automatically after an interactive login. Redisplay it with:

```sh
uber-banner
```

It is included in the initramfs, so every boot from the prepared payload restores
it. It does not depend on creating a file in `/tmp`. The original LiteX BIOS
banner still appears earlier during boot.

To update an existing payload, run this in PowerShell from the project directory:

```powershell
.\prepare_linux_payload.ps1
```

Then close the console with **Ctrl+]** and use `boot.ps1 -Port COM6` for the next
boot, followed by `console.ps1 -Port COM6`. No Vivado rebuild is needed. Preparing
the payload alone does not change an already running Linux session. Use the same
`-DataRate` and `-BuildVariant` options as your build if they differ from defaults.

The generator verifies the original downloaded images and adds a deterministic
CPIO overlay; the generated manifest records the input hashes, overlay hash and
final image hashes. Its DTB includes the complete enlarged initramfs. Concatenated
archives are supported by the [Linux initramfs format](https://www.kernel.org/doc/html/latest/driver-api/early-userspace/buffer-format.html).
The banner is presentation artwork; use the hardware identification above for
live evidence. The LiteX logo and tagline come from LiteX's BIOS.

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
