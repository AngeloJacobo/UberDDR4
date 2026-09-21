# Linux on AXKU3 with UberDDR4

[<img width="812" height="450" alt="image" src="https://github.com/user-attachments/assets/5b2dd61b-9df7-4864-9042-615ed2acf07c" />](https://youtu.be/BuG7Sij6QpU?si=UYbmmnR7TkbpSWEW)

A small RISC-V Linux system with UberDDR4 as its main-memory controller.
VexRiscv executes the software, LiteX supplies the SoC interconnect and
peripherals, and UberDDR4 drives the board's external DDR4.

The demonstrated configuration is DDR4-2400, a 300 MHz CPU, and 1 GiB of
CPU-visible RAM. It boots to a Buildroot shell over USB-UART. No Ethernet,
SD card or additional board hardware is needed.


## Where to start reading

The hardware is mainly in **axku3_uberddr4.py**; most other files automate builds
and tests. You do not need to understand all of them to use the example.

| Files | What they do |
| --- | --- |
| `axku3_uberddr4.py` | Clocks/reset, CPU, memory map and Wishbone connection to UberDDR4 |
| `axku3_platform.py`, `constraints/*.xdc` | AXKU3 pins, I/O standards and board constraints |
| `uberddr4.sh` | The single entry point: setup, check, test, build, payload, implement, all, boot, campaign, console, clean |
| `local.example.sh` | Optional local tool paths; copy to `local.sh` |
| `dependencies.json`, `setup_dependencies.py`, `prepare_cpu.py` | Pinned sources/tools and isolated CPU source preparation |
| `prepare_linux_payload.py`, `linux_hardware_trials.py`, `.tcl`, `serial_trace.py` | Internals of payload preparation and the JTAG/UART boot runner |
| `validate_*.py`, `test_*.py` | Build acceptance checks and offline regression tests |

`uberddr4.sh` is the only entry point; the Python helpers do the main work. Run
`./uberddr4.sh --help` for the full command and option list.
`validate_generated.py` serves the retained non-Linux bring-up mode; the normal
Linux path uses `validate_linux_generated.py`. Generated Verilog, binaries and
logs stay in the Git-ignored `build/` subfolder, separate from the source files.

## Requirements

- ALINX AXKU3 (XCKU3P-FFVB676-2-I), external power, JTAG USB and CP210x UART USB.
- Windows, Python **3.12** with pip, Git for Windows, and Vivado **2022.2** with
  XSim/XSDB and the board's approved USB drivers.
- Run the commands below from `projects/axku3_linux` in a shell: Git Bash on
  Windows, or any POSIX shell elsewhere.

`uberddr4.sh` is a portable shell script and does not require PowerShell. On
Windows use the Git Bash that ships with Git for Windows, including inside
Windows Terminal or VS Code. It converts paths for native Python and Vivado
itself; no manual conversion or piping is needed.

**The demonstrated and tested host is Windows.** `uberddr4.sh` itself is POSIX
shell and its Windows-only pieces are all behind a host check, but `setup`
cannot currently produce a working RISC-V toolchain on a Linux host:

Python must be available as `python.exe` on Windows (`python3` elsewhere); Vivado
defaults to `C:\Xilinx\Vivado\2022.2`. To change paths, copy `local.example.sh` to
`local.sh` and edit it. Local settings are ignored by Git.
The USB drivers let Windows recognize the JTAG programmer and CP210x UART
adapter. They are Windows prerequisites, not downloaded by this script.

Use a checkout outside OneDrive with no spaces in its full path. Git-ignore
does not prevent OneDrive syncing. If necessary, set `CACHE_ROOT` in `local.sh`
to a nonsynced path without spaces; it overrides the project-local default.

## Build

Run `./uberddr4.sh setup` once to download the dependency versions in
`dependencies.json`. Everything it downloads stays under this project's
Git-ignored `build/` folder; LiteX is not installed globally. Vivado, host
Python, Git and USB drivers remain separately installed prerequisites.

Each dependency is cloned at its pinned commit, and submodules are fetched only
where `dependencies.json` lists them. The CPU packages ship the Verilog this
project uses, so their Scala sources are skipped; cloning those would also pull
a nested test-data repository whose path is too long for Git for Windows.

```text
build/
  dependencies/  LiteX, Migen, CPU/software sources and Python packages
  tools/         RISC-V compiler, build helpers and download archives
  linux/         Linux-capable CPU package and Linux image inputs
  output/        Generated sources, BIOS, bitstreams, boot payloads and test logs
```

```sh
./uberddr4.sh setup
./uberddr4.sh test
./uberddr4.sh build
./uberddr4.sh payload
```

Run each command separately and stop on error, or use `./uberddr4.sh all`
below to run the whole sequence in one invocation. `build` without switches
generates the SoC and compiles its BIOS; it does not run synthesis. The Linux
kernel and OpenSBI use pinned prebuilt images. Payload preparation verifies
the pinned base initramfs, adds the Linux banner files, and generates a device
tree covering the resulting image for this exact SoC. No LiteDRAM project is
needed.

After those checks pass, synthesize, then place/route and write the bitstream:

```sh
./uberddr4.sh build --synthesize-only
./uberddr4.sh implement
```

These are the long steps. The script checks synthesis and routed timing/DRC
before hardware use. Do not rebuild just to reconnect or reboot.

### Everything in one command

`all` runs the six steps above in order - setup, test, build, payload,
synthesize, implement - and stops at the first failure:

```sh
./uberddr4.sh all
./uberddr4.sh all --data-rate 2400 --skip-setup
```

It starts each step in a separate process, exactly as running the commands one
after another would, so nothing a long Vivado step exports leaks into the next.
All of its options are validated before the first download, so a typo meant for
the last step is reported immediately rather than an hour in.
`--data-rate`, `--uart-name`, `--uart-baudrate` and `--build-variant` are passed
to the steps that take them; `--skip-setup` reuses dependencies already in
`build/`. `--build-root`, `--linux-deps-root` and `--cache-root` are passed to
every step.

The payload is generated before synthesis, so a bad configuration surfaces in
seconds rather than after the long runs. Expect a few hours on the test laptop,
almost all of it Vivado. Each step prints `ALL_STEP:` with the command it is
about to run; success ends with `ALL_PASS`. `all` does not touch the board -
run `boot` afterwards.

## Boot and use Linux

Turn on the board and connect both USB cables. Close any program using the
UART port. Replace COM6 with the CP210x port shown in Device Manager:

```sh
./uberddr4.sh boot --port COM6
./uberddr4.sh console --port COM6
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
uber-banner
```

The boot checks already log in as root, so the automatic banner may appear
earlier in the boot transcript. `uber-banner` displays it again in your console.


## Demonstrate LiteX and UberDDR4 from Linux

The current bitstream already has a LiteX identifier ROM and UberDDR4's debug
registers. No FPGA rebuild is needed for the following presentation. Run these in
the board's root shell through the paced console described above. The
hardware commands only read registers.

First show Linux's CPU and physical RAM map:

```sh
uname -a
cat /proc/cpuinfo
cat /proc/iomem
```

For the corrected payload, System RAM ends at `0x7fffefff`; the last 4 KiB is
reserved. The hardware RAM window starts at `0x40000000`.

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


### Built-in LiteX and UberDDR4 banner

Prepared Linux payloads include both ASCII logos in `/etc/uberddr4-banner`.
The banner appears automatically after an interactive login. Redisplay it with:

```sh
uber-banner
```

It is included in the initramfs, so every boot from the prepared payload restores
it. It does not depend on creating a file in `/tmp`. The original LiteX BIOS
banner still appears earlier during boot.

To update an existing payload, run this from the project directory:

```sh
./uberddr4.sh payload
```

Then close the console with **Ctrl+]** and use `./uberddr4.sh boot --port COM6` for
the next boot, followed by `./uberddr4.sh console --port COM6`. No Vivado rebuild is
needed. Preparing the payload alone does not change an already running Linux
session. Use the same `--data-rate` and `--build-variant` options as your build if
they differ from defaults.


## Repeat

Linux and its files live in RAM. Reset/power loss does not reload them, and
Linux `reboot` is not supported by this minimal reset implementation. Close
the console and rerun `./uberddr4.sh boot`; it reuses the bitstream. For ten
independent program/boot/test cycles:

```sh
./uberddr4.sh boot --port COM6 --trials 10
```

### Reliability soak

One passing boot shows the flow works once; reliability is a rate. The
`campaign` command repeats the whole batch and reports how many rounds and
trials passed:

```sh
./uberddr4.sh campaign --port COM6 --repeat 5 --trials 10
```

That is five rounds of ten program/upload/boot/test cycles, 50 in total.
Each round reprograms the FPGA and gets its own evidence directory
`build/output/hardware/linux_uberddr4_<rate>_<stamp>_roundNN`, with the same
per-trial transcripts, manifest and `summary.tsv` a plain `boot` produces. The
campaign's own `campaign_<rate>_<stamp>/campaign.tsv` records each round's
start time, end time, evidence directory and verdict.

A failed trial is recorded and the soak continues, because stopping at the
first failure only reports that one happened; the next trial reprograms the
board, which recovers it from whatever state the failure left. The run still
ends non-zero, naming the failed rounds, and prints the tally:

```text
CAMPAIGN_ROUNDS_PASS=4
CAMPAIGN_ROUNDS_FAIL=1
CAMPAIGN_TRIALS_PASS=48
CAMPAIGN_TRIALS_FAIL=2
```

`CAMPAIGN_PASS` is printed only when every round passed. Pass
`--stop-on-failure` to halt at the first failed trial instead and leave the
board in that state for inspection. Failed trials keep their UART transcript
and a `trial_NN_failure.json` alongside the passing ones either way.

Generated files and prepared boot payloads are in `build/output`; downloaded
Linux inputs are in `build/linux`. Each boot saves its transcript and summary
under `build/output/hardware`. A `CACHE_ROOT` override relocates all four folders.
The script does not program boot flash. JTAG alone cannot provide this image's
Linux console.

## Cleanup

Close active builds/terminals and save any bitstreams or hardware logs you want.
From this project directory:

```sh
./uberddr4.sh clean --dry-run  # Preview; delete nothing
./uberddr4.sh clean            # Delete build/output; keep dependencies and tools
./uberddr4.sh clean --all      # Delete all of build; setup is needed again
```

Deletion asks for confirmation and is permanent, not a move to the Recycle Bin.
Pass `--yes` to skip the prompt in a script; without a terminal and without
`--yes`, cleanup refuses rather than assuming yes. `--all --dry-run` previews a
full cleanup. Only this project's local `build/` is eligible; `local.sh` is never
loaded by cleanup, external cache/path overrides are not followed, and linked
directories are refused. Source files and `local.sh` are kept.

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
