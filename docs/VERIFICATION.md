# Verification and simulation

[Back to README](../README.md). These are reproducible entry points, not a claim
that all configurations passed on the current checkout. Simulation, formal
properties and FPGA timing/hardware tests cover different parts of the design.
The [hardware report](../HARDWARE_QUALIFICATION.md) records a dated campaign;
Linux build and test instructions are maintained in the
[Linux project](../projects/axku3_linux/README.md).

## Tools and working directory

Run the commands below from the repository root in Bash. On Windows, the shell
scripts support Git Bash with native Vivado tools and PowerShell. Set
`XILINX_VIVADO` to the installation directory, not its `bin` directory. The
original shell examples use Vivado 2023.1; the AXKU3 qualification used 2022.2.
These facts are not a blanket minimum-version or cross-version qualification.
Lint requires Verilator, compile checks require Icarus Verilog and Yosys, and
formal requires SymbiYosys (`sby`) plus its configured SMT toolchain.

```bash
# Linux example; substitute the installed version/path.
source /path/to/Vivado/2023.1/settings64.sh
# Git Bash alternative:
# export XILINX_VIVADO='C:/Xilinx/Vivado/2022.2'
bash testbench/setup_micron_model.sh
bash testbench/run_xsim.sh
```

The setup script checks nine Micron model files in Vivado's
`data/ip/xilinx/ddr4_v2_2/data/dlib/ultrascale/ddr4_sdram/tb/ddr4_model` directory.
It installs links under `testbench/micron/`, falling back to copies on hosts
without usable symlinks. Re-running replaces those generated files. The vendor
model is not bundled with this repository; obtain it through your installed
Vivado distribution and follow its supplied license. The checked-in
`ddr4_sdram_model_wrapper.sv` is only an empty compatibility include, not the
model itself. Actual device instances are in the testbench.

## Entry points and scope

| Command | What it selects |
| --- | --- |
| `bash run_compile.sh --lint` | Verilator checks for the configured core/wrapper source sets |
| `bash run_compile.sh --compile` | Icarus parse and Yosys checks |
| `bash run_compile.sh --formal` | Four tasks in `formal/ddr4_singleconfig.sby` |
| `bash run_compile.sh --formal-regr` | Thirty tasks in `formal/ddr4_multiconfig.sby` |
| `bash run_compile.sh --sim baseline` | One named simulation case |
| `bash run_compile.sh --sim-regr` | The 26-entry simulation matrix |
| `bash run_compile.sh --no-sim` | Lint, compile and baseline formal |
| `bash run_compile.sh --all` | Lint, compile, expanded formal and simulation regression |
| `bash run_compile.sh` | Lint, compile, baseline formal and baseline simulation |

Expanded formal replaces the single-config selection in `--all`; it does not
add four more tasks. The retained executable progress banner still says
"28 tasks", while the actual multi-config file has 30. This documentation-only
change leaves executable strings and task dispatch unchanged.

The core source list in `run_compile.sh` contains the component PHY, controller,
prober and Wishbone top, with a separate AXI helper list. It is not a native
BITSLICE implementation or timing signoff. The XSim flow compiles both PHY
implementations and selects one with `PHY_IMPL`. A successful parse or abstract
controller proof cannot establish dedicated I/O placement, electrical margins
or complete native-PHY behavior.

## Simulation configuration

```bash
PHY_IMPL=component bash testbench/run_xsim.sh
PHY_IMPL=native bash testbench/run_xsim.sh
PHY_IMPL=native bash testbench/regression_test.sh
bash testbench/regression_test.sh 1 4 26
```

Numeric selections are one-based entries in `ALL_TESTS` in
`testbench/regression_test.sh`. They are not permanent test identifiers.
The 26 entries cover baseline; five fly-by delays; two map-0 cases; full BIST;
four x16 cases; two x4 cases; five rate/fly-by cases including exploratory
DDR4-1250; 4-Gbit density; row widths 14 and 17; forced training failure;
CSR reset; and byte-mask stress. A row-width stress case is not proof that a
particular density/organization exists as a purchasable memory device.

Baseline testbench parameters are tCK=834 ps, x8, two byte lanes, row width 16,
column width 10, 8-Gbit density, address map 1, BIST mode 1 and DM test disabled.
They differ from the public top's 833 ps and default-enabled DM test. The test
sets `MICRON_SIM=1`, which shortens startup waits and restricts the BIST counter
to 10 bits. Never carry that setting into hardware.

Normal simulation runs startup calibration/BIST followed by application tests
labelled A through Z (26 phases). Forced-training-failure and CSR-reset modes
have their own completion criteria. Simulation PASS markers are interpreted by
the scripts; retain the detailed logs to see which mode actually ran.

| Environment variable | Use |
| --- | --- |
| `PHY_IMPL` | `component` (default) or `native` |
| `DUMP_VCD=1` | Enable waveform dumping for a direct XSim run |
| `REGRESSION_DUMP_VCD=1` | Enable dumping in the regression; it otherwise forces DUMP_VCD=0 |
| `SIM_TIMEOUT_MINUTES` | Positive per-case wall-clock limit in the regression; default 60 component / 240 native |
| `MICRON_DENSITY`, `MICRON_SPEED` | Model defines for direct runs; defaults `DDR4_8G_X8`, `FIXED_2400` |
| `EXTRA_DEFINES` | Additional xvlog `-d` flags for direct experiments |
| `SIM_CONFIG_FILE` | Existing Verilog configuration-header path compiled before the testbench |

Native primitive simulation can be slow. The testbench also has a separate
**10 ms simulated-time** watchdog (`TB_TIMEOUT_PS`), independent of wall time.
For numeric overrides on Vivado 2022.2 Windows, use a configuration header as
the regression does; that tool version can misparse `-d NAME=<number>`.
Keep model density/speed, controller timing and organization consistent.

Each regression recreates `testbench/regression_logs/`, including its generated
configuration header. Direct runs overwrite `sim_result.log` and other XSim
outputs. `--clean` on `run_xsim.sh` removes `xsim.dir` before rebuilding.
`run_compile.sh` recreates `build_logs/` before parsing options, including help.
Copy evidence elsewhere before another run. Use one simulation flow per checkout;
the regression lock is not a lock for arbitrary direct Vivado/XSim invocations.

The Windows helper uses a Job Object plus a timeout/stop fallback that selects
simulator processes by name and start time. That fallback is not restricted by
repository path; avoid concurrent independent XSim runs on the same Windows
host while a regression is running. On Unix, process-group cleanup and wall
limits depend on `setsid`/`timeout` availability.

## What the formal harness establishes

The single-config file has four prove-mode tasks. The multi-config file has
24 rate/organization/address-map combinations, four bounded-stall configurations
and two additional row widths: **30 total**. Ordinary tasks use depth 8;
bounded-stall tasks use depth 28. These are SymbiYosys prove tasks, not merely
bounded simulation and not a complete proof of a board-level DDR subsystem.

`ddr4_controller_formal.vh` is included inside the controller under FORMAL. It
checks command timing, bank bookkeeping, Wishbone ordering/data relationships,
address decode and selected progress properties. `mini_fifo.v` and
`f_addr_decode.v` support that abstraction; `fwb_slave.v` checks/assumes the bus
contract. Read the assumptions alongside assertions: the harness constrains
calibration, ROM/environment state and outstanding bookkeeping. Bounded-stall
configurations add environment assumptions. An assumed invariant is not an
independently proven property, even when justified by an expected operating
sequence. No PHY training, analog timing or full-system liveness claim follows
from this harness alone. The source also contains cover targets, but the supplied
SBY tasks use prove mode; a passing prove task does not show those covers were
reached. Use a separate cover-mode configuration to collect reachability traces.

## Hardware acceptance and new configurations

Use the [AXKU3 example instructions](../example_demo/axku3/README.md) to build
the standalone BIST design. Check setup, hold **and pulse-width/minimum-period**
slack, complete routing, DRC and CDC reports before programming. Preserve source
revision, tool/IP versions, parameter values, constraints, bitstream/LTX hashes
and per-attempt captures. Exercise fresh initialization repeatedly and retain
recovery counts. A passing LED, or one successful boot, is insufficient evidence
for a new pin map, memory device, clock rate or temperature range.

The September 2026 historical report gives board-specific results through
DDR4-2400 and a failing DDR4-2666 pulse-width gate. It does not qualify arbitrary
UltraScale parts or every parameter combination. Documentation/comment edits
can be checked by comparing executable tokens and tool directives; this audit
does not substitute a new simulation, formal or hardware campaign.

The retained `testbench/ddr4_sim_top.wcfg` is an older waveform-view artifact.
It contains the former `u_dut/u_phy` hierarchy and native experiment probes.
Rebuild the view from the current elaborated design (`gen_component_phy/u_phy`
or `gen_native_phy/u_phy/u_native`) before relying on its signal display. It
does not control RTL behavior and is not used by the batch test verdict.

Several directed traffic captions also predate the current parameterization.
The named bank/row fixtures principally describe x4/x8 map1. Phase P uses
literal addresses 0x0ff/0x3ff/0xfff, not the maximum address of every geometry.
Phase Q's fixed `7<<10`, `pq_ba<<8`, `pq_bg` expression reaches eight banks in
default x8 map1 despite the executable "16 banks" caption. Some 128-bit data
patterns and 16-bit masks are zero-extended on wider ports. Phase R uses a
fixed 40-cycle drain delay, not a general outstanding-ACK completion check.
A data PASS is useful evidence for those actual transactions, but does not
establish every named timing path, complete bank coverage, or full-width mask
coverage. Comments now state those limits; stimulus and executable log strings
remain unchanged by this documentation-only update.
