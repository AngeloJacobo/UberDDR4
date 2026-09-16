# Verification and simulation

[Back to README](../README.md). These are reproducible entry points, not a claim
that all configurations passed on the current checkout. Simulation, formal
properties and FPGA timing/hardware tests cover different parts of the design.

## Tools and working directory

Run the commands below from the repository root in Bash. On Windows, the shell
scripts support Git Bash with native Vivado tools and PowerShell. Set
`XILINX_VIVADO` to the installation directory, not its `bin` directory.
Lint requires Verilator, compile checks require Icarus Verilog and Yosys, and
formal requires SymbiYosys.

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
It installs links under `testbench/micron/`. The vendor
model is not bundled with this repository; obtain it through your installed
Vivado distribution and follow its supplied license. 

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


## Simulation configuration

```bash
PHY_IMPL=component bash testbench/run_xsim.sh
PHY_IMPL=native bash testbench/run_xsim.sh
PHY_IMPL=native bash testbench/regression_test.sh
bash testbench/regression_test.sh 1 4 26
```

Numeric selections are one-based entries in `ALL_TESTS` in
`testbench/regression_test.sh`. 
The 26 entries cover baseline; five fly-by delays; two map-0 cases; full BIST;
four x16 cases; two x4 cases; five rate/fly-by cases including exploratory
DDR4-1250; 4-Gbit density; row widths 14 and 17; forced training failure;
CSR reset; and byte-mask stress.

Baseline testbench parameters are tCK=834 ps, x8, two byte lanes, row width 16,
column width 10, 8-Gbit density, address map 1, BIST mode 1 and DM test disabled.
The test sets `MICRON_SIM=1`, which shortens startup waits and restricts the BIST counter
to 10 bits. **Never carry that setting into hardware.**

Normal simulation runs startup calibration/BIST followed by application tests
labelled A through Z (26 phases).

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
contract.
