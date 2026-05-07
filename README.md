
# UberDDR4 — Open Source DDR4 SDRAM Controller

An open-source, fully parameterized DDR4 SDRAM controller targeting Xilinx UltraScale+ FPGAs. Built as the successor to [UberDDR3](https://github.com/AngeloJacobo/UberDDR3), this 4:1 memory controller provides a Wishbone B4 pipelined interface (with optional AXI4 wrapper) and handles the complete DDR4 initialization sequence, refresh scheduling, bank/bank-group management, PHY calibration, and built-in self-test.

**Features:**
- Configurable timing parameters auto-derived from clock periods (DDR4-1600 through DDR4-2400)
- 4 bank groups × 4 banks with full tFAW/tRRD/tCCD_L tracking
- Bank-group interleaved address mapping for maximum throughput
- Write leveling, read gate training (bitslip), and read eye training (IDELAY tap sweep)
- Built-in self-test (BIST) with burst, random, and alternating write-read patterns
- Debug CSR register file accessible via Wishbone
- AXI4 slave wrapper using the [ZipCPU](https://github.com/ZipCPU/wb2axip) AXI-to-Wishbone bridge
- Formally verified (25 properties, k-induction proofs for both address mappings)
- Simulated against the Micron DDR4 Verilog model with 19 self-checking test phases

This project is funded through [NGI0 Entrust](https://nlnet.nl/entrust), a fund established by [NLnet](https://nlnet.nl) with financial support from the European Commission's [Next Generation Internet](https://ngi.eu) program.

***

# Table of Contents
- [Getting Started](#getting-started)
  - [Instantiate Design](#heavy_check_mark-instantiate-design)
  - [Create Constraint File](#heavy_check_mark-create-constraint-file)
- [Lint and Formal Verification](#lint-and-formal-verification)
- [Simulation](#simulation)
- [Architecture](#architecture)
  - [CSR Register Map](#csr-register-map)
- [File Structure](#file-structure)
- [Acknowledgement](#acknowledgement)

***

# Getting Started

## :heavy_check_mark: Instantiate Design

For **Wishbone** integration, use [`rtl/ddr4_top.v`](rtl/ddr4_top.v) as the top module.
For **AXI4** integration, use [`rtl/axi/ddr4_top_axi.v`](rtl/axi/ddr4_top_axi.v) instead.

### Top-Level Parameters

| Parameter | Default | Description |
| :---: | :---: | :--- |
| `CONTROLLER_CLK_PERIOD` | 3333 | Controller clock period in ps (e.g., 3333 ps = 300 MHz) |
| `DDR4_CLK_PERIOD` | 833 | DDR4 memory clock period in ps (must be ¼ of controller clock) |
| `ROW_BITS` | 16 | Row address width (14–17) |
| `COL_BITS` | 10 | Column address width (10–12) |
| `BA_BITS` | 2 | Bank address width (always 2 for DDR4) |
| `BG_BITS` | 2 | Bank group bits (2 for x4/x8, 1 for x16) |
| `DQ_BITS` | 8 | Device data width per chip (4, 8, or 16) |
| `BYTE_LANES` | 2 | Number of byte lanes |
| `DENSITY` | 8 | Device density in Gb (2, 4, 8, or 16) |
| `MICRON_SIM` | 0 | Set to 1 to shorten init delays for Micron model simulation |
| `ADDR_MAPPING` | 1 | 0 = `{row,bg,ba,col}`, 1 = `{row,ba,col,bg}` (BG-interleaved) |
| `RTT_NOM` | 3'b001 | MR1 on-die termination (001 = RZQ/4) |
| `RTT_WR` | 3'b000 | MR2 dynamic write ODT (000 = off) |
| `RTT_PARK` | 3'b000 | MR5 park termination (000 = off) |
| `DRIVE_IMP` | 0 | MR1 output driver impedance (0 = RZQ/7, 1 = RZQ/5) |
| `CL` | 0 | CAS latency override (0 = auto-calculate from clock period) |
| `CWL_PARAM` | 0 | CAS write latency override (0 = auto-calculate) |
| `BIST_MODE` | 0 | 0 = disabled, 1 = tiled test, 2 = full address space test |
| `DEBUG_CSR_ENABLE` | 1 | Enable debug CSR registers (adds 1 address bit) |
| `AXI_ID_WIDTH` | 4 | AXI transaction ID width (AXI wrapper only) |

### Clock and Reset Ports

| Port | Description |
| :---: | :--- |
| `i_controller_clk` | Controller clock with period `CONTROLLER_CLK_PERIOD` |
| `i_ddr4_clk` | DDR4 PHY clock with period `DDR4_CLK_PERIOD` (4× controller clock) |
| `i_ref_clk` | 200 MHz reference clock for IDELAYCTRL |
| `i_rst_n` | Active-low asynchronous reset |

Generate all clocks from a single MMCM/PLL.

### Wishbone B4 Pipelined Interface

| Port | Direction | Description |
| :---: | :---: | :--- |
| `i_wb_cyc` | in | Bus cycle active |
| `i_wb_stb` | in | Transfer request strobe |
| `i_wb_we` | in | Write enable (1 = write, 0 = read) |
| `i_wb_addr` | in | Address bus (`EXT_ADDR_BITS` wide) |
| `i_wb_data` | in | Write data (`WB_DATA_BITS` wide, default 128 bits) |
| `i_wb_sel` | in | Byte select / write strobe (`WB_SEL_BITS` wide) |
| `o_wb_stall` | out | Pipeline stall (do not issue new STB when high) |
| `o_wb_ack` | out | Transfer acknowledge |
| `o_wb_data` | out | Read data (`WB_DATA_BITS` wide) |

### Status Outputs

| Port | Description |
| :---: | :--- |
| `o_init_done` | Sticky — asserted when calibration (and BIST if enabled) completes successfully |
| `o_init_failed` | Sticky — asserted on calibration error or BIST failure |

## :heavy_check_mark: Create Constraint File

DDR4 I/O pins must be placed in a single I/O bank with SSTL12 or POD12 I/O standards. See the Xilinx UltraScale+ SelectIO guide (UG571) for bank requirements. The reference clock (`i_ref_clk`) must be 200 MHz for IDELAYCTRL.

***

# Lint and Formal Verification

Run [`./run_compile.sh`](run_compile.sh) from the top-level directory to lint, formally verify, and simulate:

```bash
./run_compile.sh lint     # Yosys synthesis check
./run_compile.sh formal   # SymbiYosys formal proofs (4 tasks)
./run_compile.sh sim      # Xilinx xsim simulation
./run_compile.sh all      # All of the above
```

### Formal Verification Tasks

| Task | Depth | Description |
| :---: | :---: | :--- |
| `prove_map0` | 8 | ADDR_MAPPING=0, unbounded k-induction |
| `prove_map1` | 8 | ADDR_MAPPING=1, unbounded k-induction |
| `prove_map0_bounded` | 28 | ADDR_MAPPING=0 + bounded stall property |
| `prove_map1_bounded` | 28 | ADDR_MAPPING=1 + bounded stall property |

25 properties are proven covering Wishbone protocol compliance, refresh scheduling, bank state consistency, timing parameter enforcement, and command serialization.

***

# Simulation

The simulation uses the [Micron DDR4 SDRAM Verilog Model](https://www.micron.com). Place the Micron model files (`.sv` and `.sva`) under `testbench/` alongside the provided wrapper.

### Running with Vivado xsim

```bash
cd testbench
bash run_xsim.sh
```

The testbench (`ddr4_sim_top.sv`) executes 19 self-checking test phases:

| Phase | Test Description |
| :---: | :--- |
| A | Sequential burst writes to BG0/BA0 |
| B | Sequential burst writes to BG0/BA1 |
| C | Cross-bank-group writes (BG0 → BG1) |
| D | Cross-row writes (page miss) |
| E | Multi-bank-group sequential writes |
| F | Sequential read-back with data verification |
| G–H | Additional row/bank read-back |
| I | Pipeline stress — back-to-back writes |
| J | Full pipeline read-back |
| K | Read-to-write turnaround stress |
| L | Data masking (byte-lane selective writes) |
| M | tFAW sliding window stress (5 activates) |
| N | Pipeline saturation (32 outstanding writes) |
| O | Refresh-during-traffic |
| P | Address boundary corners |
| Q | Multi-bank-group interleaving (16 banks) |
| CSR | Debug CSR register read/verify |
| RETRIG | BIST re-trigger via CSR and post-check |

A regression script (`testbench/regression_test.sh`) sweeps 8 configurations across DDR4-2400/1600, DQ 8/16, and both address mappings.

***

# Architecture

```
┌─────────────────────────────────────────────┐
│               ddr4_top.v                    │
│  ┌──────────────┐  ┌──────────────────────┐ │
│  │ ddr4_prober  │  │   ddr4_controller    │ │
│  │ (BIST + CSR) │  │   (scheduling,       │ │
│  │              │  │    timing, refresh)   │ │
│  └──────┬───────┘  └──────────┬───────────┘ │
│         │    Wishbone mux     │ DFI 3.1     │
│  WB ────┤                     │             │
│         │              ┌──────┴───────────┐ │
│         │              │    ddr4_phy      │ │
│         │              │  (SERDES, IDELAY,│ │
│         │              │   training FSM)  │ │
│         │              └──────┬───────────┘ │
└─────────┼─────────────────────┼─────────────┘
          │                     │ DDR4 SDRAM
     o_init_done          ck/addr/cmd/dq/dqs
     o_init_failed
```

For AXI4 integration, `ddr4_top_axi.v` wraps `ddr4_top` with the ZipCPU `axim2wbsp` bridge:

```
AXI4 slave ──► axim2wbsp ──► ddr4_top (Wishbone) ──► DDR4
```

### CSR Register Map

When `DEBUG_CSR_ENABLE=1`, the top address bit selects between DRAM access (bit=0) and CSR access (bit=1). CSR registers are 32-bit, read via Wishbone with `addr[3:0]` selecting the register:

| Addr | Name | Access | Description |
| :---: | :--- | :---: | :--- |
| 0x0 | Controller Status | RO | `[3:0]` phy_state, `[7:4]` calib_state, `[8]` stage1_pending, `[9]` stage2_pending, `[11]` stage2_we, `[12]` refresh_idle |
| 0x1 | Bank Status | RO | `[NUM_BANKS-1:0]` per-bank active flag |
| 0x3 | BIST Correct Count | RO | Number of passing read comparisons |
| 0x4 | BIST Error Count | RO | Number of failing read comparisons |
| 0x5 | BIST State | RO | `[2:0]` FSM state, `[3]` busy, `[4]` pass, `[5]` fail |
| 0x6 | PHY Lane 0 | RO | `[3:0]` phy_state, `[12:4]` idelay_center, `[21:13]` wl_tap, `[24:22]` bitslip |
| 0x7 | PHY Lane 1 | RO | `[8:0]` idelay_center, `[17:9]` wl_tap, `[20:18]` bitslip |
| 0xA | Configuration | RO | `[1:0]` BIST_MODE, `[7:4]` BYTE_LANES |
| 0xB | Version | RO | `[7:0]` minor, `[15:8]` major |
| 0xC | Control | WO | Write bit[0]=1 to trigger BIST start |

***

# File Structure

```
UberDDR4/
├── rtl/
│   ├── ddr4_controller.v      # Memory controller (scheduling, timing, refresh)
│   ├── ddr4_phy.v             # UltraScale+ PHY (SERDES, IDELAY, training)
│   ├── ddr4_prober.v          # BIST engine + debug CSR register file
│   ├── ddr4_top.v             # Top-level (Wishbone interface)
│   └── axi/
│       ├── ddr4_top_axi.v     # AXI4 top wrapper
│       ├── axim2wbsp.v        # ZipCPU AXI-to-WB bridge
│       ├── aximrd2wbsp.v      # AXI read channel bridge
│       ├── aximwr2wbsp.v      # AXI write channel bridge
│       ├── axi_addr.v         # AXI address calculator
│       ├── sfifo.v            # Synchronous FIFO
│       ├── skidbuffer.v       # Skid buffer
│       └── wbarbiter.v        # Wishbone arbiter
├── formal/
│   ├── ddr4_controller_formal.vh  # Formal properties (25 assertions)
│   ├── ddr4_singleconfig.sby      # SymbiYosys config (4 tasks)
│   ├── ddr4_multiconfig.sby       # Multi-config sweep (24 tasks)
│   ├── fwb_slave.v                # Wishbone formal slave
│   ├── mini_fifo.v                # Helper FIFO for formal
│   └── f_addr_decode.v            # Address decode checker
├── testbench/
│   ├── ddr4_sim_top.sv            # Top-level simulation testbench
│   ├── ddr4_model_wrapper.sv      # Micron model wrapper
│   ├── run_xsim.sh               # Vivado xsim run script
│   └── regression_test.sh         # 8-config regression suite
└── run_compile.sh                 # Build & verification sweep
```

***

# Acknowledgement

This project is funded through [NGI0 Entrust](https://nlnet.nl/entrust), a fund established by [NLnet](https://nlnet.nl) with financial support from the European Commission's [Next Generation Internet](https://ngi.eu) program. Learn more at the [NLnet project page](https://nlnet.nl/project/UberDDR).

[<img src="https://nlnet.nl/logo/banner.png" alt="NLnet foundation logo" width="20%" />](https://nlnet.nl)
[<img src="https://nlnet.nl/image/logos/NGI0_tag.svg" alt="NGI Zero Logo" width="20%" />](https://nlnet.nl/entrust)
