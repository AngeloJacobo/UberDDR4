
# UberDDR4 -- Open Source DDR4 SDRAM Controller

An open-source, fully parameterized DDR4 SDRAM controller targeting Xilinx UltraScale+ FPGAs. Built as the successor to [UberDDR3](https://github.com/AngeloJacobo/UberDDR3), this 4:1 memory controller provides a Wishbone B4 pipelined interface (with optional AXI4 wrapper) and handles the complete DDR4 initialization sequence, refresh scheduling, bank/bank-group management, PHY calibration, and built-in self-test.

**Features:**
- Configurable timing parameters auto-derived from clock periods (DDR4-1600 through DDR4-2400)
- 4 bank groups x 4 banks with full tFAW/tRRD/tCCD_L tracking
- Bank-group interleaved address mapping for maximum throughput
- Speculative bank anticipation (both PRECHARGE and ACTIVATE) for next-request lookahead
- Write leveling, read gate training (bitslip), and phase-aware read eye training (IDELAY tap sweep with late-arrival detection)
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
- [Build & Verification Suite](#build--verification-suite)
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
| `DDR4_CLK_PERIOD` | 833 | DDR4 memory clock period in ps (must be 1/4 of controller clock) |
| `DEVICE_WIDTH` | 8 | DDR4 device data width (4, 8, or 16). Auto-derives BG_BITS and DM. |
| `ROW_BITS` | 16 | Row address width (14-17) |
| `COL_BITS` | 10 | Column address width (10) |
| `BYTE_LANES` | 2 | Number of 8-bit byte lanes (see device width table below) |
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

### Device Width Configuration

The PHY always operates in 8-bit byte lanes. `DEVICE_WIDTH` controls DDR4 device-specific
behavior (bank groups, data mask, page size) per JESD79-4D. `BYTE_LANES` sets the total
number of 8-bit lanes across all devices.

| Config | `DEVICE_WIDTH` | `BYTE_LANES` | BG_BITS | DM | Physical Topology |
| :---: | :---: | :---: | :---: | :---: | :--- |
| 2x x8 (default) | 8 | 2 | 2 | yes | 1 x8 chip per byte lane |
| 1x x16 | 16 | 2 | 1 | yes | 1 x16 chip spanning 2 byte lanes |
| 2x x16 | 16 | 4 | 1 | yes | 2 x16 chips (4 byte lanes) |
| 4x x4 | 4 | 2 | 2 | no | 2 x4 chips paired per byte lane |

**Note:** x4 devices have no DM pin (JESD79-4D Table 28), so byte-masked writes
(`wb_sel != all-1s`) are not supported for x4 configurations.

### Clock and Reset Ports

| Port | Description |
| :---: | :--- |
| `i_controller_clk` | Controller clock with period `CONTROLLER_CLK_PERIOD` |
| `i_ddr4_clk` | DDR4 PHY clock with period `DDR4_CLK_PERIOD` (4x controller clock) |
| `i_ref_clk` | 200 MHz reference clock for IDELAYCTRL |
| `i_rst_n` | Active-low asynchronous reset |

Generate all clocks from a single MMCM/PLL.

### Wishbone B4 Pipelined Interface (DRAM Data Path)

| Port | Direction | Description |
| :---: | :---: | :--- |
| `i_wb_cyc` | in | Bus cycle active |
| `i_wb_stb` | in | Transfer request strobe |
| `i_wb_we` | in | Write enable (1 = write, 0 = read) |
| `i_wb_addr` | in | Address bus (`WB_ADDR_BITS` wide) |
| `i_wb_data` | in | Write data (`WB_DATA_BITS` wide, default 128 bits) |
| `i_wb_sel` | in | Byte select / write strobe (`WB_SEL_BITS` wide) |
| `o_wb_stall` | out | Pipeline stall (do not issue new STB when high) |
| `o_wb_ack` | out | Transfer acknowledge (guaranteed in-order via shared ACK pipe) |
| `o_wb_data` | out | Read data (`WB_DATA_BITS` wide) |

ACKs are returned strictly in request order using a shared shift-register (`ack_pipe_q`). Reads are inserted at the far end (full CAS latency) while writes are inserted at a variable position closer to the output, ensuring writes ACK faster when no earlier read is pending. Read ACKs are gated by `i_dfi_rddata_valid` via a `read_data_pending` counter — the pipe stalls if data hasn't arrived from the PHY yet, making the design robust to PHYs with variable or larger `tphy_rdlat`.

### Debug CSR Wishbone B4 Port (Separate, Always Accessible)

| Port | Direction | Description |
| :---: | :---: | :--- |
| `i_wb_dbg_cyc` | in | Bus cycle active |
| `i_wb_dbg_stb` | in | Transfer request strobe |
| `i_wb_dbg_we` | in | Write enable |
| `i_wb_dbg_addr` | in | CSR address (4-bit, selects 1 of 16 registers) |
| `i_wb_dbg_data` | in | Write data (32-bit) |
| `i_wb_dbg_sel` | in | Byte select (4-bit) |
| `o_wb_dbg_stall` | out | Always 0 (zero-wait-state slave) |
| `o_wb_dbg_ack` | out | Transfer acknowledge (1-cycle latency) |
| `o_wb_dbg_data` | out | Read data (32-bit) |

This port is completely independent of the DRAM data path. It can be used to read training status, BIST results, and PHY debug registers even when the controller is stalled during calibration.

### Status Outputs

| Port | Description |
| :---: | :--- |
| `o_init_done` | Sticky -- asserted when calibration (and BIST if enabled) completes successfully |
| `o_init_failed` | Sticky -- asserted on calibration error or BIST failure |

## :heavy_check_mark: Create Constraint File

DDR4 I/O pins must be placed in a single I/O bank with SSTL12 or POD12 I/O standards. See the Xilinx UltraScale+ SelectIO guide (UG571) for bank requirements. The reference clock (`i_ref_clk`) must be 300 MHz for IDELAYCTRL.

***

# Build & Verification Suite

[`run_compile.sh`](run_compile.sh) is the unified entry point for lint, compile, formal, and simulation.

```bash
./run_compile.sh                     # default (lint + compile + formal + sim baseline)
./run_compile.sh --all               # everything (lint + compile + formal-regr + sim-regr)
./run_compile.sh --lint              # verilator lint only
./run_compile.sh --compile           # iverilog + yosys compile check
./run_compile.sh --formal            # formal single config (4 tasks)
./run_compile.sh --formal-regr       # formal regression (all configs, 32 tasks)
./run_compile.sh --sim [TEST]        # single sim test (default: baseline)
./run_compile.sh --sim-regr          # full sim regression (22 tests, ~4 hours)
./run_compile.sh --no-sim            # lint + compile + formal (skip sim)
./run_compile.sh --help              # list all options and available test names
```

Multiple flags can be combined (e.g. `--lint --formal`). Logs are written to `build_logs/`.

### Prerequisites

| Tool | Purpose | Install |
| :--- | :--- | :--- |
| [Verilator](https://verilator.org) | RTL lint | [OSS CAD Suite](https://github.com/YosysHQ/oss-cad-suite-build) |
| [Icarus Verilog](http://iverilog.icarus.com) | Syntax/elaboration check | OSS CAD Suite |
| [Yosys](https://yosyshq.net/yosys/) | Synthesis check | OSS CAD Suite |
| [SymbiYosys](https://github.com/YosysHQ/sby) | Formal verification | OSS CAD Suite |
| [Vivado xsim](https://www.xilinx.com/products/design-tools/vivado.html) | Simulation | Xilinx Vivado 2023.1+ |

Set `XILINX_VIVADO` to the Vivado install path (e.g. `/path/to/Vivado/2023.1`).

***

# Architecture

```
+--------------------------------------------------+
|                  ddr4_top.v                       |
|  +--------------+     +----------------------+   |
|  | ddr4_prober  |     |   ddr4_controller    |   |
|  | (BIST + CSR) |     |   (scheduling,       |   |
|  |              |     |    timing, refresh)   |   |
|  +---+------+---+     +----------+-----------+   |
|      |      |   DRAM WB mux      | DFI 3.1       |
|  DBG |  DRAM|---+                 |               |
|  WB -+  WB -+--+          +------+-----------+   |
|                            |    ddr4_phy      |   |
|                            |  (SERDES, IDELAY,|   |
|                            |   training FSM)  |   |
|                            +------+-----------+   |
+-----------------------------------+---------------+
                                    | DDR4 SDRAM
     o_init_done              ck/addr/cmd/dq/dqs
     o_init_failed
```

For AXI4 integration, `ddr4_top_axi.v` wraps `ddr4_top` with the ZipCPU `axim2wbsp` bridge:

```
AXI4 slave --> axim2wbsp --> ddr4_top (Wishbone) --> DDR4
```

### PHY Eye Training

The PHY calibration sequence performs write leveling, bitslip (gate) training, and a full phase-aware IDELAYE3 tap sweep for read eye training. The eye training algorithm sweeps all 512 IDELAYE3 taps across 9 DQS offsets, tracking up to two passing ranges per lane and selecting the center of the widest range.

**Late-Arrival Detection (`PHY_EYE_LATE`, state 4'd6):**  
High IDELAYE3 tap values can push a BL8 burst's arrival past `rddata_en` by one full CLKDIV cycle. The `PHY_EYE_LATE` state re-samples `train_window` one cycle after `rddata_en` to catch these taps. Ranges are identified by `(offset, late)` tuples so on-time and late regions are never merged. If the best range for a lane is late, the per-lane `rd_lat_extra` flag is set; that lane then uses `rddata_en_d1` for capture, and `rddata_valid` is delayed by one cycle. The controller's `pipe_stall` credit counter absorbs the extra latency transparently. See [`doc/eye_training_redesign.md`](doc/eye_training_redesign.md) for the full design specification.

### CSR Register Map

CSR registers are 32-bit, accessed via the dedicated debug Wishbone port (`i_wb_dbg_addr[3:0]` selects the register):

| Addr | Name | Access | Description |
| :---: | :--- | :---: | :--- |
| 0x0 | STATUS | RO | `[3:0]` phy_state, `[7:4]` calib_state, `[8]` stage1_pending, `[9]` stage2_pending, `[11]` stage2_we, `[12]` refresh_idle |
| 0x1 | BANK_STATUS | RO | `[NUM_BANKS-1:0]` per-bank active flag (1=row open, 0=idle) |
| 0x2 | TRAIN_FAIL | RO | `[BL-1:0]` gate_fail, `[2*BL-1:BL]` eye_fail, `[3*BL-1:2*BL]` wl_fail, `[3*BL+1:3*BL]` calib_retry_count |
| 0x3 | CORRECT_COUNT | RO | BIST: number of passing read comparisons |
| 0x4 | ERROR_COUNT | RO | BIST: number of failing read comparisons |
| 0x5 | BIST_STATUS | RO | `[2:0]` FSM state, `[3]` busy, `[4]` pass, `[5]` fail_sticky, `[6]` init_done, `[7]` init_failed |
| 0x6 | LANE0_TRAINING | RO | `[8:0]` idelay_center, `[17:9]` wl_dqs_tap, `[21:18]` bitslip, `[30:22]` best_start |
| 0x7 | LANE1_TRAINING | RO | `[8:0]` idelay_center, `[17:9]` wl_dqs_tap, `[21:18]` bitslip, `[30:22]` best_start |
| 0x8 | EYE_HEALTH | RO | `[8:0]` lane0 eye_width, `[17:9]` lane1 eye_width, `[19:18]` rd_lat_extra, `[20]` en_vtc |
| 0x9 | WRITE_PATH | RO | `[7:0]` lane0 wl_dq_tap, `[15:8]` lane1 wl_dq_tap, `[23:16]` lane0 dqs_initial_tap, `[31:24]` lane1 dqs_initial_tap |
| 0xA | CONFIG | RO | `[1:0]` BIST_MODE, `[7:4]` BYTE_LANES |
| 0xB | VERSION | RO | `[7:0]` minor, `[15:8]` major |
| 0xC | CONTROL | R/W | `[0]` W1S: trigger BIST start, `[1]` W1S: soft reset (re-calibrate), `[2]` R/W: auto_reset_en |
| 0xD | INIT_PROGRESS | RO | `[5:0]` ROM instruction_address, `[6]` pause_counter, `[7]` reset_done, `[8]` pipe_stall |

### CSR Debugging Guide

This section explains how to interpret the CSR register dump produced at end-of-simulation (or read via the debug Wishbone port at runtime) and how to use the values for board bring-up and failure triage.

#### Understanding Tap Values

The tap resolution is specified in the device datasheets as `TIDELAY_RESOLUTION` / `TODELAY_RESOLUTION`:
- **Kintex UltraScale+ (DS922):** 2.1 to 12 ps/tap
- **Artix UltraScale+ (DS931):** 2.1 to 12 ps/tap

**Calculating actual ps/tap from the CSR dump:** The DQS ODELAYE3 is initialized with `DELAY_VALUE = DDR4_CLK_PERIOD / 4` (90° phase shift). BISC converts this to a tap count, reported as "DQS Init Tap" in CSR 0x9/0xE. We can use this to get ps/tap:

```
ps_per_tap = (DDR4_CLK_PERIOD / 4) / DQS_Init_Tap

Example (DDR4-2400): 833 ps / 4 / 52 taps = 4.0 ps/tap
Example (DDR4-1600): 1250 ps / 4 / 78 taps = 4.0 ps/tap
```

This applies to both IDELAY and ODELAY since they share the same resolution.

**Interpreting eye width in ps:** Multiply eye_width taps by ps_per_tap.  
Example: 104 taps × 4.0 ps = 416 ps. At DDR4-2400 the bit period is 416 ps, so this represents a full bit period of margin — an excellent result in simulation (real hardware will be narrower due to ISI/crosstalk).

#### Field-by-Field Interpretation

**CSR 0x0 — STATUS (Controller + PHY live state)**
| Bit | Field | Healthy Value | Meaning |
| :---: | :--- | :---: | :--- |
| [3:0] | PHY Calib State | 0 | See decode table below |
| [7:4] | Controller Calib State | 13 | See decode table below |
| [8] | Stage1 Pending | 0 | 0 = No WB request in stage 1 |
| [9] | Stage2 Pending | 0 | 0 = No WB request in stage 2 |
| [11] | Stage2 WE | — | 1=write, 0=read request in stage 2 |
| [12] | Refresh Idle | 0 | 1 = ROM is in the tREFI (7.8us) wait (accepting user traffic) |

**PHY Calib State decode ([3:0]):**

| Value | State | Description |
| :---: | :--- | :--- |
| 0 | PHY_IDLE | Idle — no training in progress |
| 1 | PHY_GATE_DONE | Gate (bitslip) training complete |
| 2–7 | PHY_EYE_* | Eye training (sweep, track, decide, verify, late-check, done) |
| 8–11 | PHY_WL_* | Write leveling (sample, adjust, check, done) |

**Controller Calib State decode ([7:4]):**

| Value | State | Description |
| :---: | :--- | :--- |
| 0 | CALIB_IDLE | Waiting for calibration trigger |
| 1–4 | CALIB_GATE_* | Gate training (enable MPR, read, wait, exit) |
| 5–8 | CALIB_EYE_* | Eye training (enable MPR, read, wait, exit) |
| 9–12 | CALIB_WL_* | Write leveling (enable WL, strobe, wait, exit) |
| 13 | CALIB_DONE | All training complete (healthy steady-state) |
| 14 | CALIB_ERROR | Fatal — training exhausted retries |

---  

**CSR 0x1 — BANK_STATUS**

Each bit represents one bank (up to 16 banks for 4 BG × 4 BA). Bit=1 means that bank has an open (activated) row. After every refresh, all banks become idle (0).
| Bit | Field |
| :--- | :--- |
| [3:0] | {BG0BA3, BG0BA2, BG0BA1, BG0BA0} |
| [7:4] | {BG1BA3, BG1BA2, BG1BA1, BG1BA0} |
| [11:8] | {BG2BA3, BG2BA2, BG2BA1, BG2BA0} |
| [12:15] | {BG3BA3, BG3BA2, BG3BA1, BG3BA0} |

---  

**CSR 0x2 — TRAIN_FAIL**

Training flags per byte lane plus retry count. All zeros means all training phases passed on the first attempt.

| Bit | Field | Healthy Value | Meaning |
| :---: | :--- | :---: | :--- |
| [BL-1:0] | gate_fail | 0 | 1 = Bitslip alignment failed (MPR pattern never found at any offset) |
| [2*BL-1:BL] | eye_fail | 0 | 1 = Eye training failed (no passing IDELAY range found) |
| [3*BL-1:2*BL] | wl_fail | 0 | 1 = Write leveling failed (DQ 0→1 transition never detected) |
| [3*BL+1:3*BL] | calib_retry_count | 0 | Training retries for current phase (0–3). Reaches 3 → `CALIB_ERROR`. Non-zero = at least one timeout during training. |

---  

**CSR 0x3/0x4 — BIST Correct/Error Counts**

Total number of read comparisons that passed (0x3) or failed (0x4). A healthy system has `correct_count (CSR 0x3) == expected` and `error_count (CSR 0x4) == 0`.

| BIST_MODE | BIST_ADDR_BITS | Expected Correct Count | Reason |
| :---: | :--- | :--- | :--- |
| 1 | 10 (simulation) | 1024 | 3 partitioned phases cover 1024 addresses total (each address read once) |
| 1 | WB_ADDR_BITS (hardware) | 2^WB_ADDR_BITS | Same — full address space covered once across all phases |
| 2 | 10 (simulation) | 3072 | 3 phases × 1024 addresses (each phase reads the full range independently) |
| 2 | WB_ADDR_BITS (hardware) | 3 × 2^WB_ADDR_BITS | Same — full range read three times |

---  

**CSR 0x5 — BIST_STATUS**

| Bit | Field | Healthy Value | Meaning |
| :---: | :--- | :---: | :--- |
| [2:0] | BIST State | 7 | See decode table below |
| [3] | Busy | 0 | BIST is issuing WB transactions (states 1–5) |
| [4] | Pass | 1 | All read comparisons matched |
| [5] | Fail Sticky | 0 | Latched on first mismatch; never clears until reset |
| [6] | Init Done | 1 | Calibration + BIST completed successfully |
| [7] | Init Failed | 0 | Training error or BIST failure |

**BIST State decode ([2:0]):**

| Value | State | Description |
| :---: | :--- | :--- |
| 0 | BIST_IDLE | Waiting for start trigger |
| 1–2 | BIST_BURST_* | Burst phase (sequential write, then read-back) |
| 3–4 | BIST_RANDOM_* | Random phase (scrambled-address write, then read-back) |
| 5 | BIST_ALT_WRITE_READ | Alternating phase (interleaved write/read) |
| 6 | BIST_FINISH | Draining final read responses |
| 7 | BIST_DONE | All phases complete (healthy steady-state) |

---

**CSR 0x6/0x7 — LANE0/LANE1_TRAINING (per-lane calibration results)**

| Bit | Field | Example | Interpretation |
| :---: | :--- | :---: | :--- |
| [8:0] | IDELAY Center | 220 | IDELAYE3 tap at center of widest passing eye. Formula: best_start (CSR 0x6) + eye_width (CSR 0x8) / 2. |
| [17:9] | WL DQS Tap | 56 | ODELAYE3 tap where DQS rising edge aligns to CK at DRAM. Starting from the BISC-calibrated 90° baseline (CSR 0x9), WL sweeps in steps of 4 until detecting a 0→1 transition on DQ. |
| [21:18] | Bitslip | 1 | ISERDES barrel-shift count aligning the deserializer word boundary to the BL8 burst start. |
| [30:22] | Best Start | 168 | First passing IDELAY tap of the widest eye window. Combined with eye_width (CSR 0x8), fully describes the passing region. |

**Cross-lane comparison:**
- Different WL DQS taps between lanes would most likely correlate (proportionally) to fly-by delay (e.g. 25 WL DQS Tap difference would mean 100 ps fly-by between lanes at 4ps/tap).
  - **fly_by_delay_ps derivation:**  = (Lane 1 WL DQS Tap (CSR 0x7) - Lane 0 WL DQS Tap(CSR 0x6)) * ps_per_tap
- Different IDELAY center/Best Start/Bitslip between lanes might not correlate with the fly-by delay. The more useful data is Eye Width at CSR 0x8.

---

**CSR 0x8 — EYE_HEALTH (read margin and PHY health)**

| Bit | Field | Example | Interpretation |
| :---: | :--- | :---: | :--- |
| [8:0] | Lane 0 Eye Width | 100 | Lane 0 width of the passing IDELAY range in taps (e.g. at 4 ps/tap = 400 ps margin). Minimum safe: 50% of bit period (e.g. DDR4-2400 bit period is 416 ps so eye_width_ps of 400ps is perfect). |
| [17:9] | Lane 1 Eye Width | 100 | For lane 1. |
| [19:18] | Rd Lat Extra | 11 | Per-lane flag: 1 = read data arrives 1 CLKDIV cycle late (high IDELAY tap pushed data past clock boundary). |
| [20] | EN_VTC | 1 | 1 = IDELAYCTRL voltage-temperature compensation is active (normal post-training). 0 during training tap updates. |

**eye_width_ps derivation:** eye_width_ps = Eye Width (CSR 0x8) * ps_per_tap

---

**CSR 0x9 — WRITE_PATH (DQ/DQS ODELAY taps)**

| Bit | Field | Example | Interpretation |
| :---: | :--- | :---: | :--- |
| [7:0] | Lane 0 WL DQ Tap | 4 | Lane 0 DQ ODELAYE3 tap after write leveling. Tracks DQS tap delta since DQS is just 90° phase-shifted from DQ |
| [15:8] | Lane 1 WL DQ Tap | 4 | For lane 1 |
| [23:16] | Lane 0 DQS Init Tap | 52 | BISC-calibrated DQS ODELAYE3 baseline (the initial 90° phase-shift tap before WL sweep). This is the tap that BISC chose to represent `DDR4_CLK_PERIOD/4` ps of delay. |
| [31:24] | Lane 1 DQS Init Tap | 52 | For lane 1. Should be very close to lane 0 (same silicon process for both ODELAYE3 primitives). |

**Sanity check:** WL DQS Tap (CSR 0x6/0x7) - DQS_Init_Tap (CSR 0x9) == WL DQ Tap (CSR 0x9)  
**ps_per_tap derivation:**  ps_per_tap = (DDR4_CLK_PERIOD/4) / DQS_Init_Tap (CSR 0x9)

---

**CSR 0xA — CONFIG (static synthesis parameters)**

| Bit | Field | Example | Interpretation |
| :---: | :--- | :---: | :--- |
| [1:0] | BIST_MODE | 2 | 0 = no BIST, 1 = partitioned address, 2 = full-range. |
| [7:4] | BYTE_LANES | 2 | Number of DQ byte lanes instantiated. |

---

**CSR 0xB — VERSION (IP revision)**

| Bit | Field | Example | Interpretation |
| :---: | :--- | :---: | :--- |
| [7:0] | Minor | 1 | Minor version number. |
| [15:8] | Major | 0 | Major version number. Currently 0.1. |

---

**CSR 0xC — CONTROL (runtime control, R/W)**

| Bit | Field | Example | Interpretation |
| :---: | :--- | :---: | :--- |
| [0] | BIST Start | 0 | W1S: write 1 to trigger a BIST run (self-clears). Read always returns 0. |
| [1] | Soft Reset | 0 | W1S: write 1 to trigger soft reset + full re-calibration (self-clears). |
| [2] | Auto Reset En | 0 | R/W: when set, a BIST failure automatically triggers soft-reset and re-calibration. |

---

**CSR 0xD — INIT_PROGRESS (boot/calibration progress tracker)**

| Bit | Field | Example | Interpretation |
| :---: | :--- | :---: | :--- |
| [5:0] | ROM Instruction Addr | 35 | Init ROM step (0–35). Values 33–35 = refresh loop (normal operation). Stuck at 22 = gate/eye training stalled. Stuck at 27 = WL stalled. |
| [6] | Pause Counter | 0 | 1 = ROM is frozen (PHY is training). 0 = ROM is advancing normally. |
| [7] | Reset Done | 1 | Init ROM completed (address 32 reached). 0 = still initializing. |
| [8] | Pipe Stall | 0 | 1 = WB pipeline is stalled waiting for PHY read data. Persistent high = PHY not returning data (check rddata_valid). |

**ROM address decode ([5:0]):**

| Value | Description |
| :---: | :--- |
| 0–21 | Power-on reset + MRS programming + ZQCL |
| 22 | Gate/eye training trigger (ROM freezes here) |
| 23–26 | Post-eye-training MRS writes |
| 27 | Write leveling trigger (ROM freezes here) |
| 28–31 | Post-WL MRS writes + final ZQCL |
| 32 | Init complete (reset_done asserts) |
| 33–35 | Periodic refresh loop (PRE ALL (34) → REF (35) → tREFI wait (33)) |

---

#### Example CSR Dump (Healthy System as based on simulation)

| Field | Value | Interpretation |
| :--- | :---: | :--- |
| PHY FSM State | 0 | PHY_IDLE (training complete) |
| Calib State | 13 | CALIB_DONE (all training passed) |
| Bank Status | 0...0 | All banks idle (refresh done) |
| Train Fail | 0/0/0 | No failures in any phase |
| Calib Retry Count | 0 | First-pass success |
| BIST FSM State | 7 | BIST_DONE (all phases complete) |
| BIST Correct Count | 3072 | 3 phases × 1024 = all matched (BIST_MODE=2) |
| BIST Error Count | 0 | Zero mismatches |
| BIST Pass | 1 | Overall PASS |
| Init Done | 1 | Calibration + BIST completed successfully |
| Lane 0/1 IDELAY | 220/220 | Symmetric eye centering |
| Lane 0/1 Eye Start | 168/168 | Symmetric first-pass tap |
| Lane 0/1 Eye Width | 104/104 | ~400 ps margin (healthy) |
| Lane 0/1 WL DQS Tap | 56/56 | Symmetric (no fly-by between lanes) |
| Lane 0/1 WL DQ Tap | 4/4 | Small delta (DQ tracks DQS) |
| Lane 0/1 DQS Init | 52/52 | BISC 90° baseline (both lanes) |
| Rd Lat Extra | 11 | Both lanes late (expected at tap 220) |
| ROM Instruction Addr | 33 | In tREFI wait (normal steady-state) |
| Reset Done | 1 | Init complete |



***

# File Structure

```
UberDDR4/
|-- rtl/
|   |-- ddr4_controller.v      # Memory controller (scheduling, timing, refresh)
|   |-- ddr4_phy.v             # UltraScale+ PHY (SERDES, IDELAY, training)
|   |-- ddr4_prober.v          # BIST engine + debug CSR register file
|   |-- ddr4_top.v             # Top-level (Wishbone interface)
|   +-- axi/
|       |-- ddr4_top_axi.v     # AXI4 top wrapper
|       |-- axim2wbsp.v        # ZipCPU AXI-to-WB bridge
|       |-- aximrd2wbsp.v      # AXI read channel bridge
|       |-- aximwr2wbsp.v      # AXI write channel bridge
|       |-- axi_addr.v         # AXI address calculator
|       |-- sfifo.v            # Synchronous FIFO
|       |-- skidbuffer.v       # Skid buffer
|       +-- wbarbiter.v        # Wishbone arbiter
|-- formal/
|   |-- ddr4_controller_formal.vh  # Formal properties (25 assertions)
|   |-- ddr4_singleconfig.sby      # SymbiYosys config (4 tasks)
|   |-- ddr4_multiconfig.sby       # Multi-config sweep (24 tasks)
|   |-- fwb_slave.v                # ZipCPU Wishbone B4 protocol monitor
|   |-- mini_fifo.v                # Pipeline oracle FIFO for formal
|   +-- f_addr_decode.v            # Independent address decode checker
|-- testbench/
|   |-- ddr4_sim_top.sv            # Top-level simulation testbench
|   |-- ddr4_model_wrapper.sv      # Micron model wrapper
|   |-- run_xsim.sh               # Vivado xsim run script
|   +-- regression_test.sh         # 8-config regression suite
+-- run_compile.sh                 # Build & verification sweep
```

***

# Acknowledgement

This project is funded through [NGI0 Entrust](https://nlnet.nl/entrust), a fund established by [NLnet](https://nlnet.nl) with financial support from the European Commission's [Next Generation Internet](https://ngi.eu) program. Learn more at the [NLnet project page](https://nlnet.nl/project/UberDDR).

[<img src="https://nlnet.nl/logo/banner.png" alt="NLnet foundation logo" width="20%" />](https://nlnet.nl)
[<img src="https://nlnet.nl/image/logos/NGI0_tag.svg" alt="NGI Zero Logo" width="20%" />](https://nlnet.nl/entrust)
