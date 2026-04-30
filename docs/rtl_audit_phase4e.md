# RTL Bug Hunt Audit — Phase 4E

**Design**: UberDDR4 DDR4 SDRAM Controller
**Files audited**: `rtl/ddr4_controller.v` (1519 lines), `rtl/ddr4_phy.v`, `rtl/ddr4_top.v`
**Date**: Phase 4E (pre-Phase-5 hardening)
**Methodology**: rtl-bug-hunt skill, checklists 1, 5–8, 11–13, 16, 18, 20–21

## Audit Validation Tracker

### IP-Level Checklists

| # | Checklist | Status | Files Audited | Findings | Notes |
|---|-----------|--------|---------------|----------|-------|
| 1 | Pipeline Stage Alignment | DONE | ddr4_controller.v | 0 | stage1→stage2 handoff correct; counter_q checked, counter_d loaded |
| 2 | Data Transformation Consistency | N/A | | | No data transformations — commands only |
| 3 | State Memory Enable vs Data | N/A | | | No state memory |
| 4 | Instruction Chain Integrity | N/A | | | No instruction chains |
| 5 | FSM Completeness | DONE | ddr4_controller.v | 0 | ROM FSM has default clause (L1189); refresh loop wraps correctly |
| 6 | Datapath Bit-field & Format | DONE | ddr4_controller.v | 0 | cmd_d 29-bit packing verified; MR0-MR6 vs JESD79-4D correct |
| 7 | Counter/Timer Width & Wrap | DONE | ddr4_controller.v | 0 | All MAX_* derived correctly; DELAY_COUNTER_WIDTH=20 sufficient |
| 8 | Circular Buffer Pointer Logic | DONE | ddr4_controller.v | 0 | tFAW 2-bit index wraps naturally for 4-entry array |
| 9 | Register Default & Access Type | N/A | | | No config registers in V1 |
| 10 | Clock Gating Control | N/A | | | No clock gating |
| 11 | Synthesizability | DONE | all 3 files | 1 NOTE | See NOTE-1 |

### Integration-Level Checklists

| # | Checklist | Status | Files Audited | Findings | Notes |
|---|-----------|--------|---------------|----------|-------|
| 12 | Port Connectivity | DONE | ddr4_top.v, ddr4_phy.v | 0 | DFI signals properly routed controller↔PHY |
| 13 | Signal Name Swap | DONE | ddr4_top.v | 0 | DFI BG/BA/addr bit ordering consistent |
| 14 | Interrupt Routing | N/A | | | No interrupts |
| 15 | Address Map Consistency | N/A | | | No address map |
| 16 | Bus Width Matching | DONE | ddr4_top.v | 0 | DFI_DATA_WIDTH, WB_DATA_BITS consistent across hierarchy |
| 17 | Protocol Version Compatibility | N/A | | | No AXI in V1 |
| 18 | Clock Connectivity | DONE | ddr4_top.v | 0 | controller_clk and ddr4_clk routed to correct modules |
| 19 | Reset Domain Crossing | N/A | | | Single reset domain |
| 20 | Reset Polarity | DONE | all 3 files | 0 | i_rst_n active-low consistent throughout |
| 21 | Dummy/Placeholder | DONE | ddr4_controller.v | 2 BY-DESIGN | See BY-DESIGN-1, BY-DESIGN-2 |
| 22–30 | Cross-cutting | N/A | | | No pinmux/firewall/power/NoC/APB/DFT |

## Findings

### CONFIRMED: None

No confirmed functional bugs found.

### NOTES

**NOTE-1: `default_nettype` not set in ddr4_controller.v**
- File: `rtl/ddr4_controller.v`
- `ddr4_top.v` (L30) correctly sets `` `default_nettype none``, but
  `ddr4_controller.v` does not. Since the controller is instantiated
  inside `ddr4_top`, it inherits the directive. However, standalone
  formal verification (SymbiYosys reads the .v file directly) does not
  benefit from this protection.
- Severity: LOW (code quality)
- Recommendation: Add `` `default_nettype none`` at top of controller.

### BY-DESIGN (Placeholders — Phase 5+)

**BY-DESIGN-1: o_wb_ack tied to 1'b0**
- File: `rtl/ddr4_controller.v`, L472
- `assign o_wb_ack = 1'b0;` — Phase 5 implements the read ACK pipeline.
- Tracked by TODO comment at L1042.

**BY-DESIGN-2: Calibration outputs tied inactive**
- File: `rtl/ddr4_controller.v`, L474-475
- `o_calib_complete = 1'b0`, `o_calib_error = 1'b0` — Phase 7 training pump.
- Tracked by TODO comment at L1043.

## Detailed Checklist Analysis

### Checklist 1: Pipeline Stage Alignment

The 2-stage pipeline (stage1=accept, stage2=issue) uses correct timing:
- Stage 1 latches WB request on `wb_accept` (L1091-1103)
- Stage 2 consumes stage 1 on `stage2_update` (L1073-1088)
- `stage1_pending` clears in the same cycle stage 2 consumes (L1084)
- Counter arrays use `_q` for scheduler checks (registered — current value)
  and `_d` for loading (combinational — takes effect next cycle)
- The `<= 1` optimization correctly accounts for registered cmd_d:
  scheduler fires when counter will be 0 on the DFI output cycle

No stale-data patterns found.

### Checklist 5: FSM Completeness

The ROM controller is an address-driven FSM (not a case-statement FSM):
- `instruction_address` (6-bit) indexes into `read_rom_instruction` function
- ROM case statement (L1139-1190) covers addresses 0-35 + default (L1189)
- Default returns `rom_timer(CTL_TIMER, CMD_NOP, 0)` — safe NOP
- Refresh loop wraps: addr 35 → addr 33 (L964-967)
- No deadlock: delay_counter always decrements to zero, advancing the FSM
- `reset_done` is set exactly once (addr 32, RST_DONE flag)

### Checklist 6: Datapath Bit-field & Format

cmd_d packed word (29 bits per slot):
```
[28]=cs_n, [27]=act_n, [26]=ras_n, [25]=cas_n, [24]=we_n,
[23]=odt, [22]=cke, [21]=reset_n, [20:19]=bg, [18:17]=ba, [16:0]=addr
```
- Width: 1+4+1+1+1+2+2+17 = 29 ✓ (CMD_LEN localparam)
- ACT command (L986-996): act_n=0, row bits placed in ras_n/cas_n/we_n
  positions + addr[16:0] per JESD79-4D Table 2 ✓
- WR/RD (L998-1026): A10=0 (no auto-precharge), col[9:0] at addr[9:0] ✓
- PRE (L976-983): A10=0 for single-bank, 7'b0 covers A16:A10 ✓
- MRS (L915-927): BG={0,BG0}, BA from MRS_SELECT, A13:A0 from MR value ✓
- MR0-MR6 bit fields verified against JESD79-4D Tables 13-31 ✓
- Row padding `{{(17-ROW_BITS){1'b0}}, stage2_row}` correct for <17-bit rows ✓

### Checklist 7: Counter/Timer Width & Wrap

Counter width sizing (L310-318):
- `MAX_PRECHARGE_DELAY = max(tRAS, tWR, tRTP)` — covers all precharge sources ✓
- `MAX_ACTIVATE_DELAY = tRP` — tRP is the only activate-counter source.
  The "only-raise" ACT→ACT diff-BG load (tRRD_S, typically 0-1 cycles)
  is always ≤ tRP (typically 4-5 cycles). Safe ✓
- `MAX_WRITE_DELAY = max(tRCD, RD→WR)` — covers both write-counter sources ✓
- `MAX_READ_DELAY = tRCD` — only source is ACT→RD ✓
- `MAX_CCD_DELAY = tCCD_L` ≥ tCCD_S by definition ✓
- `MAX_WTR_DELAY = tWTR_L` ≥ tWTR_S by definition ✓
- `MAX_RRD_DELAY = tRRD_L` ≥ tRRD_S by definition ✓
- `DELAY_COUNTER_WIDTH = 20` — max init timer ~150K cycles (500µs/3.3ns),
  fits in 18 bits. 20 bits provides 4× margin ✓
- `activate_timestamp_q`: `$clog2(TFAW_CYCLES)+1` bits — correctly sized ✓
- `activate_index_q`: 2-bit, wraps naturally for 4-entry circular buffer ✓

Saturating decrement `_q - (|_q)` prevents underflow: when _q=0,
`|_q`=0, so _d=0. ✓

### Checklist 8: Circular Buffer Pointer Logic

tFAW sliding window (L542-544, 617-619, 692-693, 802-803, 1063-1068):
- 4-entry `activate_timestamp_q[3:0]`, indexed by `activate_index_q[1:0]`
- Index advances on ACT or anticipation (L1067-1068)
- 2-bit natural wrap: 0→1→2→3→0 — correct for 4-entry array ✓
- Before issuing ACT: `tfaw_blocked = |activate_timestamp_q[activate_index_q]`
  — checks oldest slot. If non-zero, 4 ACTs still in window ✓
- Before issuing anticipation: `activate_timestamp_d[activate_index_q] == 0`
  — uses decremented value (post-decrement, pre-load) ✓
- No space_avail/data_avail needed — fixed 4-entry structure with overwrite ✓

### Checklist 11: Synthesizability

- `ddr4_top.v` L30: `` `default_nettype none`` ✓
- `ddr4_controller.v`: missing (see NOTE-1)
- No dual-edge clocking anywhere ✓
- `ifdef FORMAL` properly guards all formal-only code (L1514-1516) ✓
- No `#delay` or `initial` blocks in design files ✓
- No `force`/`release` or `real` types ✓
- Functions (`rom_timer`, `rom_mrs`, `read_rom_instruction`, `find_delay`,
  `ps_to_nCK`, etc.) are pure combinational — synthesizable ✓

### Checklists 12-13: Port Connectivity + Signal Name Swap

DFI internal bus in `ddr4_top.v` (L93-108):
- All DFI signals declared with correct widths matching controller outputs
- Controller→PHY signal routing verified: cs_n, act_n, ras_n, cas_n, we_n,
  cke, odt, reset_n (4-bit each), address (4×17), bank (4×BA_BITS),
  bg (4×BG_BITS), wrdata (4×DFI_DATA_WIDTH), wrdata_en (4-bit),
  wrdata_mask (4×2×BYTE_LANES), rddata_en (4-bit)
- No rd/wr, tx/rx, or BG/BA swaps found ✓
- Parameter propagation: all width-affecting params passed from top→controller
  and top→PHY consistently ✓

### Checklist 16: Bus Width Matching

- WB_DATA_BITS = DQ_BITS × BYTE_LANES × 2 × SERDES_RATIO — consistent ✓
- DFI_DATA_WIDTH = 2 × DQ_BITS × BYTE_LANES — consistent ✓
- DDR4 DQ width = DQ_BITS × BYTE_LANES — matches physical interface ✓
- No implicit width truncation found ✓

### Checklist 20: Reset Polarity

- `i_rst_n` is active-low throughout all 3 files ✓
- Controller reset block: `if (!i_rst_n)` (L815) ✓
- PHY sync reset generation from `i_rst_n` ✓
- No polarity inversions or mismatches at module boundaries ✓

### Checklist 21: Dummy/Placeholder

Systematic search for TODO/FIXME/placeholder:
- L472: `o_wb_ack = 1'b0` — Phase 5 (BY-DESIGN-1)
- L474: `o_calib_complete = 1'b0` — Phase 7 (BY-DESIGN-2)
- L475: `o_calib_error = 1'b0` — Phase 7 (BY-DESIGN-2)
- L1042: `// Phase 5: Read ACK pipeline + refresh integration` (comment only)
- L1043: `// Phase 7: Training command pump` (comment only)
- Top-level BIST ports (`i_bist_start`, `o_bist_*`) connected to
  controller stubs — Phase 8 placeholder. BY-DESIGN.

All placeholders are properly tracked with phase annotations.

## Summary

| Verdict | Count |
|---------|-------|
| CONFIRMED | 0 |
| NOTES | 1 (code quality) |
| BY-DESIGN | 2 (Phase 5/7 placeholders) |

**Conclusion**: No functional bugs found. The controller RTL is structurally
sound across all applicable checklists. The 15 formal properties proven in
Phase 4D provide strong mathematical guarantees for the scheduler logic.
One code-quality note (missing `default_nettype` in controller) recommended
for cleanup.