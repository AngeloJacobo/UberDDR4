# UberDDR4 — Known Limitations & Planned Improvements

## L1: ROW_BITS=18 Not Supported (16Gb x4 devices)

**Status:** Not implemented
**Severity:** Feature gap — design will not compile with ROW_BITS=18
**Affected devices:** 16Gb x4 DDR4 (JESD79-4D Table 7: A0~A17 = 18 row bits)

### Root cause

The DFI address bus and internal command word are hardcoded to 17 bits throughout the design.

### Blocking locations (all must be fixed)

| # | File | Line | Issue |
|---|------|------|-------|
| 1 | `ddr4_controller.v` | 95 | `CMD_LEN = 29` — address packed in `[16:0]`, only 17 bits |
| 2 | `ddr4_controller.v` | 112 | `o_dfi_address` is `4*17` bits |
| 3 | `ddr4_controller.v` | 750-752 | `{{(17-ROW_BITS){1'b0}}}` — **compile error** when ROW_BITS=18 (negative replication) |
| 4 | `ddr4_controller.v` | 1557 | `cmd_d[bank_i][16:0]` — hardcoded 17-bit extraction |
| 5 | `ddr4_phy.v` | 89 | `i_dfi_address` is `4*17` bits |
| 6 | `ddr4_phy.v` | 122 | `o_ddr4_addr` is `[16:0]` — no A17 pin |
| 7 | `ddr4_phy.v` | 244-255 | `muxed_addr` is 17-bit, no A17 path for ACT |
| 8 | `ddr4_phy.v` | 260-289 | OSERDESE3 generate loop iterates `0..16` — no 18th serializer |
| 9 | `ddr4_top.v` | 115 | `o_ddr4_addr` is `[16:0]` |
| 10 | `ddr4_top.v` | 131 | Internal DFI bus is `4*17` |

### What is already parameterized (no changes needed)

- WB address decomposition uses `+: ROW_BITS` — correct
- Pipeline row registers are `[ROW_BITS-1:0]` — correct
- Bank active row tracking is `[ROW_BITS-1:0]` — correct
- MRS initialization uses A0-A13 only — unaffected by row width
- Prober/BIST operates at WB level — unaffected

### Fix approach

Parameterize the address pin width, e.g.:
```verilog
localparam ADDR_PINS = (DEVICE_WIDTH == 4 && ROW_BITS > 17) ? 18 : 17;
```
Then replace all hardcoded `17` with `ADDR_PINS` across controller, PHY, and top.
Widen `CMD_LEN` from 29 to 30 and shift all bit-field positions up by 1.

---

## L2: ROW_BITS Never Varied in Formal or Regression

**Status:** Test gap
**Severity:** Medium — address decode correctness is only verified at ROW_BITS=16

### Current coverage

| Verification | ROW_BITS values tested |
|-------------|----------------------|
| Formal (35 tasks) | **16 only** — RTL default, never overridden in any `.sby` task |
| Regression (22 tests) | **16 only** — hardcoded `localparam ROW_BITS = 16` in `ddr4_sim_top.sv:84` |

### Formal property that checks row address

`f_addr_decode.v` cross-checks the controller's row bit-slice against an independent oracle.
This property is parameterized and would catch row decode errors at any ROW_BITS value,
but it is currently only exercised at ROW_BITS=16.

### Required additions

#### Formal

Add ROW_BITS overrides to `ddr4_multiconfig.sby` tasks. Test at least 3 values
per DEVICE_WIDTH to cover MIN and MAX:

| DEVICE_WIDTH | ROW_BITS values to test | JEDEC basis |
|-------------|------------------------|-------------|
| x4 | 15, 18 | Tables 4-7 |
| x8 | 14, 17 | Tables 4-7 |
| x16 | 14, 17 | Tables 4-7 |

#### Regression

Make `ROW_BITS` a `define`-driven parameter in `ddr4_sim_top.sv` (like `DEVICE_WIDTH` already is),
then add regression tests for ROW_BITS: 14 (x8), 17 (x16), 18 (x4).

After L1 is fixed, add `x4_row18` (DENSITY=16, ROW_BITS=18).

**Note:** The existing `density_4g` test uses DENSITY=4 but still ROW_BITS=16. A real 4Gb x8
device has 15 row bits (JESD79-4D Table 5). This is a minor inconsistency — the extra row bit
is harmless (unused rows are never activated) but does not verify correct behavior at the
actual JEDEC-specified row width.

---

## L3: Comment Inaccuracies in Parameter Documentation

Minor documentation issues found during review:

| Parameter | Current comment | Correct per JEDEC |
|-----------|----------------|-------------------|
| `ROW_BITS` | "14-17" | 14-18 (16Gb x4 uses A0~A17) |
| `COL_BITS` | "10-11 for x4" | Always 10 (A0~A9 for all configs, Tables 4-7) |
| `BG_BITS` | "JESD79-4D Table 2" | Tables 4-7 (Table 2 is the ballout table) |
| `DENSITY` | "2, 4, 8, or 16" | Correct for monolithic dies; 32Gb is DDP only |

---

## Fixed Issues

### F1: COL_LOW formula used half the DRAM column space (BYTE_LANES ≥ 2)

**Status:** Fixed
**Severity:** Critical — 50% of DRAM capacity was unreachable with BYTE_LANES=2

#### Root cause

`COL_LOW` was computed as `$clog2(SERDES_RATIO * 2 * DQ_BITS * BYTE_LANES / 8)`,
which equals `$clog2(bytes_per_burst)`. For BYTE_LANES=2 this gave COL_LOW=4,
zeroing the bottom 4 column address bits and stepping by 16 columns per burst.

DDR4 BL8 covers 8 columns per CAS command (JESD79-4D §4.3 Table 37), regardless of
bus width. All chips share the same column address — BYTE_LANES affects data width
per beat, not column granularity. Stepping by 16 instead of 8 skipped every other
set of 8 columns, making half the DRAM page inaccessible.

| BYTE_LANES | COL_LOW (old) | COL_LOW (fixed) | Capacity loss (old) |
|--|--|--|--|
| 1 | 3 | 3 | none (correct by coincidence) |
| 2 | 4 | 3 | **50%** |
| 4 | 5 | 3 | **75%** |

#### Fix

Changed formula to `$clog2(SERDES_RATIO * 2)` = 3, matching UberDDR3's proven
approach (`$clog2(serdes_ratio*2)`). This correctly represents the BL8 burst length
in columns, independent of bus width.

#### Files changed

| File | Change |
|------|--------|
| `rtl/ddr4_controller.v` | COL_LOW formula |
| `rtl/ddr4_top.v` | COL_LOW formula |
| `rtl/axi/ddr4_top_axi.v` | COL_LOW formula |
| `testbench/ddr4_sim_top.sv` | COL_LOW formula |
| `formal/f_addr_decode.v` | Default parameter 4→3 |

#### Verification

- Formal: all 4 single/multiconfig proofs pass (x4/x8/x16, map0/map1)
- Simulation: baseline test passes with Micron DDR4 model, zero violations

#### Why tests didn't catch it

BIST writes and reads using the same address mapping. Data integrity passes even
though half the columns are never touched — the Micron model does not flag unused
columns as a violation.
