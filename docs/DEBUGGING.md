# Debugging and BIST

[Back to README](../README.md). The separate debug Wishbone slave in
`ddr4_prober.v` exposes controller, PHY and BIST status. It runs on
`i_controller_clk`, uses a **word address** from 0 to 15, returns a registered
one-cycle ACK and never asserts STALL when enabled. A CPU byte offset is four
times the word address. Registers are 32 bits; unlisted read bits are zero.

`DEBUG_CSR_ENABLE=0` suppresses ACK/data, but the existing CONTROL write decode
is not independently gated by that parameter. Tie all unused debug inputs to
zero. Byte selects are ignored. This interface is not accessible through an
extra DRAM address bit or through the retained AXI wrapper.

## Register map

Let `BL = BYTE_LANES`. Bit ranges are inclusive, with lane 0 in the low bits.
Detailed tap registers describe only lanes 0 and 1. Four-lane designs need ILA
or other external probes for the remaining detailed taps. The fixed lane-1
slices also mean this CSR layout is not a general one-lane implementation.

| Word / byte offset | Name | Read fields |
| --- | --- | --- |
| 0x0 / 0x00 | STATUS | [3:0] PHY state; [7:4] controller calibration state; [8] stage 1 pending; [9] stage 2 pending; [11] stage 2 write; [12] refresh idle |
| 0x1 / 0x04 | BANK_STATUS | [NUM_BANKS-1:0] open-row flags, indexed as `BG*4 + BA` |
| 0x2 / 0x08 | TRAIN_FAIL | [BL-1:0] gate failure; [2*BL-1:BL] eye failure; [3*BL-1:2*BL] write-level failure; [3*BL+1:3*BL] calibration retry count |
| 0x3 / 0x0c | CORRECT_COUNT | Full 32-bit BIST matching-read count |
| 0x4 / 0x10 | ERROR_COUNT | Full 32-bit BIST mismatching-read count |
| 0x5 / 0x14 | BIST_STATUS | [2:0] BIST state; [3] busy; [4] pass; [5] failure latch; [6] init done; [7] init failed |
| 0x6 / 0x18 | LANE0_TRAINING | [8:0] read center; [17:9] write-level DQS tap; [21:18] bitslip; [30:22] best eye start |
| 0x7 / 0x1c | LANE1_TRAINING | Same fields for lane 1 |
| 0x8 / 0x20 | EYE_HEALTH | [8:0] lane-0 eye width; [17:9] lane-1 eye width; [18+BL-1:18] extra read-latency flags; [18+BL] VTC enabled |
| 0x9 / 0x24 | WRITE_PATH | [7:0] lane-0 DQ TX tap; [15:8] lane-1 DQ TX tap; [23:16] lane-0 DQS baseline; [31:24] lane-1 DQS baseline |
| 0xa / 0x28 | CONFIG | [1:0] BIST mode; [7:4] byte-lane count |
| 0xb / 0x2c | VERSION | [7:0] minor=1; [15:8] major=0 (register-interface version 0.1, not a source revision) |
| 0xc / 0x30 | CONTROL | [2] automatic BIST-failure recovery enabled; [1:0] read as zero |
| 0xd / 0x34 | INIT_PROGRESS | [5:0] ROM instruction pointer; [6] training pause; [7] ROM reset done; [8] pipeline stalled |
| 0xe / 0x38, 0xf / 0x3c | Reserved | Zero |

For two lanes, EYE_HEALTH uses [19:18] for extra latency and [20] for VTC;
for four lanes these move to [21:18] and [22]. WRITE_PATH stores only the low
eight bits of each nine-bit delay count: values at or above 256 wrap in that
readback. Use the full lane probes for quantitative tap analysis. Delay counts
and training widths are PHY-specific; do not interpret every count as the same
number of picoseconds across PHYs, settings or operating conditions.

When `BIST_MODE=0`, BIST_STATUS [5:0] and the comparison counters read zero;
init status remains available. CONTROL readback still defaults to bit 2 set,
but its write handler is compiled out, including software reset.

## CONTROL writes and memory ownership

For a BIST-enabled build, bit 0 requests a BIST start and bit 1 requests full
controller/PHY recalibration. Both are write-one strobes. **Every CONTROL write
also assigns bit 2**, which is an ordinary read/write recovery enable, initially
one. Write a full word because byte selects do not mask writes.

| Value | Effect when BIST is enabled |
| --- | --- |
| 0x0 | Disable automatic recovery |
| 0x4 | Enable automatic recovery |
| 0x1 / 0x5 | Start BIST with recovery disabled / enabled |
| 0x2 / 0x6 | Request recalibration with recovery disabled / enabled |

Before a runtime BIST or reset, stop application requests and wait for all
accepted requests to complete. The top-level BIST mux does not drain an active
application master for you. BIST writes memory and destroys previous contents;
its optional failure diagnostic can change TX settings and restart destructive
tests. Do not request it on memory containing live software or other needed data.

## What BIST covers

The three phases are sequential burst writes/reads, stress-addressed
writes/reads, and alternating write/read transactions. `stress_addr()` spreads
counter bits across row/bank fields. Phase partitions refer to the **input
counter ranges**; they do not promise three disjoint physical memory regions
after the stress mapping.

The address counter width is `MICRON_SIM ? 10 : WB_ADDR_BITS`. Let
`N = 2**BIST_ADDR_BITS`.

| Mode | Phase counter lengths (burst / stress / alternating) | Expected matching reads on a clean run |
| --- | --- | --- |
| 0 | Disabled | 0 |
| 1 | N/4, N/2, N/4 | N |
| 2 | N, N, N | 3*N |

`BIST_DM_TEST=1` writes each byte position of a complete Wishbone burst
separately during the burst-write phase. This means 16 selections for a
128-bit port or 32 for a 256-bit port, not just one write per physical lane.
It increases writes, not the expected comparison count. x4 devices do not have
the implemented DM_n byte-mask function; leave this test disabled for x4.

BIST states are 0 idle, 1 burst write, 2 burst read, 3 stress write, 4 stress
read, 5 alternating write/read, 6 finish/drain and 7 done. Busy covers states
1 through 6. Pass requires state 7 and no BIST failure latch.

Counts and failure state are reset for a new run and by internal resets. The
startup `init_done` latch is not cleared merely by requesting a new BIST, so
software must observe the new run's busy/done transition, pass bit and counts.
`init_failed` can subsequently assert on a calibration error even if
`init_done` was already set; do not treat the flags as universally exclusive.

Automatic BIST-failure recovery requests full retraining and is limited to
15 requests until external reset. Its counter survives the internal resets it
requests, but is an ILA/debug probe, not a register in this CSR map. A final
PASS alone therefore cannot establish that the first attempt passed.
`BIST_REREAD_DIAG` first repeats reads at the failing address; native diagnostic
support can then explore TX delay, rewrite/retest and recover. Consistent or
varying rereads are diagnostic evidence, not a definitive attribution to a
particular DRAM, write or receive fault.

## Interpreting training progress

Controller states are 0 IDLE; 1..4 gate enable/read/wait/exit; 5..8 eye
enable/read/wait/exit; 9..12 write-level enable/strobe/wait/exit; 13 DONE;
14 ERROR. The numeric ordering is not necessarily execution order: the native
configuration performs write leveling before its final gate/eye training.

| PHY state | Component meaning | Native meaning |
| ---: | --- | --- |
| 0 | Idle | Idle |
| 1 | Gate done | Gate done |
| 2..5 | Eye sweep, track, decide, verify | Eye sweep, track, decide, verify |
| 6 | Eye late check | Eye verify done |
| 7 | Eye done | Eye done |
| 8..12 | WL sample, adjust, check, done, apply | WL sample, adjust, check, done, apply |
| 13..15 | Unused | Eye observe, center, rewind |

Read INIT_PROGRESS together with STATUS. The ROM increments its pointer when
an instruction starts, then holds while its delay timer counts down. In the
normal refresh loop, instruction 33 is PRECHARGE ALL, 34 is REFRESH, and 35 is
the tREFI countdown NOP. The pointer has wrapped to **33 during that countdown**;
33 is therefore the expected refresh-idle pointer even though the instruction
stored at 33 is PRECHARGE ALL. Initialization uses steps 0..32; the middle
training windows depend on the selected PHY. See [Architecture](ARCHITECTURE.md).

If startup fails, capture clock locks, native delay/VTC readiness, ROM pointer,
controller/PHY states, TRAIN_FAIL, retry count, BIST counters and recovery count
before resetting. A zero-width eye points to a training observation failure;
a BIST mismatch after training requires data-path investigation too. For native
reads, inspect per-lane `gate_trained_mcl` / `gate_trained_mcl_low`, not the
legacy consensus scratch fields. Check tagged RIU request/ack/readback completion
when TX adaptation ran. Compare full ILA evidence and timing reports against
the [historical qualification criteria](../HARDWARE_QUALIFICATION.md), and retain
failed runs as well as passes.
