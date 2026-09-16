# Architecture and training

[Back to README](../README.md). The implementation reference is the RTL;
[References](REFERENCES.md) identifies the standards behind the timing concepts.


`ddr4_top` derives PHY-specific controller timing and chooses the component or
native PHY. `ddr4_controller` schedules commands and read/write returns.
`ddr4_prober` supplies BIST, diagnostic recovery and the debug slave.
The debug path remains usable during calibration when enabled.

## Scheduling and responses

The controller accepts burst-word requests into two stages. It decodes the
requested row/bank/bank group, tracks open rows and maintains command-spacing
counters. A row miss can require PRECHARGE, ACTIVATE and then READ/WRITE.
Lookahead can precharge or activate a future bank when current timing and
ownership allow.

Per-bank and per-bank-group counters enforce the implemented tRCD, tRP, tRAS,
tRC, tCCD, tRRD, tWTR, tWR and tRTP delays. A four-activate window tracks tFAW.
The S/L distinction represents different/same bank-group constraints; the
interleaved address mapping helps sequential bursts use different groups.
The scheduler often tests a counter against 1 because its command is registered
for the next cycle. Descriptions saying every counter must reach zero before
issue are too simplistic.

A shared acknowledgment pipeline preserves request order. **Write completions
can use an earlier slot if that does not pass an earlier response. Read
completions are gated by returned PHY data and a pending-data count.** 

## Internal DFI interface

The interface follows DFI 3.1 frequency-ratio and training concepts, using
four packed command phases and eight data beats per controller clock. One
controller cycle spans **four DDR4 CK periods, or eight data unit intervals**.

Only the implemented control/data/status/training subset is connected. The
absence of other optional DFI facilities and the controller's command-slot
choices mean this should not be advertised as a complete arbitrary-PHY DFI
compliance implementation. In particular, no runtime frequency-change or
low-power handshake is exposed through `ddr4_top`.

## Initialization and refresh

The ROM holds DRAM reset low, waits with CKE low after reset release, programs
mode registers, performs ZQ initialization and waits for DLL lock. Hardware
reset/CKE waits are 200 us/500 us; `MICRON_SIM=1` shortens these and must not be
used for a physical DRAM build.

Calibration order differs by PHY:

| Stage | Component | Native |
| --- | --- | --- |
| First calibration window (ROM 22) | MPR gate/eye training | Write leveling |
| Second window (ROM 27) | Write leveling | Final MPR gate/eye training |
| After calibration | PRE ALL, refresh, init complete | Same |

The native controller path deliberately retrains reads after write leveling
because native receive alignment and the write-level handoff interact. The
native FSM includes coarse/fine gate searches, FIFO observation and handoff
states that do not exist in the component FSM.

`tREFI_ps` is fixed at 7,800,000 ps. MR settings do not implement automatic
higher-temperature refresh selection. Device density selects the refresh-cycle
delay; geometry and temperature still have to match the memory's requirements.
The parameterization does not qualify every speed/temperature combination.

## Component PHY

`rtl/ddr4_phy.v` uses OSERDESE3/ISERDESE3 and IDELAYE3/ODELAYE3. Its hardcoded
delay reference setting is 300 MHz. Reset sequencing establishes the delay
controller and serializer state before calibration.

Its eye training uses the DRAM MPR pattern to identify word alignment, sweeps
IDELAY tap values and chooses a passing range. The component eye logic checks
both on-time and one-controller-cycle-late captures so it does not merge ranges
with different capture latency. Selected late lanes are captured with the
additional delay and their return-valid timing is adjusted. Write leveling
sweeps DQS delay using DRAM DQ feedback and adjusts DQ relative to the baseline.


## Native PHY

`rtl/phy/ddr4_phy_native.v` implements calibration and application data handling.
`ddr4_phy_native_byte.v` owns each byte's BITSLICE/RIU wiring;
`ddr4_phy_native_reset.v` sequences primitive startup;
`ddr4_phy_native_adapter.v` presents the shared top-level PHY interface.

Each occupied I/O clock region uses a local PLL/CLKOUTPHY. ACMD and DQ maps
connect logical pins to physical BITSLICE positions. The RIU path crosses the
controller and RIU domains using tagged, bundled-data handshakes with settling
stages. A request/acknowledgment toggle alone is not the whole CDC contract.

The read path trains per-lane/nibble gate timing and RX eye settings, then
assembles complete BL8 words from native FIFOs. Application reads use the
trained per-lane masks; legacy qualification scratch fields are not necessarily
the masks in use.

