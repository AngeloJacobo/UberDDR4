# AXKU3 standalone DDR4 BIST example

[Back to the project README](../../README.md). This design connects UberDDR4's
native PHY to two Micron MT40A512M16LY-062E x16 devices on an ALINX AXKU3
`xcku3p-ffvb676-2-i`. Together they form a 32-bit, four-byte-lane, 2 GiB memory
interface. The committed wrapper targets DDR4-2400 with a 300 MHz controller.
It has no application master: the external Wishbone and debug inputs are tied
off and the internal BIST generates traffic.

This is a source example with a manually generated Clocking Wizard IP. It is
not a checked-in Vivado project or a command that reconstructs the exact
historical qualification images. See [HARDWARE_QUALIFICATION.md](../../HARDWARE_QUALIFICATION.md)
for their hashes, measured results and evidence limitations. The Linux SoC has
its own independently maintained [build flow](../../projects/axku3_linux/README.md).

## Create the Vivado project

1. Create an RTL project for part `xcku3p-ffvb676-2-i`. The dated hardware
   qualification used Vivado 2022.2; record the version you use.
2. Add `axku3_uberddr4.v`, the four root RTL files (`ddr4_top.v`,
   `ddr4_controller.v`, `ddr4_phy.v`, `ddr4_prober.v`) and all four
   `rtl/phy/ddr4_phy_native*.v` files. Set synthesis top to `axku3_uberddr4`.
3. Add `axku3_uberddr4.xdc` from this directory as a constraint file. It contains
   package/I/O constraints **and ILA creation/probe commands**. It is specific
   to this wrapper and its hierarchy; do not treat it as a generic pin-only XDC.
4. Create one Clocking Wizard IP named `clk_wiz_0`. Configure a 200 MHz
   single-ended input named `clk_in1` with **No Buffer**: the wrapper already
   instantiates the differential IBUFDS and BUFG. Use one MMCM to generate the
   following outputs with normal output buffers and zero-degree phase:

   | IP port name | Frequency | Connection |
   | --- | ---: | --- |
   | `ddr4_clk` | 300 MHz | `controller_clk`, native PLL input |
   | `ref300_clk` | 150 MHz | `ref_clk`, native RIU clock |

   Enable active-high `reset` and `locked` ports. The historical output names
   do not describe today's frequencies: `ddr4_clk` is the quarter-rate fabric
   clock, and `ref300_clk` is 150 MHz here. Generate the IP output products.
5. Synthesize and implement. Inspect all implementation errors and warnings,
   setup/hold/pulse-width timing, routing completeness, DRC and CDC reports.
   Check the native dedicated-clock placement and that the expected ILA probes
   resolve. Save the matching `.bit` and `.ltx` before programming the board.

The same-MMCM, same-phase controller/RIU relationship is required by the native
memory delay programming described in UG571 Table 2-54. Independent Clocking
Wizards are not an equivalent clock source. The native PHY creates local
PLLE4/CLKOUTPHY resources for two I/O clock regions; the serial 2.4 GHz clock
stays on dedicated routes. ACMD occupies Bank 66 and data Bank 67. Preserve the
pin maps, PLL maps, XDC and dedicated routing as one topology.

## Parameters and expected observations

The wrapper selects `PHY_IMPL=1`, `DEVICE_WIDTH=16`, `BYTE_LANES=4`, row width
16, column width 10 and density 8 Gbit per device. Its integer timing parameters
are 3332 ps controller and 833 ps DDR4; the generated clocks are nominally
300 MHz and 1.2 GHz CK. These rounded model inputs do not replace verification
of every derived timing against the actual device and implemented clocks.

It enables full-range BIST (`BIST_MODE=2`), disables byte-mask stress
(`BIST_DM_TEST=0`), enables the native post-failure diagnostic
(`BIST_REREAD_DIAG=1`) and disables the external CSR response
(`DEBUG_CSR_ENABLE=0`). ILA/debug attributes remain present. The nominal clean
full-range result is **0x0c000000 = 201,326,592 matching reads**, zero errors.
Diagnostic/recovery activity must also be inspected, not inferred from the LED.

LED0 reports `init_done`; all four LEDs report `init_failed` with failure taking
priority. Status clears on reset, including internal recovery. The active-low
fan control is held enabled. The pushbutton and Clocking Wizard lock control
the wrapper's reset register. Startup calibration includes DDR4 initialization,
native delay readiness, write leveling, final read training and then BIST.
Wait for actual completion; no fixed wall-clock completion time is guaranteed.

For a failed run, retain its ILA capture and timing/build identity before reset.
Inspect training state/failure flags, read-eye widths, per-lane trained mCL,
BIST comparison counts, RIU status and bounded-recovery count. The historical
qualification's final criteria are more informative than the LED alone.
BIST and its diagnostics overwrite memory.

## Board-level simulation

`testbench/axku3_uberddr4_sim_top.sv` instantiates this exact board wrapper and
Micron models for the two x16 devices. Add it as a simulation source, set it as
the simulation top, include the generated Clocking Wizard simulation products,
Xilinx primitive libraries/global module and the Micron model packages in their
required order. Use the 8-Gbit x16 model configuration and the intended speed
selection; see the source and the [generic model setup](../../docs/VERIFICATION.md).

The harness supplies the 200 MHz differential board clock, forces MICRON_SIM
for shortened simulation initialization/BIST and has a 2 ms simulated-time
watchdog. Optional diagnostic force macros bypass or alter observations and
are for isolating a failure, not qualification. The generic root XSim script
selects `ddr4_sim_top`; it does not automatically build this board-specific
Clocking Wizard harness.

## Porting or changing the rate

Change the actual generated clocks, RTL timing parameters, memory settings and
constraints consistently, then repeat implementation and hardware acceptance.
Do not infer support from parameter arithmetic alone. The dated AXKU3 campaign
qualified DDR4-1600/1866/2133/2400 and an exploratory 1250 MT/s setting. DDR4-2666
failed the device pulse-width/minimum-period check despite positive setup/hold
slack and was not programmed. These are board/device-specific results.
