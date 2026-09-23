# AXKU3 standalone DDR4 BIST example
[<img width="812" height="443" alt="image" src="https://github.com/user-attachments/assets/297231c0-babb-4068-91e5-85a6320c0043" />](https://youtu.be/Y6qFC9ROYH8?si=Y6DaOi6-dCAke-K-)

[Back to the project README](../../README.md). This design connects UberDDR4's
native PHY to two Micron MT40A512M16LY-062E x16 devices on an ALINX AXKU3
`xcku3p-ffvb676-2-i`. Together they form a 32-bit, four-byte-lane, 2 GiB memory
interface. The committed wrapper targets DDR4-2400 with a 300 MHz controller.
It has no application master: the external Wishbone and debug inputs are tied
off and the internal BIST generates traffic.

This is a source example, not a checked-in Vivado project. Neither the Makefile
below nor the manual steps reconstruct the exact historical qualification
images. See [HARDWARE_QUALIFICATION.md](../../HARDWARE_QUALIFICATION.md) for
their hashes, measured results and evidence limitations. The Linux SoC has its
own independently maintained [build flow](../../projects/axku3_linux/README.md).

## Build from the command line

`make` in this directory runs the whole flow in batch mode and writes
`build/axku3_uberddr4.bit`:

```
make                      # synthesis, implementation and bitstream
make synth                # stop after synthesis and its reports
make program              # program the board over JTAG with the built bitstream
make clean                # delete build/
```

`build.tcl` creates the project, generates the `clk_wiz_0` IP with the
configuration the next section describes by hand, then synthesizes, implements
and checks routed timing. Negative setup, hold or pulse-width slack stops the
build before a bitstream is written; `make ALLOW_FAILING_TIMING=1` writes one
anyway, for debugging only. Every output, including `build/vivado.log`, stays in
`build/`, which Git ignores.

Vivado must be on PATH, or name it with `make VIVADO=/path/to/vivado`. Windows
has no make of its own; Vivado ships GNU Make in `gnuwin\bin`, and that build
runs recipes through `cmd.exe` whichever shell starts it, so the recipes stay
within what `cmd.exe` and a POSIX shell both accept. A make that does use a
POSIX shell on Windows will not find `vivado.bat` by name alone and needs
`make VIVADO=vivado.bat`.

The reports the next section tells you to inspect are written to `build/` by
both paths, and are as worth reading after a batch build as after a GUI one:
`post_route_timing.rpt`, `post_route_drc.rpt`, `post_route_cdc.rpt`,
`post_route_utilization.rpt` and `post_route_clock_utilization.rpt`.

## Create the Vivado project

These are the equivalent manual steps, for working in the GUI. `build.tcl`
performs the same ones.

1. Create an RTL project for part `xcku3p-ffvb676-2-i`. The dated hardware
   qualification used Vivado 2022.2; record the version you use.
2. Add `axku3_uberddr4.v`, the four root RTL files (`ddr4_top.v`,
   `ddr4_controller.v`, `ddr4_phy.v`, `ddr4_prober.v`) and all four
   `rtl/phy/ddr4_phy_native*.v` files. Set synthesis top to `axku3_uberddr4`.
3. Add `axku3_uberddr4.xdc` from this directory as a constraint file. Its pin
   maps are specific to this wrapper and its hierarchy; do not treat it as a
   generic pin-only XDC.
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
   Check the native dedicated-clock placement. Save the matching `.bit` before
   programming the board.

## Parameters and expected observations

The wrapper selects `PHY_IMPL=1`, `DEVICE_WIDTH=16`, `BYTE_LANES=4`, row width
16, column width 10 and density 8 Gbit per device. Its integer timing parameters
are 3332 ps controller and 833 ps DDR4; the generated clocks are nominally
300 MHz and 1.2 GHz CK. These rounded model inputs do not replace verification
of every derived timing against the actual device and implemented clocks.

It enables full-range BIST (`BIST_MODE=2`), disables byte-mask stress
(`BIST_DM_TEST=0`), enables the native post-failure diagnostic
(`BIST_REREAD_DIAG=1`) and disables the external CSR response
(`DEBUG_CSR_ENABLE=0`). The nominal clean full-range result is
**0x0c000000 = 201,326,592 matching reads**, zero errors.

LED0 reports `init_failed`; all remaining three LEDs report `init_done`. 
The active-low fan control is held enabled. . Startup calibration includes DDR4 initialization,
native delay readiness, write leveling, final read training and then BIST.

This wrapper carries no on-chip observability: it builds no debug cores and
`DEBUG_CSR_ENABLE=0` leaves the CSR port unanswered, so the LEDs are the only
runtime status the board reports. The earlier debug cores in this example were
removed because they could not meet timing at 300 MHz.


## Board-level simulation

`testbench/axku3_uberddr4_sim_top.sv` instantiates this exact board wrapper and
Micron models for the two x16 devices. Add it as a simulation source, set it as
the simulation top, include the generated Clocking Wizard simulation products,
Xilinx primitive libraries/global module and the Micron model packages in their
required order. Use the 8-Gbit x16 model configuration and the intended speed
selection; see the source and the [generic model setup](../../docs/VERIFICATION.md).

