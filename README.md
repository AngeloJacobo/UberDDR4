# UberDDR4

UberDDR4 is an open-source DDR4 SDRAM controller for FPGA designs. It connects a
Wishbone B4 pipelined master to DDR4 memory and handles initialization, command
timing, bank scheduling, refresh and PHY calibration. It is the successor to
[UberDDR3](https://github.com/AngeloJacobo/UberDDR3).

The controller runs at one quarter of the DDR4 clock frequency: at DDR4-2400,
the memory clock is 1.2 GHz and the controller clock is 300 MHz. Each Wishbone
transfer carries one complete BL8 burst. A physical x16 interface uses a 128-bit
Wishbone word; a physical x32 interface uses a 256-bit word.

## Start here

| What you want to do | Read |
| --- | --- |
| Run the supplied RISC-V Linux system on AXKU3 | [Linux example](projects/axku3_linux/README.md) |
| Bring up AXKU3 with BIST and LEDs, without a CPU | [AXKU3 hardware example](example_demo/axku3/README.md) |
| Connect your own Wishbone master or adapt a board | [Integration guide](docs/INTEGRATION.md) |
| Run simulation, lint or formal checks | [Verification guide](docs/VERIFICATION.md) |
| Understand scheduling, DFI and PHY training | [Architecture](docs/ARCHITECTURE.md) |
| Read status registers or diagnose a failure | [Debugging and BIST](docs/DEBUGGING.md) |
| Check what has actually been demonstrated | [Native-PHY hardware results](HARDWARE_QUALIFICATION.md), [Linux results](projects/axku3_linux/RESULTS.md) |

## Capabilities and limits

- Quarter-rate controller with two request stages, bank/bank-group timing
  counters, and speculative precharge/activate lookahead.
- Sequential or bank-group-interleaved word addressing. Interleaving adjacent
  bursts can reduce same-bank-group timing penalties; performance still depends
  on the workload, open rows, refresh and turnarounds.
- DDR4 x4, x8 and x16 device configuration in the controller; the number of
  physical byte lanes is a separate parameter. x4 devices do not support the
  byte-masked write path. Parameterization is not qualification of every board
  topology or combination of geometry and clock rate.
- Two Xilinx PHY implementations, selected by `ddr4_top.PHY_IMPL`.
- Optional destructive BIST and a separate 32-bit debug Wishbone port.
- A retained AXI4-to-Wishbone wrapper based on ZipCPU's bridge. Its current
  [integration limitations](docs/INTEGRATION.md#axi-wrapper-limitations) include
  missing native-PHY configuration forwarding and debug-port connections.

| PHY | Select | Clocking and intended use |
| --- | --- | --- |
| Component | `PHY_IMPL=0` (default) | OSERDESE3/ISERDESE3 and IDELAYE3/ODELAYE3; external CK-rate clock (4x controller) and 300 MHz delay reference. Used by the default simulation flow. |
| Native | `PHY_IMPL=1` | BITSLICE primitives, local PLL high-speed clocks and a separate RIU clock. Used by the supplied AXKU3 hardware and Linux examples. |

The historical AXKU3 native-PHY campaign recorded ten fresh programming/BIST
trials at each of DDR4-1600, 1866, 2133 and 2400, plus exploratory DDR4-1250.
DDR4-2666 failed the device minimum-period/pulse-width gate and was not programmed.
See the [exact artifacts, acceptance criteria and limits](HARDWARE_QUALIFICATION.md).
These results do not qualify other devices, boards, configurations or operating
conditions. The controller uses a fixed nominal 7.8 us refresh interval; it does
not automatically select a higher-temperature refresh rate.

The [Linux example](projects/axku3_linux/README.md) uses a 300 MHz VexRiscv CPU,
LiteX and 1 GiB of CPU-visible RAM. Its pinned RV32 kernel needs the documented
last-page device-tree reservation. Startup BIST is disabled in that integration;
[RESULTS.md](projects/axku3_linux/RESULTS.md) distinguishes current and historical
hardware evidence and unresolved issues.

## Integrate the core

Use [`rtl/ddr4_top.v`](rtl/ddr4_top.v) for Wishbone integration. Its defaults are
`DEVICE_WIDTH=8`, `BYTE_LANES=2`, `ROW_BITS=16`, `COL_BITS=10`, `DENSITY=8`,
`ADDR_MAPPING=1`, **`BIST_MODE=1`**, `DEBUG_CSR_ENABLE=1` and `PHY_IMPL=0`.
The latency overrides are named **`CL` and `CWL`**; zero selects automatic values.

BIST starts after calibration when enabled and overwrites its test range. Disable
it explicitly when a destructive startup test is unsuitable, and quiesce all
masters before a runtime retrigger. A final PASS can follow automatic recovery;
read the [status and recovery rules](docs/DEBUGGING.md) before interpreting it.

The [integration guide](docs/INTEGRATION.md) gives the complete public parameter,
clock/reset, address, data and pin contracts. The separate debug port does not
add an address bit to the Wishbone DRAM interface.

## Verify a checkout

The root verification commands use Bash. Install the tools described in the
[verification guide](docs/VERIFICATION.md), then run from the repository root:

```bash
source /path/to/Vivado/2023.1/settings64.sh
bash testbench/setup_micron_model.sh
bash run_compile.sh --sim baseline
PHY_IMPL=native bash run_compile.sh --sim baseline
```

The Micron model is taken from your Vivado installation and is not bundled.
The testbench contains 26 lettered traffic phases, with separate BIST, CSR and
failure paths. The regression matrix contains 26 configurations. Formal jobs
check the controller under the harness assumptions; they are not a proof of the
complete PHY, AXI wrapper or physical memory interface.

## Repository map

| Path | Contents |
| --- | --- |
| `rtl/ddr4_top.v` | Wishbone integration, PHY selection and BIST ownership mux |
| `rtl/ddr4_controller.v` | Command scheduler, initialization/refresh ROM and DFI data pipeline |
| `rtl/ddr4_prober.v` | BIST, bounded recovery and debug registers |
| `rtl/ddr4_phy.v` | Component PHY |
| `rtl/phy/` | Native PHY, per-byte primitives, reset sequencer and adapter |
| `rtl/axi/` | Retained AXI wrapper and ZipCPU bridge/support modules |
| `formal/` | Controller properties, SBY task files and helper models |
| `testbench/` | Micron-model testbench, setup and simulation runners |
| `example_demo/axku3/` | Native-PHY BIST/LED board example, XDC and board testbench |
| `projects/axku3_linux/` | LiteX/VexRiscv Linux integration, host tools and evidence |
| `docs/` | Integration, architecture, verification, debug and reference guides |

The [documentation audit record](docs/DOCUMENTATION_AUDIT.md) lists the reviewed
files, corrected claims, verification and remaining implementation limits.

## License and references

Core RTL files carry GPLv3 notices, generally allowing GPLv3 or later; see
[COPYING](COPYING) and each file's header. The imported ZipCPU modules retain
Apache-2.0 or, for `sfifo.v`, public-domain notices. [NOTICE](NOTICE) describes provenance and externally
supplied dependencies. [References](docs/REFERENCES.md) identifies the DDR4,
DFI 3.1 and UG571 editions used to review this documentation.

## Acknowledgement

This project is funded through [NGI0 Entrust](https://nlnet.nl/entrust), a fund
established by [NLnet](https://nlnet.nl) with financial support from the European
Commission's [Next Generation Internet](https://ngi.eu) program. See the
[NLnet project page](https://nlnet.nl/project/UberDDR).
