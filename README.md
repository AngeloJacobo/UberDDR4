# UberDDR4

UberDDR4 is an open-source DDR4 SDRAM controller + PHY for FPGAs. It connects a
Wishbone B4 pipelined master to DDR4 memory and handles initialization, command
timing, bank scheduling, refresh and PHY calibration. It is the successor to
[UberDDR3](https://github.com/AngeloJacobo/UberDDR3).

The controller runs at **1:4 ratio**: at DDR4-2400,
the memory clock is 1.2 GHz and the controller clock is 300 MHz (thus 1:4 ratio). Each Wishbone
transfer carries one complete **BL8 burst**: a one-lane x16 memory has a 128-bit
Wishbone word and a physical 1-lane x32 memory uses a 256-bit word.

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

- 1:4 controller with two request stages for speculative PRE/ACT lookahead and actual RD/WR operation
- Sequential or bank-group-interleaved word addressing. Interleaving adjacent
  bursts can reduce same-bank-group timing penalties
- DDR4 x4, x8 and x16 device configuration in the controller. The number of
  physical byte lanes can be configured depending on your system. x4 devices do not yet support the
  byte-masked write. 
- Two Xilinx PHY implementations: component mode and native moved
- Optional BIST and a separate 32-bit debug Wishbone port.
- A AXI4-to-Wishbone wrapper based on [ZipCPU's bridge](https://github.com/ZipCPU/wb2axip/tree/master/rtl).
  
| PHY | Select | Clocking and intended use |
| --- | --- | --- |
| Component | `PHY_IMPL=0` (default) | Uses OSERDESE3/ISERDESE3/IDELAYE3/ODELAYE3 intended for DDR4-1250. Requires an external CK-rate clock (4x controller) and 300 MHz delay reference. |
| Native | `PHY_IMPL=1` | Uses BITSLICE primitives for DDR4-1250 to DDR4-2400. Requires only the 300MHz reference clock |

Component mode PHY can only run at DDR4-1250. This is just exploratory and is already below the allowed minimum DDR4 speed bin (DDR4-1333). 
The goal is to build a Xilinx PHY which uses common IO blocks (IOSERDES and IODELAY) for initial testing of UberDDR3. 
Running higher than DDR4-1250 would fail timing for these IO blocks. 

The native PHY is the one built for high DDR4 speed bins. The [Linux project](projects/axku3_linux/README.md), for example, uses the native PHY to run UberDDR4 at DDR4-2400.

## Integrate the core

Use [`rtl/ddr4_top.v`](rtl/ddr4_top.v) for Wishbone integration. Its defaults are
`DEVICE_WIDTH=8` (for x8), `BYTE_LANES=2`, `ROW_BITS=16`, `COL_BITS=10`, `DENSITY=8`,
`ADDR_MAPPING=1` (BG-interleaved), `BIST_MODE=1` (BIST passes through whole address space once), `DEBUG_CSR_ENABLE=1` (enabled CSR debugging via the Wishbone debug interface) and `PHY_IMPL=0` (component mode).
The latency overrides are `CL` and `CWL`, set this to zero to autoselect legal value based on speed bin and clock frequency.

BIST starts after calibration (gate/eye training and write leveling). Refer to [status and recovery rules](docs/DEBUGGING.md) to debug any issues due to calibration/BIST failure.

The [integration guide](docs/INTEGRATION.md) gives the complete public parameter,
clock/reset, address, data and pin contracts.

## Verify a checkout

The root verification commands use Bash on Windows. Install the tools described in the
[verification guide](docs/VERIFICATION.md), then run from the repository root:

```bash
source /path/to/Vivado/2023.1/settings64.sh
bash testbench/setup_micron_model.sh
bash run_compile.sh --sim baseline
```

The Micron model is taken from your Vivado installation and is not bundled on this repository.

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
