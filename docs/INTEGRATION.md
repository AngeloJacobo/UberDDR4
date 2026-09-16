# Integrating UberDDR4

[Back to README](../README.md). Use `rtl/ddr4_top.v` as the normal integration
boundary. It joins the controller, selected PHY and BIST/debug prober. Direct
controller-to-third-party-PHY integration requires matching the internal timing
and packing described in [Architecture](ARCHITECTURE.md); **it is not plug-and-play
DFI compliance for arbitrary PHYs**.

## Choose a configuration

Start with the [AXKU3 example](../example_demo/axku3/README.md) for hardware or
the [testbench](VERIFICATION.md) for simulation. The board uses two x16 devices:
`DEVICE_WIDTH=16`, `BYTE_LANES=4`, `ROW_BITS=16`, `COL_BITS=10`, `DENSITY=8`.
Its physical memory capacity is 2 GiB; the Linux example exposes only 1 GiB.
The default top-level configuration instead represents two x8 devices.

| Device organization | DEVICE_WIDTH | BYTE_LANES | BG_BITS | Masked writes |
| --- | ---: | ---: | ---: | --- |
| Two x8 devices | 8 | 2 | 2 | Yes |
| One x16 device | 16 | 2 | 1 | Yes |
| Two x16 devices (AXKU3) | 16 | 4 | 1 | Yes |
| Four x4 devices, paired by byte | 4 | 2 | 2 | No; use all byte selects |


## Public Wishbone-top parameters

Values below are the defaults in `ddr4_top`:

| Parameter | Default | Meaning |
| --- | --- | --- |
| `CONTROLLER_CLK_PERIOD` | 3333 | Controller period in ps; describes the supplied external clock |
| `DDR4_CLK_PERIOD` | 833 | DRAM CK period in ps; controller frequency must be CK/4 |
| `DEVICE_WIDTH` | 8 | Device DQ width: 4, 8 or 16 |
| `ROW_BITS` | 16 | Device row bits  |
| `COL_BITS` | 10 | Device column bits; three low bits belong to BL8 |
| `BYTE_LANES` | 2 | Number of physical 8-bit lanes, independent of device width |
| `DENSITY` | 8 | Per-device Gb density; 2/4/8/16 select refresh timing; does not derive ROW_BITS |
| `MICRON_SIM` | 0 | Simulation only: shortens DRAM waits and limits BIST addressing to 10 bits |
| `ADDR_MAPPING` | 1 | 0 sequential, 1 bank-group interleaved; use only these defined choices |
| `RTT_NOM` | `3'b001` | MR1 nominal ODT, RZQ/4 |
| `RTT_WR` | `3'b000` | MR2 dynamic write ODT disabled |
| `RTT_PARK` | `3'b000` | MR5 park termination disabled |
| `DRIVE_IMP` | 0 | MR1 output drive: 0 RZQ/7, 1 RZQ/5 |
| `CL` | 0 | CAS latency override; 0 uses `CL_generator` function to autocompute |
| `CWL` | 0 | CAS write latency override; 0 uses `CWL_generator` function to autocompute  |
| `PHY_IMPL` | 0 | 0 component, 1 native BITSLICE |
| `BIST_MODE` | 1 | 0 disabled, 1 partitioned range, 2 each phase covers the full address map |
| `BIST_DM_TEST` | 1 except x4 | Per-byte writes during the burst phase; automatically defaults to 0 for x4 |
| `BIST_REREAD_DIAG` | 0 | First-error diagnostic/adaptive TX path for bring-up builds |
| `DEBUG_CSR_ENABLE` | 1 | Exposes the separate debug slave's ACK/data; does not widen the DRAM address |

Native topology parameters are `PHY_ACMD_NIBBLE_COUNT` (0),
`PHY_ACMD_PIN_MAP` (all ones), `PHY_DQ_PIN_MAP` (all ones), `PHY_PLL_COUNT` (1),
`PHY_ACMD_PLL_MAP` (0) and `PHY_BYTE_PLL_MAP` (0). **The all-ones pin maps select a
canonical simulation layout, not an automatic package-pin discovery mechanism.**
Copy the board-specific maps only when using that exact pin assignment.

Keep derived parameters at their defaults: `BA_BITS`, `BG_BITS`, `DQ_BITS`,
`SERDES_RATIO`, `NUM_BG`, `NUM_BANKS`, `DFI_DATA_WIDTH`, `WB_DATA_BITS`,
`WB_SEL_BITS`, `COL_LOW` and `WB_ADDR_BITS`. The implementation assumes four
phases and eight bits per byte lane; these are not independent tuning controls.
When a generator emits parameter literals, preserve their required width. The
Linux wrapper deliberately emits a 32-bit `BYTE_LANES` constant because the
existing CONFIG CSR slices `BYTE_LANES[3:0]`.

Automatic CL/CWL at the standard tested-rate boundaries are 12/9 at 1600,
14/10 at 1866, 16/11 at 2133 and 18/12 at 2400 MT/s. The slower exploratory
1600 ps configuration selects 9/9. The RTL contains additional table branches;
their presence does not qualify higher-rate hardware. Changing CL also changes
the implemented tRCD/tRP derivation. Check the selected part's speed-bin table
and all resulting timing values before overriding it; do not optimize CL alone.

## Clock and reset contract

| Port | Component PHY (`PHY_IMPL=0`) | Native PHY (`PHY_IMPL=1`) |
| --- | --- | --- |
| `i_controller_clk` | 1:4 ratio controller/DFI clock | 1:4 ratio controller/DFI |
| `i_ddr4_clk` | External CK-rate serializer clock, 4x controller | Unused compatibility input |
| `i_ref_clk` | 300 MHz, matching the hardcoded delay primitive `REFCLK_FREQUENCY` | RIU/register-interface clock; AXKU3 at 2400 uses 150 MHz |
| `i_rst_n` | Active-low reset | Active-low reset |

For native mode, high-speed `CLKOUTPHY` is generated inside the PHY and uses
dedicated routing. It runs at the data rate (2x DRAM CK), not at the controller
frequency. Native PLL input and RIU clock must come from the same MMCM with the
same phase shift when using RL_DLY_RNK, as described in UG571 Table 2-54.

Integer ps parameters approximate the real clock periods. The board example
uses 3332/833 ps for a 300 MHz/1.2 GHz clock pair. Use a coherent rate
configuration and verify the actual generated clocks and timing reports.

## DRAM Wishbone port

All signals use `i_controller_clk`. The port is Wishbone B4 pipelined, not a
classic slave that treats a continuously asserted STB as one request.

| Signal | Direction | Meaning |
| --- | --- | --- |
| `i_wb_cyc`, `i_wb_stb`, `i_wb_we` | In | Bus cycle, request strobe, write/read direction |
| `i_wb_addr[WB_ADDR_BITS-1:0]` | In | BL8 word address |
| `i_wb_data[WB_DATA_BITS-1:0]` | In | Complete write burst |
| `i_wb_sel[WB_SEL_BITS-1:0]` | In | One active-high write enable per byte of the burst |
| `o_wb_stall` | Out | Current request cannot be accepted |
| `o_wb_ack` | Out | Completion, in accepted-request order |
| `o_wb_data[WB_DATA_BITS-1:0]` | Out | Read data valid with read ACK |

A request is accepted on an edge with `CYC && STB && !STALL`. Hold address,
direction, data and select stable while an asserted request is stalled. STB may
remain high for successive accepted requests. Keep CYC high until all accepted
requests have been acknowledged, including when STB has gaps. 
**
Read ACKs wait for PHY data. Latency varies with bank state, earlier traffic, refresh and PHY
returns so do not assume a constant request-to-ACK delay.**

## Separate debug port and status

`i_wb_dbg_*` is a separate 32-bit Wishbone port, also on the controller clock.
Its 4-bit word address selects register 0 through 15; a CPU byte mapping uses
four bytes per register. Enabled debug returns a registered one-cycle ACK and
always drives STALL low. With debug disabled, ACK remains low and data is zero;
do not issue accesses. Tie unused debug inputs to zero, including CYC/STB.

Refer to the complete [debug register map](DEBUGGING.md) before writing registers.

`o_init_done` records successful calibration and, if enabled, successful BIST.
`o_init_failed` records calibration error or a startup BIST failure. 
Both clear on internal reset/recovery as well as
external reset. **A BIST rerun after successful startup must be checked through
BIST_STATUS and comparison counts; the startup done flag is not a new-test PASS.**

## DDR4 pins and native topology

The physical port contains differential CK and DQS, bidirectional DQ,
active-low reset, CKE, CS_n, ACT_n, ODT, BA/BG, 17 address/command pins and one
DM_n output per byte lane. **During ACT, high row bits share the RAS/CAS/WE pins**;
the PHY produces the DDR4 multiplexed pins. **The interface is single-rank and
BL8. No ECC bus, CA-parity/ALERT recovery interface, DBI datapath or dynamic
frequency-change interface is exposed by this top level.**

A native ACMD map uses one byte per logical pin, `{physical_nibble[4:0], position[2:0]}`.
DQ maps use four bits per logical DQ, `{upper_nibble, position[2:0]}`. PLL maps
select a local clock source per physical ACMD nibble or byte. The AXKU3 mapping
spans Bank 66 for ACMD and Bank 67 for data, using two PLL clock regions. DDR4
pins do not all have to be in a single bank. Match the package topology, XDC,
I/O standards, VREF/DCI and native dedicated routes together.

Include `ddr4_top.v`, `ddr4_controller.v`, `ddr4_prober.v`, `ddr4_phy.v`, and the
four `rtl/phy/ddr4_phy_native*.v` files for the supplied native flow. The example
also needs its board wrapper, XDC and generated `clk_wiz_0` IP.

## AXI wrapper 

`rtl/axi/ddr4_top_axi.v` translates AXI4 requests into the Wishbone data path
using ZipCPU's `axim2wbsp`. AXI uses byte addresses and the bridge strips the
word-offset bits. AXI data width equals the full Wishbone burst width; ID width
defaults to 4. The wrapper defaults to periods 3336/834 ps and BIST mode 1.

This retained wrapper has not been updated to expose the full Wishbone top:

- It does not forward `PHY_IMPL` or native pin/PLL maps, so it selects the
  component PHY by default. It also does not expose the top's ODT/CL/CWL/diagnostic knobs.
- It leaves the separate debug ports disconnected. There is no AXI CSR window.
- With `DEBUG_CSR_ENABLE=1`, it adds an address bit but connects that address
  directly to the narrower DRAM port. The added upper bit is discarded and the
  upper half aliases DRAM; it does not select debug registers.
- The bridge's Wishbone error input is tied low. Do not expect that connection
  to report physical DDR corruption.


## Complete component-PHY connection example

This wrapper shows every `ddr4_top` port for one x16, 8-Gbit device with a
128-bit Wishbone master. It illustrates the interface only: supply phase-related
CK/quarter-rate clocks, a 300 MHz delay reference, reset sequencing and your
board's pin/electrical constraints. These are inputs here, not generated IP.
For a native hardware design, use the complete AXKU3 wrapper and its topology
instead. BIST and debug responses are explicitly disabled in this example.

```verilog
module memory_connection (
    input  wire controller_clk, ddr4_clk, ref300_clk, rst_n,
    input  wire wb_cyc, wb_stb, wb_we,
    input  wire [25:0] wb_addr,       // 16 row + 1 BG + 2 BA + 10 col - 3
    input  wire [127:0] wb_wdata,
    input  wire [15:0] wb_sel,
    output wire wb_stall, wb_ack,
    output wire [127:0] wb_rdata,
    output wire init_done, init_failed,
    output wire ck_p, ck_n, reset_n, cke, cs_n, act_n, odt,
    output wire [16:0] addr,
    output wire [1:0] ba, dm_n,
    output wire bg,
    inout  wire [15:0] dq,
    inout  wire [1:0] dqs_p, dqs_n
);
    ddr4_top #(
        .CONTROLLER_CLK_PERIOD(3336), .DDR4_CLK_PERIOD(834),
        .DEVICE_WIDTH(16), .ROW_BITS(16), .COL_BITS(10),
        .BYTE_LANES(2), .DENSITY(8), .ADDR_MAPPING(1),
        .CL(0), .CWL(0), .MICRON_SIM(0), .PHY_IMPL(0),
        .BIST_MODE(0), .BIST_DM_TEST(0), .BIST_REREAD_DIAG(0),
        .DEBUG_CSR_ENABLE(0)
    ) memory (
        .i_controller_clk(controller_clk), .i_ddr4_clk(ddr4_clk),
        .i_ref_clk(ref300_clk), .i_rst_n(rst_n),
        .i_wb_cyc(wb_cyc), .i_wb_stb(wb_stb), .i_wb_we(wb_we),
        .i_wb_addr(wb_addr), .i_wb_data(wb_wdata), .i_wb_sel(wb_sel),
        .o_wb_stall(wb_stall), .o_wb_ack(wb_ack), .o_wb_data(wb_rdata),
        .i_wb_dbg_cyc(1'b0), .i_wb_dbg_stb(1'b0), .i_wb_dbg_we(1'b0),
        .i_wb_dbg_addr(4'b0), .i_wb_dbg_data(32'b0), .i_wb_dbg_sel(4'b0),
        .o_wb_dbg_stall(), .o_wb_dbg_ack(), .o_wb_dbg_data(),
        .o_ddr4_ck_p(ck_p), .o_ddr4_ck_n(ck_n), .o_ddr4_reset_n(reset_n),
        .o_ddr4_cke(cke), .o_ddr4_cs_n(cs_n), .o_ddr4_act_n(act_n),
        .o_ddr4_addr(addr), .o_ddr4_ba(ba), .o_ddr4_bg(bg),
        .o_ddr4_odt(odt), .o_ddr4_dm_n(dm_n), .io_ddr4_dq(dq),
        .io_ddr4_dqs_p(dqs_p), .io_ddr4_dqs_n(dqs_n),
        .o_init_done(init_done), .o_init_failed(init_failed)
    );
endmodule
```


## Refresh and Operating assumptions

The controller uses normal 1x refresh with `tREFI_ps=7_800_000` (7.8 us) and
has no automatic temperature-dependent refresh switch. JESD79-4D Table 171
requires a shorter interval above 85 C (3.9 us through 95 C for the stated
range). 

The supplied design assumes BL8, one rank, the documented preambles and mode
register choices. Memory speed tables, tAA/tRCD/tRP, ODT, physical DQS wiring,
I/O bank voltage and package routing all remain part of board integration.
