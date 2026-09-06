# AXKU3 DDR4 hardware qualification

Qualification date: 2026-09-06 (Asia/Singapore)  
Device: AXKU3, `xcku3p-ffvb676-2-i`  
Tool: Vivado 2022.2

## Result

The native PHY passed ten fresh FPGA programming and initialization trials at
each requested standard data rate (DDR4-1600, DDR4-1866, DDR4-2133, and
DDR4-2400). The exploratory DDR4-1250 configuration also passed ten fresh
trials. Every accepted trial completed `0x0c000000` (201,326,592) comparisons
with zero errors and had a coherent final ILA state.

| Rate | tCK | Controller clock | Final timing WNS/WHS/WPWS | Routing | Hardware | Final trained mCL |
| --- | ---: | ---: | ---: | ---: | ---: | --- |
| DDR4-2400 | 833 ps | 300 MHz | +0.066 / +0.010 / +0.039 ns | 47,201/47,201, 0 errors | 10/10 | `492492` |
| DDR4-2133 | 938 ps | 266.667 MHz | +0.189 / +0.010 / +0.065 ns | 47,008/47,008, 0 errors | 10/10 | `410410` |
| DDR4-1866 | 1071 ps | 233.333 MHz | +0.193 / +0.010 / +0.099 ns | 47,166/47,166, 0 errors | 10/10 | `38e38e` |
| DDR4-1600 | 1250 ps | 200 MHz | +0.350 / +0.010 / +0.143 ns | 47,283/47,283, 0 errors | 10/10 | `30c30c`, or valid per-lane `2cc30c` |
| DDR4-1250 | 1600 ps | 156.25 MHz | +0.633 / +0.010 / +0.231 ns | 47,293/47,293, 0 errors | 10/10 | `249249` |

DDR4-1250 is not claimed as a JEDEC speed bin. It was tested because the
seller MIG project declares `C0.DDR4_MAX_PERIOD=1600`, and it is reported
separately from the four standard rates.

## Maximum native-PHY rate

DDR4-2400 is the highest timing-supported standard rate for this native PHY
on the AXKU3 `xcku3p-ffvb676-2-i`. A ceiling build at DDR4-2666 used the
current RTL, a 333.333 MHz quarter-rate controller clock, a 750 ps DDR4 clock
period, and a 2666.667 MHz `CLKOUTPHY`. Implementation closed ordinary setup
and hold timing at WNS `+0.006 ns` and WHS `+0.009 ns`, with all 47,243
routable nets complete and no routing errors.

The build nevertheless fails the device pulse-width/minimum-period gate on 90
endpoints. The `-2` speed file requires `TX_BITSLICE/CLK` period >= 3.195 ns,
while DDR4-2666 provides 3.000 ns, for WPWS `-0.195 ns` and TPWS
`-17.550 ns`. This is a characterized primitive limit, not a fabric path that
placement or routing can repair. The corresponding DDR4-2400 clock period is
3.333 ns and passes the same check by `+0.138 ns`.

The parallel-clock primitive limit corresponds to approximately 2504 MT/s;
because the next JEDEC bin above DDR4-2400 is DDR4-2666, DDR4-2400 is the
maximum supported standard rate. The DDR4-2666 bitstream was not programmed
or hardware-tested. Its diagnostic artifacts use the stem
`artifacts/ddr2666_ceiling_status` and are retained only as negative timing
evidence.

## Acceptance criteria

A trial was accepted only when its final saved ILA capture satisfied all of
the following, rather than relying on the board PASS indication alone:

- terminal PASS, `init_done=1`, `init_failed=0`;
- BIST state `7`, calibration state `d`, and calibration result `1`;
- correct count `0c000000` and error count `00000000`;
- PHY state `0` and all write-level, read-eye, and read-gate failure bits zero;
- no pending application read, due capture, return, FIFO-nonempty, or
  read-data-valid state at completion;
- all 29 physical delay/VTC-ready bits set (`1fffffff`);
- if adaptive TX-eye ran: done=1, update-error=0, pass-found=1, and
  restore-pending=0;
- if RIU was used: state/req/ack/error idle, and control data equal to the
  tagged readback value;
- on builds containing the bounded recovery probe, recovery count was in
  range 0..15 (all accepted final trials completed with count 0).

All 50 saved final captures passed these checks. Adaptive TX-eye ran in all ten
DDR4-1600 trials and two DDR4-1866 trials. At DDR4-1866 the two RIU updates
both ended with control/readback `0x0433/0x0433`. DDR4-2133, DDR4-2400, and
DDR4-1250 did not require adaptation.

### Interpretation of trained versus legacy consensus fields

The qualification-only signal names `qual_status_app_upper/lower` can be
misleading. Those registers are retained legacy consensus scratch values and
are not used by the production application read-gate datapath. The actual
application mask is calculated from each lane's `gate_trained_mcl` or
`gate_trained_mcl_low` in `rtl/phy/ddr4_phy_native.v`.

Consequently, three DDR4-1600 trials legitimately report per-lane trained
values `2cc30c` while the unused consensus scratch remains `30c30c`. The
hardware applied the former, all training-failure bits were zero, and each
trial completed the full memory comparison without error. Requiring these
unrelated fields to be equal would incorrectly reject valid lane-independent
training.

## Exact artifacts and evidence

The generated files are under `.tmp/freq_qual/` and are intentionally ignored
by Git because of their size. Preserve both the `.bit` and matching `.ltx` for
each image.

| Rate | Artifact stem | BIT SHA-256 | LTX SHA-256 | Ten-run evidence directory |
| --- | --- | --- | --- | --- |
| 2400 | `artifacts/ddr2400_riu_tagged_status` | `EE4F5698476900D26C0DB058CC770D3DBCDEEBB69CB9317514CF9C5C58D2ABE4` | `61B2038A1C3A3DE66B516146DFF8D76C70567F2CCE76597BC9218050BCACF97B` | `hw_ddr2400_10fresh/` |
| 2133 | `artifacts/ddr2133_riu_tagged_status` | `490AF2A1CE0BFFA47DF1B984100CA01FE100CC5C0B8265F840EB03C7FA21953A` | `DD6D82431525274C5455DBA3495BDFED4643028DFD9A8E851382D4F711356197` | `hw_ddr2133_10fresh/` |
| 1866 | `artifacts/ddr1866_autorecover_status` | `2964198E52668D06B4492838280D4CE5A5E7516041FE3CD25B751AD9C7BEF76A` | `25891D3F68D38B92D073B96114E6448304ACF1AB4DFF244239C2C4C115FB5E45` | `hw_ddr1866_autorecover_status_10fresh/` |
| 1600 | `artifacts/ddr1600_autorecover_status` | `28A4C5CFBB289BBF79481EE1C4F7E13F309876CB881B5CF5E0A960166823A901` | `E22E5E7DA093FE6EAE2CF6A8270E5687D122DAA56820F4597DFDA1493E823F35` | `hw_ddr1600_autorecover_status_10fresh/` |
| 1250 | `artifacts/ddr1250_autorecover_status` | `D40728B47361F53D8F963190741B13DDA1CCAA724547397531F87C3249013481` | `2BF5A9D765C71451ABBF67E588F46E0A7FCF5882FAFFD95AC09912063C6BA63C` | `hw_ddr1250_autorecover_status_10fresh/` |

Each evidence directory contains `trial_01.csv` through `trial_10.csv`. The
matching timing, route-status, DRC, and CDC reports use the artifact stem plus
`_timing_summary.rpt`, `_route_status.rpt`, `_drc.rpt`, and `_cdc.rpt`.

DDR4-2133 and DDR4-2400 were built and qualified on the RIU-tagged source
lineage before the later lower-rate recovery additions. The later changes are
confined to RIU bundled-data reliability, retry bookkeeping, and diagnostic
failure recovery; neither high-rate ten-run set exercised those paths.

## Defects found and fixed

- `22e0966` tagged RIU readback transactions so stale acknowledgements could
  not be mistaken for the current absolute delay load.
- `395eb00` allowed RIU arbitration to outlast XiPHY BISC maintenance.
- `a70ec45` made the RIU bundled-data CDC lossless by letting payload/tag data
  settle before the synchronized request/response event is consumed.
- `016dc80` added bounded (maximum 15) full retraining for every terminal BIST
  diagnostic failure, and exposed the recovery count to the final ILA image.

Rejected intermediate results are not included in the qualification totals.
They included a DDR4-1866 9/10 set with an RIU timeout, DDR4-1600 sets with RIU
timeouts, and a DDR4-1600 diagnostic failure that revealed the missing
diagnostic-to-full-retrain path. The final results above were collected only
after the corresponding fixes and clean implementation gates.

## References used

- AMD/Xilinx UG571 v1.16, *UltraScale Architecture SelectIO Resources User
  Guide*: `ug571-ultrascale-selectio.pdf`. RIU protocol was checked on pages
  325-327 and `NIBBLE_CTRL0` on page 329.
- Local DDR4 specification: `ddr4.pdf`.
- DFI specification: `DDR_PHY_Interface_Specification_v3_1 (1).pdf`.
- Seller board material: `C:\Users\ajacobo\Downloads\AXKU3\AXKU3`.
- Seller MIG reference project: `C:\Users\ajacobo\Downloads\AXKU3\ddr4_test`.
- Seller MIG configuration:
  `ddr4_test.srcs\sources_1\ip\ddr4_core\ddr4_core.xci`, including
  `C0.DDR4_MAX_PERIOD=1600`, `C0.DDR4_tCK=750`, and speed grade `075E`.
- Seller XiPHY wrapper references under
  `ddr4_test.gen\sources_1\ip\ddr4_core\ip_1\rtl\xiphy_files`, especially
  `ddr4_phy_v2_2_xiphy_bitslice_wrapper.sv` and
  `ddr4_phy_v2_2_xiphy_control_wrapper.sv`.
