# Reference documents and implementation evidence

[Back to README](../README.md). Project behavior is defined by the checked-in
RTL and scripts. Standards explain the required memory/interface behavior;
they do not certify this implementation or every exposed configuration.

The documentation audit used these locally supplied editions. The PDFs are
external reference material and are not added to the release repository.
Page numbers below are printed page numbers unless explicitly labelled PDF.

| Reference | Edition inspected | Relevant material |
| --- | --- | --- |
| JEDEC JESD79-4D, *DDR4 SDRAM* (`ddr4.pdf`) | July 2021 | Device organization and pin functions; MR0/MR1/MR2/MR5/MR6; initialization, BL8, write leveling, MPR reads, refresh and AC timing. MR5 data mask is in Table 28, printed p.26 (PDF p.34); refresh modes in sections 4.8.2 and 4.9; normal/high-temperature tREFI in Table 171, printed p.235 (PDF p.243). |
| *DDR PHY Interface (DFI) Specification* (`DDR_PHY_Interface_Specification_v3_1 (1).pdf`) | Version 3.1, March 21, 2014 | Read-data response contract in section 3.3.3; controller/PHY frequency ratios and phase alignment in section 4.8; training handshakes and timing definitions. |
| AMD/Xilinx UG571, *UltraScale Architecture SelectIO Resources User Guide* (`ug571-ultrascale-selectio.pdf`) | v1.16, January 14, 2025 | Component SERDES/delay primitives; native BITSLICE/BITSLICE_CONTROL, reset and VTC; RIU protocol and register tables. RL_DLY_RNK is Table 2-54, p.335, including its same-MMCM/same-phase PLL-input and RIU-clock requirement. |

UG571 marks some memory RIU fields as reserved for MIG use. UberDDR4's native
PHY uses device-specific memory mechanisms and requires the supplied topology,
clock relationships and board validation; these registers are not a portable
DFI feature. Use the FPGA's applicable speed files and implementation reports
for actual timing limits.

## Evidence hierarchy

- Parameter defaults, register bits, training order, data packing and supported
  ports: `rtl/ddr4_top.v`, `rtl/ddr4_controller.v`, `rtl/ddr4_prober.v` and both
  PHY implementations. The AXI wrapper and board wrapper have distinct defaults.
- Runnable test selections and tool behavior: `run_compile.sh`, `testbench/`
  and `formal/*.sby`. Count tasks from the task list, not a progress banner.
- Historical board measurements and artifact identity:
  [HARDWARE_QUALIFICATION.md](../HARDWARE_QUALIFICATION.md). Its large ignored
  qualification artifacts are not supplied by a clean clone.
- Current Linux release/build evidence: the separately maintained
  [Linux documentation](../projects/axku3_linux/README.md) and
  [results](../projects/axku3_linux/RESULTS.md). This audit did not edit that tree.

The seller's AXKU3 schematics and MIG reference project were provenance sources
for the existing board mapping. They are not bundled prerequisites that this
release fetches automatically. The historical qualification report identifies
the relevant MIG settings and wrapper names; users porting to another board
must obtain and verify their own schematic, memory data sheet and package map.
