# Documentation audit and update record

Audit date: 2026-09-08. Source checkout: `UberDDR4-linux-release`, based on
commit `08fe013` with the pre-existing working-tree changes retained as the
comparison baseline. This is a documentation review, not a hardware signoff.

## Scope and outcome

The root documentation, standalone board example, first-party RTL comments,
formal comments and verification-script comments were reviewed against the
implementation and the [DDR4, DFI 3.1 and UG571 references](REFERENCES.md).
The follow-up audit revisited the edited material, searched for stale claims
and broken references, and checked changes against a byte snapshot taken before
editing. The inventory below accounts for all 40 previously tracked files
outside the Linux project, including files that needed no edit or are historical
artifacts. Nine new documentation/license files complete the guide set.

**No RTL or executable logic was changed.** Twenty-seven existing files were
updated: two Markdown documents and 25 files with comment-only edits. Those
25 include 16 Verilog/SystemVerilog/header files and nine shell, PowerShell,
formal-task, constraint or ignore files. All other baseline files outside the
excluded subtree are byte-identical.

`projects/axku3_linux/` was excluded from edits and from final-state comparisons
because another task is maintaining it. Links to its own instructions/results
remain the authoritative entry points. No Linux build/test command or artifact
regeneration was performed by this documentation update. External vendor
models, generated IP/build outputs, PDFs and ignored qualification artifacts
were not rewritten.

The [README](../README.md) now provides a concise orientation and directs users
to integration, architecture, debug, verification and board instructions.
Previously incorrect behavior descriptions have been corrected; implementation
limitations that need code changes are explicitly documented below.

## Corrections verified against the implementation

| Area | Corrected or added information |
| --- | --- |
| Public interface | `CWL` parameter name; actual defaults per wrapper; complete Wishbone-top connection example; separate debug port; burst-word address units, acceptance, byte selects, response order and reset use |
| Clocks and topology | Component 300 MHz delay reference; native controller/RIU same-MMCM clock contract; native per-region PLLs; AXKU3 Bank 66/67 topology; one Clocking Wizard with 300/150 MHz outputs |
| Timing and capability | Four CK/eight data UIs per controller cycle; implementation-specific DFI subset; automatic CL/CWL and CL-derived tRCD/tRP caveats; fixed 7.8 us refresh and temperature limits; board-specific hardware-rate evidence |
| Training | Component gate handshake versus eye/bitslip search; native write-level-first sequence; different PHY state maps; native write compensation=2 and startup guard=9; variable primitive-ready startup time |
| BIST | Default mode 1, partitioned input counters versus full mode 2; per-Wishbone-byte mask stress; destructive runtime ownership; startup versus rerun status; bounded recovery and diagnostic/adaptive rewrite behavior |
| Debug | CONTROL write strobes and bit-2 assignment; ignored byte selects; disabled-CSR side effects and BIST-disabled CONTROL limits; lane-dependent EYE_HEALTH bits; truncated WRITE_PATH taps; reserved 0xE/0xF; refresh ROM pointer interpretation |
| Verification | Four baseline / 30 expanded formal tasks, 26 simulation configurations and 26 lettered traffic phases; assumptions versus assertions; prove versus cover modes; script prerequisites/output lifetimes; Windows cleanup scope |
| Test comments | Repaired corrupted diagrams/punctuation; corrected model include-shim description; parameter-dependent bank/row fixtures, Phase Q bank coverage, wider-port pattern limits and fixed-delay drain behavior |
| Hardware record | Preserved dated metrics and artifact hashes; clarified absent ignored artifacts and historical source lineage; linked current board/integration/reference guides |
| Attribution | Added verbatim GPLv3 text and notice inventory; retained upstream license headers; distinguished `sfifo.v` public-domain dedication from Apache-2.0 imports |

## Follow-up verification

- Compared every baseline file outside the excluded Linux subtree. For edited
  HDL, lexical code tokens (including strings/macros/attributes) are identical;
  synthesis, Synopsys and Verilator comment directives are identical too.
  For edited scripts, SBY/XDC and ignore rules, all non-comment lines are
  identical. Pre-existing working changes were the baseline, not just Git HEAD.
- Checked all local Markdown links and linked heading anchors in the root and
  new guide set. References to the separately maintained Linux tree were checked
  for existence without editing it.
- Counted task definitions directly: 4 single-config formal tasks, 30 expanded
  formal tasks and 26 simulation matrix entries. Checked the integration
  example's parameter names and all 38 public port connections against the RTL.
- Parsed the complete documentation example with Vivado 2022.2 `xvlog -sv` in
  an isolated audit directory. This is a syntax check, not elaboration, timing
  closure or board qualification.
- Ran Bash syntax checks on all four shell entry points and a PowerShell AST
  parse on `windows_xsim_job.ps1`; all passed. `git diff --check` passed for
  the edited scope. No repository build log directory was recreated.
- Checked comment encoding after repairing malformed punctuation. Read the
  relevant local specification text and visually checked the DDR4 tREFI table,
  DFI frequency-ratio diagrams and UG571 RL_DLY_RNK clock requirement.

The byte snapshot, comparison results and temporary example compilation are
local audit working artifacts, not dependencies added to the project. No fresh
RTL simulation, formal solver campaign, implementation or board test is claimed
for this comment-only update. Historical test results retain their original
scope and dates.

## Remaining implementation and evidence limits

These are documented behavior or retained artifacts; changing them would exceed
the documentation/comment-only scope.

- The AXI wrapper remains component-only, omits separate debug connections and
  truncates its legacy upper address bit. See [Integration](INTEGRATION.md#axi-wrapper-limitations).
- CSR side-effect gating, BIST-disabled CONTROL behavior, fixed lane-0/1 detail
  registers and truncated delay counts remain as implemented. See [Debugging](DEBUGGING.md).
- Refresh does not adapt to temperature. Device geometry, clock rates, ODT and
  physical routing need configuration-specific validation. No arbitrary-PHY
  DFI compatibility or universal UltraScale speed qualification is claimed.
- Formal conclusions depend on the explicit environment/internal assumptions.
  Cover statements are present, but the committed SBY tasks use prove mode.
  Directed simulation data checks do not establish every historical timing or
  bank-coverage caption. See [Verification](VERIFICATION.md).
- Executable strings are intentionally unchanged, including the stale
  "28 tasks" progress banner, the Phase Q "16 banks" caption and some malformed
  punctuation in testbench output strings. Correct counts/coverage are recorded
  in adjacent comments and the verification guide.
- The saved `.wcfg` contains older hierarchy/experiment probes. Its view must be
  rebuilt from the current elaboration before use. It is not a batch verdict.
- The historical multi-rate qualification's large `.tmp/freq_qual/` evidence and
  generation scripts are absent from this release. Its dated results were
  preserved, not independently reproduced or reattributed to a later build.

## File inventory

"Comments" means executable tokens/lines were preserved. Imported modules were
reviewed for their role and attribution and kept intact; this does not certify
every upstream formal claim or parameter combination. Historical commit-message
files and the old waveform layout are identified rather than rewritten.

| Existing file | Disposition |
| --- | --- |
| [.gitattributes](../.gitattributes) | Reviewed; no update needed |
| [.gitignore](../.gitignore) | Comments updated; code/directives preserved |
| [.tmp/commit_msg_2026-06-21_105010.txt](../.tmp/commit_msg_2026-06-21_105010.txt) | Historical commit message; preserved |
| [.tmp/commit_msg_2026-06-22_190314.txt](../.tmp/commit_msg_2026-06-22_190314.txt) | Historical commit message; preserved |
| [HARDWARE_QUALIFICATION.md](../HARDWARE_QUALIFICATION.md) | Documentation updated |
| [README.md](../README.md) | Documentation updated |
| [example_demo/axku3/axku3_uberddr4.v](../example_demo/axku3/axku3_uberddr4.v) | Comments updated; code/directives preserved |
| [example_demo/axku3/axku3_uberddr4.xdc](../example_demo/axku3/axku3_uberddr4.xdc) | Comments updated; code/directives preserved |
| [example_demo/axku3/testbench/axku3_uberddr4_sim_top.sv](../example_demo/axku3/testbench/axku3_uberddr4_sim_top.sv) | Comments updated; code/directives preserved |
| [formal/ddr4_controller_formal.vh](../formal/ddr4_controller_formal.vh) | Comments updated; code/directives preserved |
| [formal/ddr4_multiconfig.sby](../formal/ddr4_multiconfig.sby) | Comments updated; code/directives preserved |
| [formal/ddr4_singleconfig.sby](../formal/ddr4_singleconfig.sby) | Comments updated; code/directives preserved |
| [formal/f_addr_decode.v](../formal/f_addr_decode.v) | Comments updated; code/directives preserved |
| [formal/fwb_slave.v](../formal/fwb_slave.v) | Imported helper/provenance reviewed; preserved |
| [formal/mini_fifo.v](../formal/mini_fifo.v) | Comments updated; code/directives preserved |
| [rtl/axi/axi_addr.v](../rtl/axi/axi_addr.v) | Imported helper/provenance reviewed; preserved |
| [rtl/axi/axim2wbsp.v](../rtl/axi/axim2wbsp.v) | Imported helper/provenance reviewed; preserved |
| [rtl/axi/aximrd2wbsp.v](../rtl/axi/aximrd2wbsp.v) | Imported helper/provenance reviewed; preserved |
| [rtl/axi/aximwr2wbsp.v](../rtl/axi/aximwr2wbsp.v) | Imported helper/provenance reviewed; preserved |
| [rtl/axi/ddr4_top_axi.v](../rtl/axi/ddr4_top_axi.v) | Comments updated; code/directives preserved |
| [rtl/axi/sfifo.v](../rtl/axi/sfifo.v) | Imported helper/provenance reviewed; preserved |
| [rtl/axi/skidbuffer.v](../rtl/axi/skidbuffer.v) | Imported helper/provenance reviewed; preserved |
| [rtl/axi/wbarbiter.v](../rtl/axi/wbarbiter.v) | Imported helper/provenance reviewed; preserved |
| [rtl/ddr4_controller.v](../rtl/ddr4_controller.v) | Comments updated; code/directives preserved |
| [rtl/ddr4_phy.v](../rtl/ddr4_phy.v) | Comments updated; code/directives preserved |
| [rtl/ddr4_prober.v](../rtl/ddr4_prober.v) | Comments updated; code/directives preserved |
| [rtl/ddr4_top.v](../rtl/ddr4_top.v) | Comments updated; code/directives preserved |
| [rtl/phy/ddr4_phy_native.v](../rtl/phy/ddr4_phy_native.v) | Comments updated; code/directives preserved |
| [rtl/phy/ddr4_phy_native_adapter.v](../rtl/phy/ddr4_phy_native_adapter.v) | Comments updated; code/directives preserved |
| [rtl/phy/ddr4_phy_native_byte.v](../rtl/phy/ddr4_phy_native_byte.v) | Comments updated; code/directives preserved |
| [rtl/phy/ddr4_phy_native_reset.v](../rtl/phy/ddr4_phy_native_reset.v) | Comments updated; code/directives preserved |
| [run_compile.sh](../run_compile.sh) | Comments updated; code/directives preserved |
| [testbench/ddr4_sim_top.sv](../testbench/ddr4_sim_top.sv) | Comments updated; code/directives preserved |
| [testbench/ddr4_sim_top.wcfg](../testbench/ddr4_sim_top.wcfg) | Legacy waveform view; preserved, limitations documented |
| [testbench/micron/ddr4_sdram_model_wrapper.sv](../testbench/micron/ddr4_sdram_model_wrapper.sv) | Comments updated; code/directives preserved |
| [testbench/regression_test.sh](../testbench/regression_test.sh) | Comments updated; code/directives preserved |
| [testbench/run_xsim.sh](../testbench/run_xsim.sh) | Comments updated; code/directives preserved |
| [testbench/setup_micron_model.sh](../testbench/setup_micron_model.sh) | Comments updated; code/directives preserved |
| [testbench/windows_xsim_job.ps1](../testbench/windows_xsim_job.ps1) | Comments updated; code/directives preserved |
| [testbench/xsim_batch.tcl](../testbench/xsim_batch.tcl) | Reviewed; no update needed |

New documentation/license files:

- [docs/ARCHITECTURE.md](../docs/ARCHITECTURE.md)
- [docs/DEBUGGING.md](../docs/DEBUGGING.md)
- [docs/INTEGRATION.md](../docs/INTEGRATION.md)
- [docs/VERIFICATION.md](../docs/VERIFICATION.md)
- [docs/REFERENCES.md](../docs/REFERENCES.md)
- [docs/DOCUMENTATION_AUDIT.md](../docs/DOCUMENTATION_AUDIT.md)
- [example_demo/axku3/README.md](../example_demo/axku3/README.md)
- [NOTICE](../NOTICE)
- [COPYING](../COPYING)

## Keeping the documentation current

When the implementation changes, update the relevant guide and nearby comments
together. Recheck top versus wrapper defaults, address/data widths, CSR field
positions, training-state encodings, clock relationships and BIST ownership.
Count tests from their actual task lists. Treat old output captions and dated
hardware results as evidence with a scope, not as current feature definitions.
For new hardware evidence, record the exact source/configuration, tool/device,
bitstream/LTX hashes, timing/route reports and per-attempt failures/recovery.
