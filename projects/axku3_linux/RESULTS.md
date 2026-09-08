# Hardware status

On 2026-09-08, the earlier debugging-branch Linux configuration passed ten consecutive FPGA
reprogram/upload/boot/userspace trials at DDR4-2400. Each trial checked:

- the BIOS memory test and an additional 64 MiB memory preflight;
- CRC32 readback of the kernel, device tree, initramfs and OpenSBI in DDR4;
- Linux boot, root shell and controller status;
- an exact-length 16 MiB zero file against its expected SHA-256, followed by
  repeated matching hashes of a 16 MiB random file.

Eight more 16 MiB zero-file checks passed on the final boot. This is evidence
for the recorded workload, not a guarantee for arbitrary traffic or conditions.

| Item | Recorded configuration |
| --- | --- |
| CPU | VexRiscv-SMP, one RV32 core with Sv32 MMU, 300 MHz |
| RAM | 1 GiB visible; x32 DDR4 at 2400 MT/s |
| Software | Linux 5.14.0, OpenSBI 0.8, Buildroot initramfs |
| Console | 1 Mbaud physical UART |
| Routed slack | WNS +0.062 ns, WHS +0.010 ns, WPWS +0.039 ns |
| Debug | Controller CSRs; no ILA; startup BIST disabled |

Recorded bitstream SHA-256:
`88cfaaa2d09c4e896d29aa78e78482c2d19f8cdb25bcd734144ad89a88bff229`

These are historical results, not qualification of the current cleanup.
The example now uses the original master RTL without the added debug ports,
BIST address override or altered BIST patterns. The SoC wrapper sizes the
BYTE_LANES parameter explicitly instead of modifying the controller's CSR logic.
A fresh build and hardware campaign are required before assigning results to it.

## Integration checks

On 2026-09-08, this RTL-clean revision passed all 36 offline regression tests,
SoC generation, BIOS compilation (21,348 bytes within the 64 KiB limit), and
Linux payload validation using the project-local dependencies. The normal build
and payload commands passed under Windows PowerShell 5.1; batch-launcher tests
also passed under PowerShell 7, including subprocess output and failure handling.

This revision is being left uncommitted for review; no new synthesis, timing
closure or hardware trial is claimed. These checks cover wrapper wiring and
host-side integration, not physical memory reliability.

## Open issues

- With the optional startup BIST enabled in earlier Linux-sized SoCs, data
  mismatches prevented CPU release. Linux had not started. Disabling BIST
  permits boot, but does not resolve that hardware/test-path issue.
- An earlier Linux zero-file test returned a wrong hash. The failed file was
  overwritten before diagnosis, so its cause is unknown. The current checker
  checks it immediately and preserves it on failure. Later passes do not erase
  that observation.
- One earlier boot produced no BIOS output; a repeat worked. Its cause was
  not established. The final ten-trial campaign had no such failure.

Do not call this production-qualified or use Linux boot as proof of peak DDR
bandwidth. The pinned 2022 software bundle is an offline demonstration image,
not a maintained distribution. No power/temperature/voltage-corner or long-soak
qualification is claimed.
