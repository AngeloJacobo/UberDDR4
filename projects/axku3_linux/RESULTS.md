# Hardware status

## Current release and zero-file correction

The release source was committed as `08fe013` on 2026-09-08. The tested
bitstream SHA-256 is
`f36bffbe66c012d1becd117def9a9db0a9f380b8dab9f582b066e6f69d6aa11f`.
It passes routed timing (WNS +0.096 ns, WHS +0.010 ns, WPWS +0.039 ns),
with zero routing errors. That bitstream was built from the master controller
RTL at `00db340`; this correction required no RTL or bitstream change.

The first Linux trial failed the known-zero file hash. The file was preserved
and mapped to the final physical RAM page. A controlled test reproduced a
kernel write skipping exactly the final 32 bytes while reporting success;
direct mmap writes to those bytes passed 100 rounds.

The failed file's 30 nonzero bytes were confined to file offsets
`0x274fe0..0x274fff`, mapped to physical `0x7fffffe0..0x7fffffff`.
In this RV32 kernel, the last physical page maps at virtual `0xfffff000`.
A 4096-byte copy makes the exclusive end pointer wrap to zero. The kernel's
unrolled copy loop writes 4064 bytes, then its unsigned end check skips the
remaining 32 bytes while reporting success. This was confirmed in the exact
pinned Image's disassembly and by seeding the page through mmap, issuing a
kernel zero write, and observing that only the final 32 bytes retained the
seed. The preceding file page was a passing control.

Earlier passing tests could miss the defect if they did not allocate this
page or if the skipped bytes already contained zeros. The pinned kernel Image
SHA-256 is `fa269a349ac423417c9d617ee77fa366b5576e131fc45cb7457c1ba3b63a5d50`.
Upstream subsequently reserved this same RV32 virtual page to prevent unsafe
allocation in [Linux commit 994af1825a2a](https://github.com/torvalds/linux/commit/994af1825a2aa286f4903ff64a1c7378b52defe6).
That upstream fix describes ERR_PTR overlap; the copy-tail failure was
established separately by the reproduction above.

The device tree now reserves physical `0x7ffff000..0x7fffffff` with `no-map`,
keeping the unsafe final RV32 virtual page out of the old kernel allocator.
The explicit reservation is 4 KiB, with no bitstream or RTL change. The first
corrected boot reported 1,028 KiB more total reserved memory during early boot.
The corrected DTB SHA-256 is
`3f246e9b9ad34cc4401c019ba780cca309c5f18b97c29aa83e9ea9ed6e47de9a`.

All 42 offline checks passed after the correction: the full 41-test suite,
then the additional stale-payload guard test. Ten consecutive full FPGA
program/upload/boot/userspace trials passed with the corrected payload.
Each passed the BIOS 2 MiB and 64 MiB checks, all four payload CRC readbacks,
the exact 16 MiB known-zero SHA-256, and repeated random-file hashes.

After the final boot, three further 16 MiB known-pattern/zero-write/readback
rounds passed. All 4096 mapped file pages excluded the unsafe PFN, and Linux
explicitly reported `0x7ffff000..0x7fffffff` as reserved. The bus-error counter
was zero. The original failure captures, per-trial logs, manifests and host
verification records are retained locally in the Git-ignored
`evidence/2026-09-08-zero-file-failure/` archive. They are diagnostic artifacts
and are not required to build or run the release. The completed campaign was
`linux_uberddr4_2400_20260908_203837`; these results apply to the exact bitstream
and payload hashes above, not to subsequent unbuilt RTL revisions.

## Historical debugging branch

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
The current release build and its separate hardware results are recorded above.

## Integration checks

On 2026-09-08, this RTL-clean revision passed all 36 offline regression tests,
SoC generation, BIOS compilation (21,348 bytes within the 64 KiB limit), and
Linux payload validation using the project-local dependencies. The normal build
and payload commands passed under Windows PowerShell 5.1; batch-launcher tests
also passed under PowerShell 7, including subprocess output and failure handling.

Those 36 checks preceded commit `08fe013`; the current build and expanded
checks are recorded above. Offline checks cover wrapper wiring and host-side
integration, not physical memory reliability.

## Open issues

- With the optional startup BIST enabled in earlier Linux-sized SoCs, data
  mismatches prevented CPU release. Linux had not started. Disabling BIST
  permits boot, but does not resolve that hardware/test-path issue.
- A historical debugging-branch Linux zero-file test returned a wrong hash. The failed file was
  overwritten before diagnosis, so its cause is unknown. The current checker
  checks it immediately and preserves it on failure. Later passes do not erase
  that observation. The separately preserved release failure is diagnosed above.
- One earlier boot produced no BIOS output; a repeat worked. Its cause was
  not established. The final ten-trial campaign had no such failure.

Do not call this production-qualified or use Linux boot as proof of peak DDR
bandwidth. The pinned 2022 software bundle is an offline demonstration image,
not a maintained distribution. No power/temperature/voltage-corner or long-soak
qualification is claimed.
