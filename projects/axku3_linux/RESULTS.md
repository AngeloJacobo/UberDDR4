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

## Banner payload and interactive console update

After the hardware campaign above, payload preparation was extended to append
an uncompressed 1,536-byte CPIO archive containing the LiteX/UberDDR4 banner,
the executable `uber-banner` command, and an interactive-login profile hook.
The original downloaded initramfs remains byte-for-byte intact as the first
archive. The generated DTB covers the enlarged initramfs and retains the RV32
last-page reservation. Kernel, OpenSBI and FPGA bitstream are unchanged by this
update.

The regenerated default DDR4-2400 payload has these SHA-256 hashes:

| File | SHA-256 |
| --- | --- |
| `rootfs.cpio` | `b48d88260e0411c3870de84d526d5a8c65a1b8a3c9c063eee618ca172753fbf2` |
| `rv32.dtb` | `7eecd118e731e5b2c3a22e85d849127e2fba6c1b556acca0fda22094f2a93589` |

The initramfs is 3,783,168 bytes, loaded at `0x41000000` with exclusive end
`0x4139ba00`. Repeated preparation produced identical manifests and payload
hashes. Independent CPIO extraction verified the banner contents and executable
permissions. Shell checks verified exact banner output and the interactive-only
login hook; DTB checks verified the full initramfs range and last-page reservation.
All 44 offline regression tests passed, including positive-acknowledgement,
CRC-error, missing-reply, bounded-wait and pipelined-mode recovery checks.

`console.ps1` now inserts a 5 ms pause after each transmitted byte, including
pasted input, while retaining the 1 Mbaud link and unrestricted receive output.
Host checks covered its CLI, byte preservation across multiple writes, pacing,
empty writes and short-write errors without opening a serial port.

The original upload failure recorded a roughly 400 ms host pause followed by
an `E` response. The BIOS emits `E` after 250 ms of idle time even without a
partially received frame. A controlled FPGA test using the upstream response
handler reproduced the same error by pausing the host for 400 ms between
frames. The host now handles queued timeout replies in a bounded response
window for the default single-frame transfer, still requiring a positive `K`
acknowledgement. CRC errors and unknown-command replies retain their failure
handling; pipelined transfers retain upstream behavior. Recovery counters are
recorded in `.sfl.json`.

Campaign `linux_banner_verified_20260908_234658` passed two consecutive full
program/upload/Linux tests with 251-byte packets and one outstanding packet.
The bitstream used in this campaign was SHA-256
`09ca92da1f4b3ce8557d765066b7b0bbd87330150afa9209985fba38dac9befc`,
with WNS +0.096 ns, WHS +0.010 ns, WPWS +0.039 ns and zero routing errors.
It was already built before the banner/transport changes; those changes did
not rebuild or modify it. This is a different artifact from the earlier
ten-trial bitstream recorded above.

The first trial required no timeout recovery. The second deliberately paused
the host for 400 ms, 800 ms and 400 ms after frames 100, 11000 and 35000. The
loader recovered five queued timeout replies across these three events,
retaining 251-byte packets without restarting an upload. Both trials passed
the BIOS 2 MiB and 64 MiB checks, all four complete-image CRC readbacks, Linux
boot with the automatic banner, the exact 16 MiB zero-file hash and repeated
random-file hashes. These results apply to the payload hashes in this section.

A subsequent live Linux check used the exact `console.ps1` pacing code to
transmit 3,252 command bytes, including a 2,048-byte known-content payload split
across short commands. Its SHA-256 matched
`eb076a2ec6ced9ee2e823e098446513cf5b2bb60fbcb04e6c85dc23dedaa414a`.
The check also verified `uber-banner`, the exact stored banner hash, the README's
hardware-ROM reader, the reserved last RAM page and a zero bus-error count.
A single command around 1 KiB hit the shell line editor's length limit; pacing
does not remove that limit. The README now explains using shorter lines and
prints the assembled hardware identifier in one operation to avoid interleaved
terminal echo.

One earlier reprogram after the controlled failure produced no fresh BIOS
output despite JTAG reporting success. Its cause was not established; another
reprogram restored output and the final two-trial campaign above passed. This
startup observation remains a limitation, not a claim of fault-free startup.
Completed diagnostic and qualification records, including failures, are kept
locally under the Git-ignored `evidence/2026-09-08-banner-and-uart/` folder.
See the [README](README.md) for updating the payload and using the console.

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
