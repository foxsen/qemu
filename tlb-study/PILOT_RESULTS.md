# QEMU 8.2 SoftMMU Pilot Results

Date: 2026-09-09. These are exploratory measurements, not final publication
numbers. The host was an Intel Core i7-1260P running Linux 7.0.0-31; QEMU was
pinned to logical CPU 2 (a P-core, sibling CPU 3) with the `powersave` governor.
There is no host `/dev/kvm`, so every guest below used single-threaded TCG.
The first four full-workload result pairs predate the structured provenance
extension and do not pass `validate-results.py --require-provenance`; final
experiments must rerun them rather than backfilling hashes. The newer PTW,
SPEC2006, and GAPBS directories record and pass the provenance requirements.

## Minimal Guest Results

The guest ran Debian's 6.1.0-50-amd64 installer kernel with a BusyBox initramfs.
Timing used the uninstrumented build (five repetitions). Counters used the
`QEMU_TLB_PROFILE` build (three repetitions). The `perf` share is one longer,
uninstrumented run: 512 passes and 5--8 seconds inside the exact workload
barrier.

| access pattern | guest pages | timing median (ns/access) | helper-load L1 miss/access | helper-load refill/access | named SoftMMU share |
|---|---:|---:|---:|---:|---:|
| dense, 64 B stride | 4 KiB | 7.597 | 1.948% | 1.563% | 38.05% |
| dense, 64 B stride | 2 MiB | 7.433 | 1.948% | 1.563% | not sampled |
| random, 4 KiB stride | 4 KiB | 273.337 | 100.030% | 100.010% | 95.35% |
| random, 4 KiB stride | 2 MiB | 249.790 | 100.019% | 100.012% | 95.03% |

The ratio can slightly exceed 100% because the measurement boundary includes a
small amount of barrier and reporting code. For the random case, all three runs
confirmed that 126 MiB of the 128 MiB mapping was backed by transparent huge
pages. Of the recorded TLB installations, about 98.4% carried a 21-bit page
size, yet QEMU still installed approximately one entry per 4 KiB-spaced guest
load. Thus the existing entry tag/index does not reuse one 2 MiB translation
across its 512 base pages.

Paired random runs gained 5.46--11.71% from guest huge pages (8.71% mean), even
though refill count did not fall. The likely benefit is the shorter x86 page
walk per refill. Dense paired results ranged from -14.24% to +11.35%, so no
dense speedup is established.

The nohuge `probe/store` refill count varied sharply across three random runs:
2,594, 1,970,618, and 86,484. The corresponding huge-page counts were 1, 7,
and 2. This layout-sensitive conflict behavior is a concrete target for a page
table cache, a protected PTW translation structure, or a less alias-prone hash.

## Full Debian Workloads

The persistent guest used Debian 12, kernel 6.1.0-52, one vCPU, and 1 GiB RAM.
The table separates profile-build counters from baseline-build `perf`; their
elapsed times must not be compared because the counters use atomic increments.

| workload | duration | profile L1 misses | profile refills | window data miss / mem op | window data refill / mem op | partial / elided flushes | baseline named SoftMMU share |
|---|---:|---:|---:|---:|---:|---:|---:|
| sysbench random 4 KiB writes | 10 s | 3,815,763 | 2,392,680 | 0.106% | 0.0587% | 10,727 / 32,489 | 5.21% |
| stress-ng `tlb-shootdown` | 11 s | 14,412,744 | 9,203,694 | 0.612% | 0.495% | 41,063 / 168,169 | 16.84% |
| DaCapo 9.12 `avrora`, one iteration | 63 s | 31,278,629 | 7,316,467 | 0.221% | 0.0372% | 11,743 / 35,681 | 6.61% |
| nested QEMU/KVM, random 4 KiB walk | 19 s | 31,922,148 | 9,166,424 | 0.858% | 0.128% | 130,626 / 770,988 | 13.87% |
| SPEC2006 `429.mcf` train | 41 s | 24,503,865 | 21,422,202 | 0.291% | 0.2649% | 1,699 / 7,821 | 15.22% |
| GAPBS PR scale-20, eight trials | 33 s | 13,070,428 | 1,633,778 | 0.205% | 0.0246% | 1,010 / 3,182 | 1.17% |

Conditioned on a miss in the direct-indexed main table, the victim and refill
outcomes were:

| workload | victim recovery / L1 miss | refill / L1 miss |
|---|---:|---:|
| sysbench | 44.46% | 55.54% |
| stress-ng `tlb-shootdown` | 19.14% | 80.86% |
| DaCapo `avrora` | 83.15% | 16.85% |
| nested QEMU/KVM | 85.13% | 14.87% |
| SPEC2006 `429.mcf` | 8.95% | 91.05% |
| GAPBS PR scale-20 x8 | 88.00% | 12.00% |

Thus an L1 miss is not synonymous with a page-table walk: the victim table
filters most misses in DaCapo and nested QEMU, but only one fifth in the
flush-heavy stress case. The absolute refill rate remains below 0.5% of guest
memory operations in all six runs, while the cost of the remaining slow path
is still measurable.

The capacity snapshot taken at the end barrier further bounds what “dynamic”
meant in these runs:

| workload | largest per-mode table at end | max occupancy in current resize window |
|---|---:|---:|
| sysbench | 1,024 | 664 |
| stress-ng `tlb-shootdown` | 1,024 | 662 |
| DaCapo `avrora` | 4,096 | 379 |
| nested QEMU/KVM | 1,024 | 392 |
| SPEC2006 `429.mcf` | 8,192 | 623 |
| GAPBS PR scale-20 x8 | 16,384 | 873 |

The occupancy field is not a lifetime peak: QEMU resets it when a resize occurs
or a resize window expires. The ending allocations nevertheless show that the
representative tables remained orders of magnitude below the 2^22 compile-time
ceiling. High victim recovery, especially in DaCapo and nested QEMU, also shows
that recent direct-mapped evictions remain common despite that dynamic sizing.

The two ratio columns come from separate plugin-plus-counter runs and divide
helper/atomic data-path events by all plugin-observed guest load/store
operations in the exact marker window. They exclude instruction-fetch and PTW
probe events. The first four windows contained 1.31 billion, 586 million,
11.34 billion, and 2.69 billion guest memory operations; `429.mcf` and GAPBS
added 8.04 billion and 6.32 billion. They are event rates,
not hardware-style probabilities: split or exceptional accesses can generate
more than one slow-path event. Plugin runs are not used for timing.

The first four independent `perf` runs contained 9,338, 11,654, 71,383, and
18,592 parsed sample records; `429.mcf` and GAPBS added 4,047 and 3,261.
Percentages use each record's cycle-period
weight rather than treating records as equal. These counts establish sampling
coverage, but they are not confidence intervals and do not capture run-to-run,
thermal, or phase variation.

`stress-ng` is deliberately pathological; `sysbench` demonstrates that a real
program may spend a much smaller fraction in named refill functions despite
millions of events. More importantly, all six data-path refill rates are below
0.5%, while their independent named slow-path shares range from 1.17% to
16.84%. DaCapo recovered 23,962,162 main-table misses through the victim TLB,
making victim hits much more common than refills. Fast-hit lookup instructions
remain embedded in JIT guest symbols and are absent from the named shares,
unlike the TACO paper's outlined lookup measurement.

The nested row ran Debian's QEMU 7.2 with `-accel kvm` inside the outer QEMU
8.2 TCG guest. The inner VM completed a Linux boot and 524,288 random page
accesses. Outer-QEMU refill installations comprised about 3.52 million 4 KiB,
5.60 million 2 MiB, and 6.8 thousand 1 GiB translations. It is therefore a
useful mixed-page-size case rather than merely another user-space stress loop.

## Linux 3.12.9 Clean Build

The final paper-style `make -j1` run extracted a fresh Linux 3.12.9 tree,
applied the pinned compatibility patch, ran `defconfig`, and completed both
`vmlinux` and `bzImage` under the uninstrumented QEMU 8.2.9 baseline. It took
14,674.95 wall seconds (14,646.13 QEMU CPU seconds), returned zero, and ended
with `TLB-LINUX-BUILD-PASS`. The artifact SHA-256 digests are
`2b3349a952809c57b3151edeb83f8145ff0dd1745cb7d84f1eeabca0a75a4610`
for `vmlinux` and
`4d271012bb4b1fe72fc1e2f21f563f80fe758e110c1a5a5e1aa6dc175abf6900`
for `bzImage`.

The 1,451,310 parsed sample records assign 7.89% of weighted cycles to named
SoftMMU functions: 7.13% core translation/refill work and 0.75% supporting
functions. The largest individual entries are `probe_access_internal`
(1.93%), `mmu_translate` (1.30%), `victim_tlb_hit` (0.93%), and
`tlb_set_page_full` (0.57%). Guest JIT code accounts for 46.12% and all other
host work for 45.99%. This timing run deliberately has no miss denominator;
the instrumented application and mechanism runs provide those event ratios.
During the measured build, QEMU recorded 1,438,276 partial and 4,941,500
elided TLB flushes, with no full flush.

The result directory
`results/linux-3.12.9-clean-perf-20260909-r01` passes
`validate-results.py --require-provenance --require-linux-build
--qemu-version 8.2.9`. It records the baseline QEMU SHA-256
`b157a3b318252a7d7220c51e3ff935f17f6326175efaf15019019fa4bd929956`,
the source and patch hashes, immutable guest backing chain, host topology,
runner hashes, command lines, and UTC measurement interval.

For provenance, two earlier attempts are retained only as diagnostics. The
first clean run executed for 14,359 seconds and reached the final `vmlinux`
link, where it failed with unresolved `parse_header` and `fsp_detect` symbols
caused by modern GCC inline semantics. Its 1,421,137 samples assigned 7.84% to
named SoftMMU work, closely matching the successful clean run.

An independent host build and a later QEMU recovery run also completed both
artifacts, but they are not used for timing because the former did not run
inside QEMU and the latter reused a persistent object tree. The compatibility
patch SHA-256 is
`a8ba819b2463a1f817f932d2bdad6dd1f63dc64444e96a87c5e589a5f23031e8`.

## PTW Level Smoke

The rebuilt profile binary was checked on matched 16 MiB random-page runs with
16,384 explicit accesses. With 4 KiB mappings, helper loads caused 16,502
refills and the primary x86 walker visited 82,873 levels across 16,600 walks
(4.992 levels/walk). The THP run caused essentially the same 16,507 helper-load
refills, while 14,473 installs reported a 21-bit translation. PTW work fell to
68,622 level visits across 16,619 walks (4.129 levels/walk), a 17.20% reduction
in level visits without a reduction in SoftTLB refills. Both results pass the
PTW and accounting validator. Their instrumented elapsed times are mechanism
smoke data, not baseline performance measurements.

The nested QEMU/KVM run was repeated after correcting the profiler's NPT walk
gate: QEMU 8.2 stores `nested_pg_mode` without the primary `PG_MODE_PG` bit, so
that bit alone cannot identify a nested walk. The inner VM had AMD NPT enabled
and completed 524,288 random page accesses. During the measured interval the
outer QEMU recorded 4,248,853 primary walks (19,149,367 level visits, 4.507 per
walk) and 2,079,998 nested walks (8,325,190 visits, 4.002 per walk). The result
passes `--require-nested-ptw`; it is counter evidence, not a timing comparison.

Application-level PTW profiles reinforce the mechanism result. `429.mcf`
recorded 17,064,366 walks and 72,113,447 level visits (4.226 per walk), while
the GAPBS eight-trial profile recorded 1,106,165 walks and 4,569,610 visits
(4.131 per walk). A separate `stress-ng tlb-shootdown` profile reached
8,393,357 walks at 4.814 levels per walk, including 4.995 for stores.

## Downloaded Workloads

- Linux 3.12.9 (`linux-3.12.9.tar.xz`): SHA-256
  `6a3e9f1abbaeaad34cddf0ddd69d60877765003faccf151b99d33e073566f5cb`.
- DaCapo 9.12 Bach (`dacapo-9.12-bach.jar`): SHA-256
  `33ea1a464480d486e30b172ff07787880152b8de748eb080058494d27e0fd0a9`.
- GAPBS v1.5 at commit `b5e3e19c2845f22fb338f4a4bc4b1ccee861d026`
  is pinned in `gapbs-v1.5-b5e3e19c.tar.gz` (SHA-256
  `b494c44636b0cbcb683d14a7d2f447f12442b5a6eb9cb1c527a1ea8c8cd7762e`).
  The report uses an in-guest generated scale-20 Kronecker graph because the
  official full input set exceeds the host's disk and memory.
- SPEC CPU2006 remains outside this repository because it is licensed. A local
  licensed tree is available for guest packaging; initial targets are the TLB-
  intensive `429.mcf`, `471.omnetpp`, and `483.xalancbmk` train inputs.
  `prepare-spec2006.py` generated a reproducible, Git-ignored archive with
  SHA-256 `e2ab39a27e3342131f4fc3723f4eadb1ab365124acaee04bc94b4dd33c10ae5d`.

## Interpretation

The initial hypothesis is viable but workload-dependent. A low miss fraction
does not imply a low time fraction: dense access refilled only once per 64 data
loads but spent about 38% of sampled cycles in the selected slow-path/refill
families. Conversely, general workloads can have far lower slow-path shares.
Although QEMU's main SoftTLB can theoretically grow far beyond a hardware TLB,
the representative dense/random and full-guest runs ended at only
1,024--4,096 entries. A deliberately flush-heavy `mprotect` run did drive one
MMU-mode table to 65,536 entries, confirming that large growth is possible but
policy- and workload-dependent. The direct-mapped index plus a small victim
table still permits conflict misses. The compile-time maximum therefore does
not establish that refill work is negligible in practice.

The strongest immediate experiments are (1) a large-page-aware SoftTLB entry,
(2) separating or caching PTW translations, and (3) hash/index changes aimed at
the observed layout-sensitive conflicts. Any optimization should be evaluated
on kernel build, DaCapo, sysbench, stress-ng, and nested QEMU rather than only
the synthetic random loop.

## Recommended Experiment Order

1. Add a small large-page translation cache consulted only after a main-table
   and victim miss. On a hit, synthesize the ordinary 4 KiB entry. This keeps
   the current fast-hit sequence unchanged while testing whether reusing one
   2 MiB/1 GiB translation is valuable. Reuse QEMU's existing large-page
   invalidation range and generation rules before attempting a variable-mask
   check on every access.
2. Instrument `target/i386/tcg/sysemu/excp_helper.c:mmu_translate()` by page
   table level, then add a non-leaf translation cache keyed by CR3/PCID, paging
   mode, level, and virtual-address prefix. Flush generations must cover CR3,
   `INVLPG`, global mappings, permission changes, and nested translation.
3. Test a second hash/skewed victim structure only on main-table miss. The
   highly variable nohuge probe/store counts justify this experiment, but any
   extra instruction in the common generated-code hit path needs an explicit
   dense-workload break-even test.
4. If these changes do not move end-to-end results, shift scope to TB lookup and
   chaining, generated-code quality, or MTTCG scalability. Those areas affect
   more execution than refills, but are substantially larger projects and need
   separate single-vCPU and multi-vCPU baselines.

For each step, compare an uninstrumented baseline and candidate with repeated
AB/BA ordering; use counter/plugin runs only to explain the result. Treat a
microbenchmark win without a kernel-build, DaCapo, or nested-QEMU improvement
as a mechanism demonstration rather than a system-emulation speedup.
