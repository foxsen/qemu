# QEMU 8.2 SoftMMU TLB Study

This directory provides a reproducible x86-64 system-emulation environment for
characterizing QEMU 8.2.9 SoftMMU lookup and refill costs. It is inspired by
Tong et al., *Optimizing Memory Translation Emulation in Full System
Emulators* ([TACO DOI](https://doi.org/10.1145/2686034),
[IBM report](https://dominoweb.draco.res.ibm.com/reports/RT0956.pdf)).

`REPORT_ZH.md` gives a concise Chinese interpretation of the current evidence;
`FIXED_TLB_NOVICTIM_RESULTS_ZH.md` records the fixed-4096/no-victim attribution
experiment and its scope relative to TACO's 38.1% result; `PILOT_RESULTS.md`
retains the detailed measurements and caveats.

## Scope and Method

The published work used QEMU 1.7.0, a 256-entry direct-mapped STLB, one x86
vCPU, 1 GiB RAM, an Ubuntu 2.6.38 guest, and OProfile on an i7-4770 host. Its
workloads were Linux 3.12.9 `make -j1`, DaCapo default inputs, SPECint2006 train,
a Linux 2.6.31.4 boot, and multiprogrammed mixes. It reported 38.1% average
memory-emulation time and 24.4% average speedup from the combined changes.

This is a characterization on modern QEMU, not a bit-identical reproduction.
QEMU 8.2 already has a dynamically resized main TLB and a victim TLB. It also
inlines the fast lookup in generated host code, whereas the paper outlined
lookups and registered them separately with JVMTI. Therefore `perf` can measure
named slow-path/refill functions directly, but cannot yet separate fast lookup
instructions from the rest of each JIT-compiled guest instruction.

Two follow-on results shape the experiment design. Hong et al.'s
[dynamic-SoftTLB study](https://www.iis.sinica.edu.tw/papers/dyhong/19476-F.pdf)
argues that table size must balance hit rate against flush cost and studies
large-page-aware invalidation. Barr et al.'s
[translation-caching study](https://cs.rice.edu/CS/Architecture/docs/barr-isca10.pdf)
shows why caching partial, non-leaf translations can skip page-table levels.
The latter is hardware work, so it motivates a QEMU experiment rather than
establishing that a software PTW cache will be profitable.

## QEMU 8.2 Capacity Context

Each MMU-mode table starts at 256 entries, can shrink to 64, and on this 64-bit
x86 build has a compile-time ceiling of 2^22 entries. It remains direct mapped.
Resizing happens only when that MMU mode is flushed: occupancy above 70% doubles
the table, while sustained occupancy below 30% can shrink it after a 100 ms
window. A large ceiling therefore does not guarantee a large table during a
given phase, and oversizing trades fewer misses for slower flushes and poorer
host-cache locality.

QEMU 8.2's own `tlb_add_large_page` comment states that the TLB does not support
large-page coverage. A fill records only one `TARGET_PAGE_SIZE` region; the
reported larger page size is used to make invalidation conservative. This is
the distinction tested by the huge-page microbenchmarks below.

## Build

The profiling build adds detailed refill/PTW counters without burdening the
ordinary timing baseline:

```sh
../configure --target-list=x86_64-softmmu --enable-plugins --enable-slirp \
  --disable-capstone --enable-debug-info --disable-werror \
  --extra-cflags='-fno-omit-frame-pointer -DQEMU_TLB_PROFILE'
ninja
```

Use the same configuration without `-DQEMU_TLB_PROFILE` for timing. The current
checkout uses `build-tlb-profile/` and `build-tlb-base/` respectively.

Run the standalone harness regressions after changing collection or analysis
code:

```sh
./tlb-study/run-tests.sh
```

## Minimal Guest and Microbenchmarks

Fetch or verify the pinned public workload artifacts before building guests:

```sh
./tlb-study/fetch-workloads.py
./tlb-study/fetch-workloads.py --check
```

`build-initramfs.sh` creates a BusyBox initramfs around `guest/pagewalk-bench.c`.
The benchmark provides `dense`, `seq`, `random`, `conflict`, and `mprotect`
modes and verifies actual transparent-huge-page use through `/proc/self/smaps`.

```sh
./tlb-study/build-initramfs.sh
./tlb-study/run-profile.py --mode dense --mib 128 --passes 128 \
  --page-mode nohuge --name dense-timing --qemu build-tlb-base/qemu-system-x86_64 --perf
./tlb-study/run-profile.py --mode dense --mib 128 --passes 128 \
  --page-mode nohuge --name dense-counters
./tlb-study/analyze-results.py tlb-study/results/dense-*
```

The harness stops the VM at guest `READY`/`DONE` barriers, snapshots `info jit`
through QMP, and optionally attaches host `perf`. `perf-script.txt` retains raw
cycle-period weights so aggregation does not lose thousands of low-frequency
JIT symbols to percentage rounding. Counter runs and timing runs
must remain separate because atomic instrumentation measurably slows QEMU.
The experimental LP/PTW cache's own lookup/hit counters are an exception: they
remain in both builds so each timing run records the mechanism actually used.
Consequently, reported `on` timings conservatively include that accounting
cost; a production implementation should compile or sample those counters out.
The runner refuses a non-empty result directory; choose a fresh `--name` for
every repetition so stale `perf` data cannot be mixed with a new JSON record.

For an exact full-system memory-access denominator, build the window plugin and
use it in a separate counter run:

```sh
./tlb-study/build-mem-window.sh
./tlb-study/run-cloud-profile.py --plugin-window --name stress-window \
  --command 'stress-ng --tlb-shootdown 1 --timeout 10s'
```

The harness copies a fixed-address marker into the guest. The plugin snapshots
an inline memory counter at its start/stop PCs, so boot and shutdown accesses
are excluded. This adds one host-side increment to each guest memory operation;
do not use the same run for timing or `perf` shares. It is currently valid only
for the harness's single-vCPU configuration.

## Counter Semantics

`l1_miss` counts a miss in QEMU's direct-indexed main table; `victim_hit` is the
subset recovered by the victim table; `fill_call = l1_miss - victim_hit` except
for exceptional paths. The analyzer reports the balance residual globally and
for every origin/access pair. All 134 combinations in the current result set
have zero residual. This is an observed consistency check, not an invariant:
an atomic operation on a write-only page can request an additional read fill
without a corresponding L1 miss. Origin records distinguish generated
load/store helpers, probe paths (including instruction probes and
page-table-walk accesses), and atomic helpers. `fill_page_bits` records the
guest translation size returned by
the target MMU. The explicit benchmark access count is not a denominator for
all origins: a guest miss can cause additional probe accesses during page-table
walking.

On x86 profile builds, `ptw_walk_count` counts invocations of a paged
`mmu_translate()`, while `ptw_level_count` counts visits to page-table levels
1--5. Levels 2 and 3 can be huge-page leaves, so level visits must not be
interpreted as non-leaf visits without considering `fill_page_bits`.
`ptw_full_restart_count` records only a restart of the complete walk after a
failed final accessed/dirty-bit update; same-level compare-exchange retries are
not included. Primary guest walks and nested/NPT walks are reported separately.
For a nested row, `access` describes the NPT translation request: it can be a
store used to read or update the primary guest's paging structures, rather than
the original guest instruction's access type.

## Full Debian Guest

The base is Debian's official
[Bookworm generic cloud image](https://cloud.debian.org/images/cloud/bookworm/latest/debian-12-genericcloud-amd64.qcow2)
(SHA-256 `47bf3a3778efce65af733135aab92e5c01c05daac36d4e9a8dcded05dc1c932d`).
`build-cloud.sh` creates an 8 GiB copy-on-write overlay and NoCloud seed. The
local SSH key and disk artifacts are ignored by Git.

```sh
./tlb-study/build-cloud.sh
./tlb-study/boot-cloud.sh
./tlb-study/run-cloud-profile.py --name sysbench-memory \
  --command 'sysbench memory --memory-block-size=4K --memory-total-size=4G run'
```

For optimization comparisons, add `--snapshot` so every run starts from the
same overlay state and discards guest writes at shutdown. `--disk`, `--seed`,
`--memory`, and `--smp` make the machine configuration explicit. Each new
`result.json` records hashes for QEMU, the runner/QMP helper, the seed, copied
workload inputs, and immutable backing images. It also records the Git HEAD and
tracked-diff hash, host CPU/topology/governor, machine/perf settings, and guest
OS/package versions. A mutable non-snapshot overlay is
recorded by path, size, and timestamp but is deliberately not presented as an
immutable reproducibility identity. Measurement start/end timestamps are in
UTC; setup, QMP, perf, and workload failures retain the same provenance in a
structured `failure.json` instead of leaving only partial console logs.

Linux 3.12.9 needs a small GCC 12 compatibility shim. `-fcommon` restores the
pre-GCC-10 tentative-definition behavior, `-fno-pie` avoids a conflict with the
kernel code model, and the copied version header makes the old build system use
its existing GCC 4 attribute definitions. A paper-style single-thread build is:

```sh
./tlb-study/run-linux-build.py --mode verify
./tlb-study/run-linux-build.py --mode perf
./tlb-study/run-linux-build.py --mode profile
./tlb-study/run-linux-build.py --mode window
```

The wrapper always extracts a fresh source tree, applies the pinned patch,
runs `defconfig`, builds `vmlinux` and `bzImage` with `make -j1`, checks that
both artifacts are nonempty, and records their SHA-256 digests. `verify` is an
uninstrumented correctness run; the other modes provide independent timing,
counter, and exact memory-operation measurements.

Promote a completed run only after the workload-specific gate succeeds:

```sh
./tlb-study/validate-results.py \
  tlb-study/results/linux-3.12.9-clean-perf \
  --qemu-version 8.2.9 --require-provenance --require-linux-build
```

`--no-perfmap` is intentional for this multi-hour job: a JIT symbol map can
grow by many GiB, while the unnamed executable mappings are still classified as
guest JIT code by the analyzer. The local patch contains three
modern-toolchain compatibility fixes: the old `x86cpu` module-table declaration
and two missing `static` qualifiers in Sentelic and unlzo inline helpers. These
affect linkage only, not the kernel's intended runtime behavior. Its SHA-256 is
`a8ba819b2463a1f817f932d2bdad6dd1f63dc64444e96a87c5e589a5f23031e8`.
The wrapper also passes `-fno-stack-protector`: this is redundant in the Debian
guest but prevents Ubuntu's GCC 12 defaults from introducing an unavailable
`__stack_chk_fail` when the same source/patch is checked on the host.

The `perf` analyzer requires named SoftMMU samples to originate from the
`qemu-system-*` executable, preventing same-named host-kernel TLB functions from
being counted. Its core category includes refill, PTW, victim, atomic-access,
and TLB-maintenance functions; fast-hit instructions embedded in guest JIT code
remain outside the named share.

The cloud guest is intended for Linux kernel builds, DaCapo, `stress-ng`,
`sysbench`, and nested-QEMU experiments. SPEC CPU2006 remains external because
it is licensed and non-redistributable; the local licensed installation can be
packaged into the guest without adding its data or binaries to this repository.

## Licensed SPEC CPU2006 Inputs

`prepare-spec2006.py` reads a licensed installation without modifying it and
creates a reproducible, Git-ignored archive containing the `429.mcf`,
`471.omnetpp`, and `483.xalancbmk` train inputs. Its embedded manifest records
the exact commands and SHA-256 digest of every packaged file.

```sh
./tlb-study/prepare-spec2006.py \
  --spec-root /home/foxsen/software/spec-cpu2006
./tlb-study/run-cloud-profile.py \
  --qemu build-tlb-base/qemu-system-x86_64 --perf --no-perfmap \
  --name spec-mcf-train \
  --copy-to-workloads tlb-study/workloads/spec2006-train-x86_64.tar.xz \
  --prepare-command 'cd ~/tlb-workloads && tar -xJf spec2006-train-x86_64.tar.xz' \
  --command 'cd ~/tlb-workloads/spec2006-train/429.mcf && ./mcf inp.in'
```

Use the same extraction step with `./omnetpp omnetpp.ini` or
`./Xalan -v allbooks.xml xalanc.xsl` in the corresponding directory. Never
commit, publish, or redistribute the generated archive. `run-spec2006.py`
runs the selected train workloads sequentially in separate `perf`, counter, or
plugin-window modes; for example:

```sh
./tlb-study/run-spec2006.py --mode perf --benchmark 429.mcf
./tlb-study/run-spec2006.py --mode profile --benchmark 429.mcf
./tlb-study/run-spec2006.py --mode window --benchmark 429.mcf
```

For the fixed-table/no-victim ablation, the same wrapper also accepts
`--tlb-entries 4096 --victim-tlb off --cpu 7 --nice -20`.  The optimization
matrix names the additional workloads `omnetpp` and `xalancbmk`; omitting
`--workload` deliberately retains the original default set so that adding the
two licensed inputs cannot silently enlarge an existing experiment.

These are workload characterizations, not reportable SPEC scores: the wrapper
does not invoke the official result-validation and reporting machinery. The
cloud harness refuses to overwrite a non-empty result directory; use a suffix
such as `--name-suffix r01` for repeated SPEC samples.

`summarize-workloads.py` joins independent timing and counter directories
without comparing their elapsed times, and emits Markdown, CSV, or JSON:

```sh
./tlb-study/summarize-workloads.py \
  --manifest tlb-study/canonical-results.json
./tlb-study/validate-results.py \
  --manifest tlb-study/canonical-results.json --qemu-version 8.2.9
```

The versioned manifest prevents similarly named exploratory directories from
being selected accidentally. Use `--row LABEL PERF_DIR COUNTER_DIR` for
additional workloads and add
`--format csv` or `--format json` for machine-readable output. The analyzer
labels results as `perf_timing`, `instrumented_counter`, or
`instrumented_window`; the joiner rejects a mismatched pair and records
`wall_times_comparable=false` because counter instrumentation is explanatory,
not a timing baseline. It also rejects any cloud result whose remote command
returned nonzero. Such a failed prefix remains directly analyzable for
diagnosis, but cannot silently enter a workload comparison table.
`validate-results.py` additionally checks perf sample coverage and category
percentages, exact marker closure, refill accounting, and optional
provenance/PTW requirements before a directory is promoted into the canonical
manifest. Use `--require-nested-ptw` for virtualization experiments; it implies
the ordinary PTW checks and additionally requires a nonzero nested-stage walk.

## GAP Benchmark Suite

The open GAP Benchmark Suite adds graph kernels with large, irregular working
sets. The source archive is pinned to the v1.5 commit rather than a moving
branch. `run-gapbs.py` builds a serial kernel in the guest and generates the
Kronecker graph before the measurement barrier, so the measured interval
contains graph loading and the selected graph algorithm, but not compilation
or graph generation:

```sh
./tlb-study/run-gapbs.py --mode perf --kernel pr --scale 20
./tlb-study/run-gapbs.py --mode profile --kernel pr --scale 20
./tlb-study/run-gapbs.py --mode window --kernel pr --scale 20
```

The default scale-20 graph has 2^20 vertices and uses a 4 GiB guest. Available
kernels are `bfs`, `bc`, `cc`, and `pr`. This moderate generated graph is a
TLB-stress characterization, not a compliant full GAP result: the official
input build requires roughly 275 GB of disk and 64 GB of RAM, beyond this
host's resources. The pinned archive SHA-256 is
`b494c44636b0cbcb683d14a7d2f447f12442b5a6eb9cb1c527a1ea8c8cd7762e`.

## GraphBIG

`fetch-workloads.py graphbig` downloads the official GraphBIG v3.2 tag
(`fc1ef159238dadb1e1f3f87584cf913ddab386cc`) and verifies the pinned source
archive digest.  `run-graphbig.py` covers the seven CPU workloads used by HPCA
2027 submission #54: PageRank (`pr`), graph coloring (`gc`), shortest path
(`sssp`), triangle counting (`tc`), breadth-first search (`bfs`), connected
components (`cc`), and betweenness centrality (`bc`).  A bundled-small smoke
run invokes GraphBIG's reference-output comparison before rebuilding the normal
optimized binary:

```sh
./tlb-study/fetch-workloads.py graphbig
./tlb-study/run-graphbig.py --mode perf --kernel bfs --name-suffix smoke
./tlb-study/run-graphbig.py --mode profile --kernel pr
./tlb-study/run-graphbig.py --mode window --kernel pr
```

The bundled 1,000-vertex graph is a correctness gate, not a performance
dataset.  For a larger graph, pass a tar archive containing `vertex.csv` and
`edge.csv` at its root (or select their directory with `--dataset-subdir`), and
give it an explicit provenance label:

```sh
./tlb-study/run-graphbig.py --mode perf --kernel pr \
  --dataset-archive /path/to/graphbig-8gb.tar.xz \
  --dataset-tag synthetic-8gb --memory 20G
```

Dataset extraction and compilation occur before the measurement barrier;
loading the graph and running the selected algorithm are measured.  The 8 GB
entry in #54 does not identify a public graph or generation recipe, so matching
only its byte count would not reproduce that experiment.  This host has 30 GiB
of RAM; validate a moderate dataset first because GraphBIG's in-memory property
graph can require substantially more memory than its CSV input.  Record the
dataset source, hashes, vertex/edge counts, directedness, and archive layout
before promoting such a run.

## Experimental Large-Page and PTW Caches

The current prototype provides two opt-in miss-path experiments. Set
`QEMU_SOFTMMU_LP_CACHE=on` to consult a small large-page translation cache only
after the main and victim TLBs miss. Set `QEMU_X86_PTW_CACHE=on` to cache x86
L2--L4 non-leaf walk results. Each variable also accepts `off` (the default)
and `probe`, which records potential matches without serving them. The runners
expose the same settings as `--large-page-cache` and `--ptw-cache`.

Run repeated non-kernel comparisons with a fixed host CPU, snapshot guest, and
guest THP forced to `always`:

```sh
./tlb-study/run-optimization-suite.py \
  --workload mcf --workload omnetpp --workload xalancbmk \
  --workload gapbs --workload nested \
  --variant base --variant lp --variant ptw --repetitions 3
./tlb-study/summarize-optimizations.py \
  tlb-study/results/opt-{mcf,gapbs,nested}-{base,lp,ptw}-r0[1-3]
```

Add `--perf --perf-event cpu_core/cycles/u --no-perfmap` to collect named
SoftMMU time attribution, and use `--tlb-entries 4096 --victim-tlb off` for the
fixed-table ablation. Keep perf attribution separate from uninstrumented timing.

Use a fresh `--name-tag` when the implementation changes, and validate promoted
cloud results with `--require-provenance`. The caches are x86 system-emulation
research prototypes, not stable QEMU interfaces. Their design, correctness
boundaries, timing results, and recommended next steps are summarized in
`OPTIMIZATION_RESULTS_ZH.md`.

## Pilot Observations

Single-run smoke data are promising but are not final confidence intervals. A
128 MiB dense scan had only about 1.95% helper-load L1 misses and 1.56% refills,
yet 37.9% of sampled cycles landed in the named SoftMMU/refill classification.
A random 4 KiB-page traversal missed and refilled almost once per explicit
load; the classification reached 96.0%. With 126 MiB backed by guest 2 MiB
pages, 98.4% of installs reported a 21-bit translation, but QEMU still refilled
approximately once per 4 KiB-spaced load. This specifically motivates testing
large-page-aware SoftTLB entries rather than assuming that a large software TLB
eliminates the opportunity.

Across marker-window runs of sysbench, stress-ng, DaCapo, nested QEMU/KVM,
SPEC2006 `429.mcf`, and GAPBS PageRank, the measured data-path refill rate was
only 0.025%--0.495% per guest memory operation. Independent uninstrumented runs
nevertheless placed 1.17%--16.84% of sampled cycles in named SoftMMU slow-path
functions; `429.mcf` reached 15.22%. Low refill frequency is therefore real for
these applications, but it does not make refill cost irrelevant. See
`PILOT_RESULTS.md` for the per-workload denominators and scope.
Among main-table misses, the victim TLB recovered 44.46% for sysbench, 19.14%
for stress-ng, 83.15% for DaCapo, and 85.13% for nested QEMU; the surviving
refill fraction is therefore highly workload-dependent even with dynamic main
table sizing.

A clean Linux 3.12.9 `make -j1` completed both `vmlinux` and `bzImage` under
the uninstrumented QEMU 8.2.9 baseline in 14,674.95 wall seconds. Its 1,451,310
sample records assign 7.89% of weighted cycles to named SoftMMU work (7.13%
core and 0.75% support). The fresh-tree result passes the provenance,
QEMU-version, and Linux-build gates; see `PILOT_RESULTS.md` for hashes and the
distinction between this timing run and separate miss-counter runs.
