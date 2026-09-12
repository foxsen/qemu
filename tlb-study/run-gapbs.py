#!/usr/bin/env python3
"""Run moderate, reproducible GAPBS graph workloads under the cloud harness."""

import argparse
from pathlib import Path
import re
import shlex
import subprocess
import sys


KERNELS = {
    "bfs": ("./bfs -f {graph} -n {trials} -v > bfs.out 2> bfs.err && "
            "grep -q 'Verification: *PASS' bfs.out"),
    "bc": ("./bc -f {graph} -n {trials} -v > bc.out 2> bc.err && "
           "grep -q 'Verification: *PASS' bc.out"),
    "cc": ("./cc -f {graph} -n {trials} -v > cc.out 2> cc.err && "
           "grep -q 'Verification: *PASS' cc.out"),
    "pr": ("./pr -f {graph} -n {trials} -i 20 -v > pr.out 2> pr.err && "
           "grep -q 'Verification: *PASS' pr.out"),
}


def main():
    here = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(
        description="run a serial GAPBS kernel on a generated Kronecker graph"
    )
    parser.add_argument("--mode", choices=("perf", "profile", "window"),
                        required=True)
    parser.add_argument("--kernel", choices=KERNELS, default="pr")
    parser.add_argument("--scale", type=int, default=20)
    parser.add_argument("--trials", type=int, default=1)
    parser.add_argument("--memory", default="4G")
    parser.add_argument("--cpu", default="2")
    parser.add_argument("--perf-frequency", type=int, default=99)
    parser.add_argument(
        "--archive", type=Path,
        default=here / "workloads/gapbs-v1.5-b5e3e19c.tar.gz",
    )
    parser.add_argument("--name-suffix", default="")
    parser.add_argument(
        "--large-page-cache",
        choices=("off", "on", "probe", "adaptive"), default="off",
    )
    parser.add_argument(
        "--ptw-cache", choices=("off", "on", "probe", "adaptive"),
        default="off",
    )
    parser.add_argument(
        "--guest-thp", choices=("leave", "always", "madvise", "never"),
        default="leave",
    )
    parser.add_argument("--snapshot", action=argparse.BooleanOptionalAction,
                        default=True)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    if not 10 <= args.scale <= 23:
        parser.error("--scale must be between 10 and 23")
    if args.trials < 1:
        parser.error("--trials must be positive")
    if args.name_suffix and not re.fullmatch(
            r"[A-Za-z0-9][A-Za-z0-9._-]*", args.name_suffix):
        parser.error(
            "--name-suffix must contain only letters, digits, '.', '_' and '-'"
        )

    archive = args.archive.resolve()
    if not archive.is_file():
        parser.error(f"GAPBS archive does not exist: {archive}")
    qemu = (here.parent / ("build-tlb-base" if args.mode == "perf"
                           else "build-tlb-profile") /
            "qemu-system-x86_64").resolve()
    if not qemu.is_file():
        parser.error(f"QEMU binary does not exist: {qemu}")

    graph = f"kron{args.scale}.sg"
    suffix = f"-{args.name_suffix}" if args.name_suffix else ""
    prepare = (
        "cd ~/tlb-workloads && rm -rf gapbs-run && mkdir gapbs-run && "
        f"tar -xzf {shlex.quote(archive.name)} -C gapbs-run "
        "--strip-components=1 && "
        f"make -C gapbs-run SERIAL=1 -j1 converter {args.kernel} && "
        f"cd gapbs-run && ./converter -g {args.scale} -m -b {graph}"
    )
    guest_command = KERNELS[args.kernel].format(
        graph=graph, trials=args.trials
    )
    command = [
        sys.executable, str(here / "run-cloud-profile.py"),
        "--qemu", str(qemu),
        "--name", f"gapbs-{args.kernel}-kron{args.scale}-{args.mode}{suffix}",
        "--cpu", args.cpu,
        "--memory", args.memory,
        "--copy-to-workloads", str(archive),
        "--prepare-command", prepare,
        "--command", f"cd ~/tlb-workloads/gapbs-run && {guest_command}",
        "--large-page-cache", args.large_page_cache,
        "--ptw-cache", args.ptw_cache,
        "--guest-thp", args.guest_thp,
    ]
    if args.snapshot:
        command.append("--snapshot")
    if args.mode == "perf":
        command.extend([
            "--perf", "--perf-frequency", str(args.perf_frequency),
            "--no-perfmap",
        ])
    elif args.mode == "window":
        command.append("--plugin-window")

    print(shlex.join(command), flush=True)
    if not args.dry_run:
        subprocess.run(command, check=True)


if __name__ == "__main__":
    main()
