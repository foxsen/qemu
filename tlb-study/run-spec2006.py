#!/usr/bin/env python3
"""Run selected SPEC CPU2006 train workloads under the cloud harness."""

import argparse
from pathlib import Path
import re
import shlex
import subprocess
import sys


BENCHMARKS = {
    "429.mcf": ("429-mcf", "./mcf inp.in > inp.out 2> inp.err"),
    "471.omnetpp": (
        "471-omnetpp", "./omnetpp omnetpp.ini > omnetpp.log 2> omnetpp.err"
    ),
    "483.xalancbmk": (
        "483-xalancbmk",
        "./Xalan -v allbooks.xml xalanc.xsl > train.out 2> train.err",
    ),
}


def main():
    here = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(
        description="run packaged SPEC CPU2006 train workloads sequentially"
    )
    parser.add_argument(
        "--mode", choices=("perf", "profile", "window"), required=True,
        help="uninstrumented timing, TLB counters, or exact memory denominator",
    )
    parser.add_argument(
        "--benchmark", action="append", choices=BENCHMARKS,
        help="benchmark to run; repeat the option, or omit it to run all",
    )
    parser.add_argument(
        "--archive", type=Path,
        default=here / "workloads/spec2006-train-x86_64.tar.xz",
    )
    parser.add_argument("--cpu", default="2")
    parser.add_argument("--perf-frequency", type=int, default=99)
    parser.add_argument(
        "--large-page-cache", choices=("off", "on", "probe"), default="off",
    )
    parser.add_argument(
        "--ptw-cache", choices=("off", "on", "probe"), default="off",
    )
    parser.add_argument(
        "--guest-thp", choices=("leave", "always", "madvise", "never"),
        default="leave",
    )
    parser.add_argument(
        "--name-suffix", default="",
        help="suffix for repetitions, for example r01 or baseline",
    )
    parser.add_argument("--snapshot", action=argparse.BooleanOptionalAction,
                        default=True)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    if args.name_suffix and not re.fullmatch(
            r"[A-Za-z0-9][A-Za-z0-9._-]*", args.name_suffix):
        parser.error(
            "--name-suffix must contain only letters, digits, '.', '_' and '-'"
        )

    archive = args.archive.resolve()
    if not archive.is_file():
        parser.error(
            f"archive does not exist: {archive}; run prepare-spec2006.py"
        )
    names = args.benchmark or list(BENCHMARKS)
    qemu = (here.parent / ("build-tlb-base" if args.mode == "perf"
                           else "build-tlb-profile") /
            "qemu-system-x86_64").resolve()
    if not qemu.is_file():
        parser.error(f"QEMU binary does not exist: {qemu}")

    for benchmark in names:
        slug, guest_argv = BENCHMARKS[benchmark]
        suffix = f"-{args.name_suffix}" if args.name_suffix else ""
        command = [
            sys.executable, str(here / "run-cloud-profile.py"),
            "--qemu", str(qemu),
            "--name", f"spec2006-{slug}-train-{args.mode}{suffix}",
            "--cpu", args.cpu,
            "--copy-to-workloads", str(archive),
            "--prepare-command",
            ("cd ~/tlb-workloads && "
             f"tar -xJf {shlex.quote(archive.name)}"),
            "--command",
            (f"cd ~/tlb-workloads/spec2006-train/{benchmark} && "
             f"{guest_argv}"),
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
