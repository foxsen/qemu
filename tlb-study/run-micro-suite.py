#!/usr/bin/env python3
"""Run repeatable minimal-guest timing, counter, or perf suites."""

import argparse
from pathlib import Path
import subprocess


MATRIX = {
    "dense-nohuge": ("dense", 128, 128, "nohuge"),
    "dense-huge": ("dense", 128, 128, "huge"),
    "random-nohuge": ("random", 128, 128, "nohuge"),
    "random-huge": ("random", 128, 128, "huge"),
    "conflict-nohuge": ("conflict", 128, 128, "nohuge"),
    "conflict-huge": ("conflict", 128, 128, "huge"),
    "mprotect-nohuge": ("mprotect", 128, 32, "nohuge"),
}


def main():
    here = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser()
    parser.add_argument("--phase", choices=["timing", "counters", "perf"],
                        required=True)
    parser.add_argument("--repetitions", type=int, default=5)
    parser.add_argument("--workloads", default=",".join(MATRIX))
    parser.add_argument(
        "--large-page-cache",
        choices=("off", "on", "probe", "adaptive"), default="off",
    )
    parser.add_argument(
        "--ptw-cache", choices=("off", "on", "probe", "adaptive"),
        default="off",
    )
    parser.add_argument("--tlb-entries", type=int, default=0)
    parser.add_argument("--victim-tlb", choices=("on", "off"), default="on")
    parser.add_argument("--name-tag", default="")
    parser.add_argument("--cpu", default="2")
    parser.add_argument("--nice", type=int, default=0)
    parser.add_argument("--perf-event", default="cpu_core/cycles/u")
    args = parser.parse_args()
    chosen = args.workloads.split(",")
    unknown = sorted(set(chosen) - set(MATRIX))
    if unknown:
        parser.error(f"unknown workloads: {', '.join(unknown)}")
    qemu = (here.parent / ("build-tlb-profile" if args.phase == "counters"
                           else "build-tlb-base") / "qemu-system-x86_64")
    tag = f"-{args.name_tag}" if args.name_tag else ""
    for repetition in range(1, args.repetitions + 1):
        for label in chosen:
            mode, mib, passes, page_mode = MATRIX[label]
            cmd = [
                str(here / "run-profile.py"), "--qemu", str(qemu),
                "--mode", mode, "--mib", str(mib), "--passes", str(passes),
                "--page-mode", page_mode, "--name",
                f"suite-{args.phase}-{label}{tag}-r{repetition:02d}",
                "--large-page-cache", args.large_page_cache,
                "--ptw-cache", args.ptw_cache,
                "--tlb-entries", str(args.tlb_entries),
                "--victim-tlb", args.victim_tlb,
                "--cpu", args.cpu, "--nice", str(args.nice),
                "--perf-event", args.perf_event,
            ]
            if args.phase == "perf":
                cmd.append("--perf")
            subprocess.run(cmd, check=True)


if __name__ == "__main__":
    main()
