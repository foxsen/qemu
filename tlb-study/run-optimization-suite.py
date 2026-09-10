#!/usr/bin/env python3
"""Run repeatable non-kernel workloads for LP/PTW-cache comparisons."""

import argparse
from pathlib import Path
import re
import shlex
import subprocess
import sys


VARIANTS = {
    "base": ("off", "off"),
    "lp": ("on", "off"),
    "ptw": ("off", "on"),
    "both": ("on", "on"),
}


def workload_args(here, name):
    workloads = here / "workloads"
    images = here / "images"
    if name == "sysbench":
        return {
            "command": ("sysbench memory --threads=1 --memory-block-size=4K "
                        "--memory-total-size=1G --memory-access-mode=rnd run"),
        }
    if name == "stress-tlb":
        return {
            "command": "stress-ng --tlb-shootdown 1 --timeout 10s --metrics-brief",
        }
    if name == "dacapo":
        jar = workloads / "dacapo-9.12-bach.jar"
        return {
            "copy": [jar],
            "command": ("java -Xmx512m -XX:+UseTransparentHugePages "
                        "-jar ~/tlb-workloads/dacapo-9.12-bach.jar "
                        "avrora -n 1"),
        }
    if name == "mcf":
        archive = workloads / "spec2006-train-x86_64.tar.xz"
        return {
            "copy": [archive],
            "prepare": ("cd ~/tlb-workloads && "
                        "tar -xJf spec2006-train-x86_64.tar.xz"),
            "command": ("cd ~/tlb-workloads/spec2006-train/429.mcf && "
                        "./mcf inp.in > inp.out 2> inp.err"),
        }
    if name == "gapbs":
        archive = workloads / "gapbs-v1.5-b5e3e19c.tar.gz"
        prepare = (
            "cd ~/tlb-workloads && rm -rf gapbs-run && mkdir gapbs-run && "
            "tar -xzf gapbs-v1.5-b5e3e19c.tar.gz -C gapbs-run "
            "--strip-components=1 && make -C gapbs-run SERIAL=1 -j1 "
            "converter pr && cd gapbs-run && ./converter -g 20 -m -b kron20.sg"
        )
        return {
            "copy": [archive], "prepare": prepare, "memory": "4G",
            "command": ("cd ~/tlb-workloads/gapbs-run && "
                        "./pr -f kron20.sg -n 1 -i 20 -v > pr.out 2> pr.err && "
                        "grep -q 'Verification: *PASS' pr.out"),
        }
    if name == "nested":
        kernel = images / "debian-bookworm-amd64-linux"
        initrd = images / "tlb-initramfs.img"
        return {
            "copy": [kernel, initrd],
            "command": (
                "timeout 240 sudo qemu-system-x86_64 -accel kvm -cpu host "
                "-machine pc -smp 1 -m 256M -mem-prealloc -no-user-config "
                "-nodefaults -display none -serial stdio -no-reboot "
                "-kernel ~/tlb-workloads/debian-bookworm-amd64-linux "
                "-initrd ~/tlb-workloads/tlb-initramfs.img "
                "-append 'console=ttyS0 panic=-1 quiet tlb_mode=random "
                "tlb_mib=64 tlb_passes=32 tlb_page_mode=huge'"
            ),
        }
    raise ValueError(name)


def main():
    here = Path(__file__).resolve().parent
    choices = ("sysbench", "stress-tlb", "dacapo", "mcf", "gapbs", "nested")
    parser = argparse.ArgumentParser()
    parser.add_argument("--workload", action="append", choices=choices)
    parser.add_argument("--variant", action="append", choices=VARIANTS)
    parser.add_argument("--repetitions", type=int, default=3)
    parser.add_argument("--cpu", default="2")
    parser.add_argument("--nice", type=int, default=0)
    parser.add_argument("--name-tag", default="")
    parser.add_argument("--tlb-entries", type=int, default=0)
    parser.add_argument("--victim-tlb", choices=("on", "off"), default="on")
    parser.add_argument("--perf", action="store_true")
    parser.add_argument("--perf-frequency", type=int, default=997)
    parser.add_argument("--perf-event", default="cpu_core/cycles/u")
    parser.add_argument("--perfmap", action=argparse.BooleanOptionalAction,
                        default=True)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--resume", action="store_true",
                        help="skip result directories that already contain result.json")
    args = parser.parse_args()
    if args.repetitions < 1:
        parser.error("--repetitions must be positive")
    if args.tlb_entries < 0 or (
            args.tlb_entries and
            args.tlb_entries & (args.tlb_entries - 1)):
        parser.error("--tlb-entries must be zero or a power of two")
    if args.name_tag and not re.fullmatch(
            r"[A-Za-z0-9][A-Za-z0-9._-]*", args.name_tag):
        parser.error("--name-tag contains unsupported characters")

    qemu = (here.parent / "build-tlb-base/qemu-system-x86_64").resolve()
    if not qemu.is_file():
        parser.error(f"QEMU binary does not exist: {qemu}")
    requested_workloads = args.workload or list(choices)
    variants = args.variant or list(VARIANTS)
    for rep in range(1, args.repetitions + 1):
        for workload in requested_workloads:
            config = workload_args(here, workload)
            for variant in variants:
                lp_mode, ptw_mode = VARIANTS[variant]
                tag = f"-{args.name_tag}" if args.name_tag else ""
                name = f"opt-{workload}{tag}-{variant}-r{rep:02d}"
                result = here / "results" / name / "result.json"
                if args.resume and result.is_file():
                    print(f"skip {name}", flush=True)
                    continue
                command = [
                    sys.executable, str(here / "run-cloud-profile.py"),
                    "--qemu", str(qemu), "--name", name,
                    "--cpu", args.cpu, "--nice", str(args.nice),
                    "--snapshot", "--guest-thp", "always",
                    "--large-page-cache", lp_mode, "--ptw-cache", ptw_mode,
                    "--tlb-entries", str(args.tlb_entries),
                    "--victim-tlb", args.victim_tlb,
                    "--memory", config.get("memory", "1G"),
                    "--prepare-command", config.get("prepare", "true"),
                    "--command", config["command"],
                ]
                if args.perf:
                    command.extend([
                        "--perf", "--perf-frequency",
                        str(args.perf_frequency), "--perf-event",
                        args.perf_event,
                    ])
                    if not args.perfmap:
                        command.append("--no-perfmap")
                for path in config.get("copy", []):
                    if not path.is_file():
                        parser.error(f"workload input does not exist: {path}")
                    command.extend(["--copy-to-workloads", str(path.resolve())])
                print(shlex.join(command), flush=True)
                if not args.dry_run:
                    subprocess.run(command, check=True)


if __name__ == "__main__":
    main()
