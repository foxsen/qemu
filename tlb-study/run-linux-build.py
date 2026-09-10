#!/usr/bin/env python3
"""Run the paper-style Linux 3.12.9 single-thread build reproducibly."""

import argparse
from pathlib import Path
import re
import shlex
import subprocess
import sys


HOSTCFLAGS = (
    "-Wall -Wmissing-prototypes -Wstrict-prototypes -O2 "
    "-fomit-frame-pointer -fcommon"
)
KCFLAGS = (
    "-fcommon -fno-pie -fno-stack-protector "
    "-Wno-attributes -Wno-pointer-sign -Wno-address"
)


def main():
    here = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(
        description="run a clean Linux 3.12.9 make -j1 under QEMU 8.2"
    )
    parser.add_argument(
        "--mode", choices=("verify", "perf", "profile", "window"),
        required=True,
    )
    parser.add_argument(
        "--archive", type=Path,
        default=here / "workloads/linux-3.12.9.tar.xz",
    )
    parser.add_argument(
        "--patch", type=Path,
        default=here / "workloads/linux-3.12.9-gcc12-module-table.patch",
    )
    parser.add_argument("--cpu", default="2")
    parser.add_argument("--perf-frequency", type=int, default=99)
    parser.add_argument("--name-suffix", default="")
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
    patch = args.patch.resolve()
    for label, path in (("kernel archive", archive),
                        ("compatibility patch", patch)):
        if not path.is_file():
            parser.error(f"{label} does not exist: {path}")

    qemu_build = "build-tlb-profile" if args.mode in {
        "profile", "window"
    } else "build-tlb-base"
    qemu = (here.parent / qemu_build / "qemu-system-x86_64").resolve()
    if not qemu.is_file():
        parser.error(f"QEMU binary does not exist: {qemu}")

    suffix = f"-{args.name_suffix}" if args.name_suffix else ""
    prepare = (
        "cd ~/tlb-workloads && rm -rf linux-3.12.9-run && "
        "mkdir linux-3.12.9-run && "
        f"tar -xJf {shlex.quote(archive.name)} -C linux-3.12.9-run "
        "--strip-components=1 && cd linux-3.12.9-run && "
        "cp include/linux/compiler-gcc4.h include/linux/compiler-gcc12.h && "
        f"patch -p1 < ../{shlex.quote(patch.name)} && "
        f"make HOSTCFLAGS={shlex.quote(HOSTCFLAGS)} defconfig"
    )
    guest_command = (
        "cd ~/tlb-workloads/linux-3.12.9-run && "
        f"make HOSTCFLAGS={shlex.quote(HOSTCFLAGS)} "
        f"KCFLAGS={shlex.quote(KCFLAGS)} -j1 vmlinux bzImage && "
        "test -s vmlinux && test -s arch/x86/boot/bzImage && "
        "sha256sum vmlinux arch/x86/boot/bzImage && "
        "printf 'TLB-LINUX-BUILD-PASS\\n'"
    )
    command = [
        sys.executable, str(here / "run-cloud-profile.py"),
        "--qemu", str(qemu),
        "--name", f"linux-3.12.9-clean-{args.mode}{suffix}",
        "--cpu", args.cpu,
        "--copy-to-workloads", str(archive),
        "--copy-to-workloads", str(patch),
        "--prepare-command", prepare,
        "--command", guest_command,
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
