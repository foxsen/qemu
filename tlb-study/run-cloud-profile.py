#!/usr/bin/env python3
"""Run a command in the persistent Debian guest with QMP boundaries."""

import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import platform
import re
import shlex
import subprocess
import sys
import time

from importlib.machinery import SourceFileLoader


def load_helpers(here):
    module = SourceFileLoader("run_profile", str(here / "run-profile.py"))
    return module.load_module()


def utc_now():
    return datetime.now(timezone.utc).isoformat()


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def file_record(path, with_hash=True):
    path = path.resolve()
    stat = path.stat()
    record = {
        "path": str(path),
        "size_bytes": stat.st_size,
        "mtime_ns": stat.st_mtime_ns,
    }
    if with_hash:
        record["sha256"] = sha256(path)
    return record


def command_text(command):
    result = subprocess.run(
        command, text=True, stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT, check=False,
    )
    return {"returncode": result.returncode, "output": result.stdout.strip()}


def git_record(root):
    head = command_text(["git", "-C", str(root), "rev-parse", "HEAD"])
    status = command_text([
        "git", "-C", str(root), "status", "--short",
        "--untracked-files=no",
    ])
    diff = subprocess.run(
        ["git", "-C", str(root), "diff", "--binary", "--submodule=short"],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, check=False,
    )
    return {
        "head": head["output"] if head["returncode"] == 0 else None,
        "tracked_status": status["output"],
        "status_returncode": status["returncode"],
        "tracked_diff_returncode": diff.returncode,
        "tracked_diff_size_bytes": len(diff.stdout),
        "tracked_diff_sha256": hashlib.sha256(diff.stdout).hexdigest(),
    }


def host_record(cpu):
    cpu_match = re.search(r"\d+", str(cpu))
    first_cpu = int(cpu_match.group()) if cpu_match else 0
    cpu_root = Path("/sys/devices/system/cpu") / f"cpu{first_cpu}"
    fields = {}
    for name, relative in {
            "governor": "cpufreq/scaling_governor",
            "scaling_driver": "cpufreq/scaling_driver",
            "scaling_min_khz": "cpufreq/scaling_min_freq",
            "scaling_max_khz": "cpufreq/scaling_max_freq",
            "base_frequency_khz": "cpufreq/base_frequency",
            "energy_performance_preference":
                "cpufreq/energy_performance_preference",
            "thread_siblings": "topology/thread_siblings_list",
            "core_id": "topology/core_id",
            "package_id": "topology/physical_package_id",
    }.items():
        path = cpu_root / relative
        fields[name] = path.read_text().strip() if path.exists() else None
    model = None
    for line in Path("/proc/cpuinfo").read_text().splitlines():
        if line.startswith("model name"):
            model = line.split(":", 1)[1].strip()
            break
    no_turbo = Path("/sys/devices/system/cpu/intel_pstate/no_turbo")
    return {
        "uname": platform.uname()._asdict(),
        "cpu_model": model,
        "taskset_cpu_list": str(cpu),
        "first_logical_cpu": first_cpu,
        "process_affinity": sorted(os.sched_getaffinity(0)),
        "perf_version": command_text(["perf", "--version"]),
        "intel_pstate_no_turbo": (
            no_turbo.read_text().strip() if no_turbo.exists() else None
        ),
        **fields,
    }


def disk_record(path, exact_hash):
    """Record the launch image and immutable qcow2 backing files."""
    path = path.resolve()
    info = command_text([
        "qemu-img", "info", "--output=json", "--backing-chain", str(path),
    ])
    chain = []
    if info["returncode"] == 0:
        for index, entry in enumerate(json.loads(info["output"])):
            filename = Path(entry["filename"])
            if not filename.is_absolute():
                filename = path.parent / filename
            chain.append(file_record(
                filename, with_hash=(exact_hash or index > 0)
            ))
    return {
        "launch_image": file_record(path, with_hash=exact_hash),
        "launch_image_hash_is_identity": exact_hash,
        "qemu_img_info": info,
        "backing_chain": chain,
    }


def run_options_record(args):
    return {
        "cpu": args.cpu,
        "nice": args.nice,
        "memory": args.memory,
        "smp": args.smp,
        "ssh_port": args.ssh_port,
        "perf": args.perf,
        "perf_frequency": args.perf_frequency if args.perf else None,
        "perf_event": args.perf_event if args.perf else None,
        "perf_callgraph": args.perf_callgraph,
        "perfmap": args.perfmap,
        "plugin_window": args.plugin_window,
        "snapshot": args.snapshot,
        "large_page_cache": args.large_page_cache,
        "ptw_cache": args.ptw_cache,
        "tlb_entries": args.tlb_entries,
        "victim_tlb": args.victim_tlb,
        "guest_thp": args.guest_thp,
    }


def combined_prepare_command(command, guest_thp):
    if guest_thp == "leave":
        return command
    path = "/sys/kernel/mm/transparent_hugepage/enabled"
    thp = (
        f"printf '%s\\n' {shlex.quote(guest_thp)} | "
        f"sudo tee {path} >/dev/null && "
        f"printf 'transparent_hugepage=' && cat {path}"
    )
    return f"{thp} && {command}"


def ssh_base(key, port):
    return [
        "ssh", "-i", str(key), "-p", str(port),
        "-o", "StrictHostKeyChecking=no",
        "-o", "UserKnownHostsFile=/dev/null",
        "-o", "LogLevel=ERROR",
        "-o", "ConnectTimeout=5", "debian@127.0.0.1",
    ]


def wait_ssh(base, qemu, timeout=300):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline and qemu.poll() is None:
        check = subprocess.run(base + ["test -f /var/lib/tlb-study-cloud-ready"],
                               stdout=subprocess.DEVNULL,
                               stderr=subprocess.DEVNULL)
        if check.returncode == 0:
            return
        time.sleep(2)
    raise RuntimeError("Debian guest did not become SSH-ready")


def guest_record(ssh):
    packages = command_text(ssh + [
        "dpkg-query -W -f='${Package}=${Version}\\n' "
        "qemu-system-x86 stress-ng sysbench openjdk-17-jre-headless "
        "2>/dev/null || true"
    ])
    return {
        "uname": command_text(ssh + ["uname -a"]),
        "os_release": command_text(ssh + ["cat /etc/os-release"]),
        "packages": packages,
    }


def marker_pcs(marker):
    symbols = {}
    output = subprocess.check_output(["nm", "-n", str(marker)], text=True)
    for line in output.splitlines():
        fields = line.split()
        if len(fields) == 3 and fields[2] in {
                "tlb_window_start", "tlb_window_stop"}:
            symbols[fields[2]] = "0x" + fields[0]
    if len(symbols) != 2:
        raise RuntimeError(f"marker symbols missing from {marker}")
    return symbols["tlb_window_start"], symbols["tlb_window_stop"]


def main():
    here = Path(__file__).resolve().parent
    helpers = load_helpers(here)
    parser = argparse.ArgumentParser()
    parser.add_argument("--qemu", type=Path,
                        default=here.parent / "build-tlb-profile" /
                        "qemu-system-x86_64")
    parser.add_argument("--command", required=True)
    parser.add_argument("--prepare-command", default="true",
                        help="guest setup performed before the READY barrier")
    parser.add_argument("--name", required=True)
    parser.add_argument("--cpu", default="2")
    parser.add_argument("--nice", type=int, default=0)
    parser.add_argument("--disk", type=Path,
                        default=here / "images/debian-12-tlb-overlay.qcow2")
    parser.add_argument("--seed", type=Path,
                        default=here / "images/tlb-seed.iso")
    parser.add_argument("--memory", default="1G")
    parser.add_argument("--smp", type=int, default=1)
    parser.add_argument(
        "--snapshot", action="store_true",
        help="discard guest disk writes after the run",
    )
    parser.add_argument("--ssh-port", type=int, default=2222)
    parser.add_argument("--perf", action="store_true")
    parser.add_argument("--perf-frequency", type=int, default=997)
    parser.add_argument("--perf-event", default="cpu_core/cycles/u")
    parser.add_argument("--perf-callgraph", action="store_true")
    parser.add_argument("--perfmap", action=argparse.BooleanOptionalAction,
                        default=True,
                        help="emit JIT symbols; disable for long workloads")
    parser.add_argument("--perf-report", action="store_true",
                        help="also build the redundant flat perf report")
    parser.add_argument("--plugin-window", action="store_true",
                        help="count guest memory operations inside the barrier")
    parser.add_argument(
        "--large-page-cache",
        choices=("off", "on", "probe", "adaptive"), default="off",
        help="experimental victim-miss large-page cache mode",
    )
    parser.add_argument(
        "--ptw-cache", choices=("off", "on", "probe", "adaptive"),
        default="off",
        help="experimental x86 L2--L4 non-leaf page-table cache mode",
    )
    parser.add_argument(
        "--tlb-entries", type=int, default=0,
        help="fix each SoftMMU TLB to this power-of-two entry count",
    )
    parser.add_argument(
        "--victim-tlb", choices=("on", "off"), default="on",
        help="enable or disable the eight-entry victim TLB",
    )
    parser.add_argument(
        "--guest-thp", choices=("leave", "always", "madvise", "never"),
        default="leave",
        help="set and record the guest transparent-huge-page policy",
    )
    parser.add_argument("--copy-to-workloads", action="append", type=Path,
                        default=[], metavar="FILE",
                        help="copy a host file into ~/tlb-workloads before prepare")
    args = parser.parse_args()

    args.qemu = args.qemu.resolve()
    args.disk = args.disk.resolve()
    args.seed = args.seed.resolve()
    for label, path in {
            "QEMU binary": args.qemu,
            "guest disk": args.disk,
            "NoCloud seed": args.seed,
    }.items():
        if not path.is_file():
            parser.error(f"{label} does not exist: {path}")
    copy_sources = []
    for source in args.copy_to_workloads:
        source = source.resolve()
        if not source.is_file():
            parser.error(f"copy source does not exist: {source}")
        copy_sources.append(source)
    args.copy_to_workloads = copy_sources
    guest_prepare_command = combined_prepare_command(
        args.prepare_command, args.guest_thp
    )
    if args.smp < 1:
        parser.error("--smp must be positive")
    if args.tlb_entries < 0 or (
            args.tlb_entries and
            args.tlb_entries & (args.tlb_entries - 1)):
        parser.error("--tlb-entries must be zero or a power of two")
    if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9._-]*", args.name):
        parser.error(
            "--name must contain only letters, digits, '.', '_' and '-'"
        )

    result_dir = here / "results" / args.name
    if result_dir.exists() and any(result_dir.iterdir()):
        parser.error(
            f"result directory is not empty: {result_dir}; use a new --name"
        )
    result_dir.mkdir(parents=True, exist_ok=True)

    provenance = {
        "runner_started_utc": utc_now(),
        "qemu_binary": file_record(args.qemu),
        "harness": {
            "runner": file_record(Path(__file__)),
            "qmp_helpers": file_record(here / "run-profile.py"),
        },
        "guest_disk": disk_record(args.disk, exact_hash=args.snapshot),
        "seed": file_record(args.seed),
        "copied_inputs": [file_record(path)
                          for path in args.copy_to_workloads],
        "git": git_record(here.parent),
        "host": host_record(args.cpu),
        "run_options": run_options_record(args),
    }

    # AF_UNIX paths are limited to 108 bytes on Linux.  Keep control sockets
    # outside potentially long, descriptive result-directory names.
    qmp_path = Path("/tmp") / f"qemu-tlb-cloud-{os.getpid()}.sock"
    qmp_path.unlink(missing_ok=True)
    cmd = [
        "taskset", "-c", args.cpu, str(args.qemu),
        "-accel", "tcg,thread=single", "-machine", "pc", "-cpu", "max",
        "-smp", str(args.smp), "-m", args.memory,
        "-no-user-config", "-display", "none",
        "-serial", f"file:{result_dir / 'console.log'}",
        "-qmp", f"unix:{qmp_path},server=on,wait=off",
        "-drive", f"if=virtio,format=qcow2,file={args.disk}",
        "-drive", ("if=virtio,format=raw,readonly=on,file="
                   f"{args.seed}"),
        "-netdev", ("user,id=net0,hostfwd=tcp:127.0.0.1:"
                    f"{args.ssh_port}-:22"),
        "-device", "virtio-net-pci,netdev=net0",
    ]
    if args.snapshot:
        cmd.append("-snapshot")
    if args.perfmap:
        cmd.append("-perfmap")
    marker = here / "build/tlb-marker"
    if args.plugin_window:
        plugin = here / "build/mem-window.so"
        if not marker.exists() or not plugin.exists():
            raise RuntimeError("run tlb-study/build-mem-window.sh first")
        provenance["window_marker"] = file_record(marker)
        provenance["window_plugin"] = file_record(plugin)
        start_pc, stop_pc = marker_pcs(marker)
        cmd.extend(["-d", "plugin", "-D", str(result_dir / "plugin.log"),
                    "-plugin",
                    f"{plugin},start={start_pc},stop={stop_pc}"])
    qemu_env = os.environ.copy()
    qemu_env["QEMU_SOFTMMU_LP_CACHE"] = args.large_page_cache
    qemu_env["QEMU_X86_PTW_CACHE"] = args.ptw_cache
    qemu_env["QEMU_SOFTMMU_VICTIM_TLB"] = args.victim_tlb
    if args.tlb_entries:
        qemu_env["QEMU_SOFTMMU_TLB_ENTRIES"] = str(args.tlb_entries)
    else:
        qemu_env.pop("QEMU_SOFTMMU_TLB_ENTRIES", None)
    with ((result_dir / "qemu.stdout").open("wb") as stdout,
          (result_dir / "qemu.stderr").open("wb") as stderr):
        qemu = subprocess.Popen(cmd, stdout=stdout, stderr=stderr,
                                env=qemu_env)
    qemu_nice = helpers.set_process_nice(qemu.pid, args.nice)
    key = here / "images/tlb-study-key"
    ssh = ssh_base(key, args.ssh_port)
    perf = None
    marker_start = "/tmp/tlb-marker start; " if args.plugin_window else ""
    marker_stop = "/tmp/tlb-marker stop; " if args.plugin_window else ""
    remote = (
        f"{guest_prepare_command}; prep_rc=$?; "
        "if test \"$prep_rc\" -ne 0; then "
        "printf 'TLB-CLOUD-PREP-FAILED rc=%s\\n' \"$prep_rc\"; "
        "exit \"$prep_rc\"; fi; "
        "printf 'TLB-CLOUD-READY\\n'; IFS= read -r _; "
        f"{marker_start}{args.command}; rc=$?; {marker_stop}"
        "printf 'TLB-CLOUD-DONE rc=%s\\n' \"$rc\"; "
        "IFS= read -r _; exit \"$rc\""
    )
    try:
        qmp = helpers.QMP(qmp_path)
        wait_ssh(ssh, qemu)
        provenance["guest"] = guest_record(ssh)
        for source in args.copy_to_workloads:
            subprocess.run([
                "scp", "-i", str(key), "-P", str(args.ssh_port),
                "-o", "StrictHostKeyChecking=no",
                "-o", "UserKnownHostsFile=/dev/null",
                "-o", "LogLevel=ERROR", str(source),
                "debian@127.0.0.1:/home/debian/tlb-workloads/",
            ], check=True)
        if args.plugin_window:
            subprocess.run([
                "scp", "-i", str(key), "-P", str(args.ssh_port),
                "-o", "StrictHostKeyChecking=no",
                "-o", "UserKnownHostsFile=/dev/null",
                "-o", "LogLevel=ERROR", str(marker),
                "debian@127.0.0.1:/tmp/tlb-marker",
            ], check=True)
        workload = subprocess.Popen(
            ssh + ["bash", "-lc", shlex.quote(remote)],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT, text=True, bufsize=1,
        )
        prepare_output = []
        for line in workload.stdout:
            if line.strip() == "TLB-CLOUD-READY":
                break
            prepare_output.append(line)
            if line.startswith("TLB-CLOUD-PREP-FAILED"):
                (result_dir / "prepare.log").write_text(
                    "".join(prepare_output))
                raise RuntimeError(
                    f"{line.strip()}\n" + "".join(prepare_output[:-1]))
        else:
            raise RuntimeError("remote workload did not reach ready barrier")
        qmp.command("stop")
        before_text = qmp.hmp("info jit")
        helpers.validate_tlb_config(helpers.parse_tlb(before_text),
                                    args.tlb_entries, args.victim_tlb)
        before_cpu = helpers.process_cpu_seconds(qemu.pid)
        measurement_started_utc = utc_now()
        before_wall = time.monotonic_ns()
        perf_path = result_dir / "perf.data"
        if args.perf:
            perf_cmd = [
                "sudo", "-n", "perf", "record", "-e", args.perf_event,
                "-F", str(args.perf_frequency), "-p", str(qemu.pid),
                "-o", str(perf_path),
            ]
            if args.perf_callgraph:
                perf_cmd[6:6] = ["-g", "--call-graph", "dwarf,8192"]
            perf = subprocess.Popen(
                perf_cmd,
                stdout=(result_dir / "perf.stdout").open("wb"),
                stderr=(result_dir / "perf.stderr").open("wb"),
            )
            time.sleep(0.5)
        qmp.command("cont")
        workload.stdin.write("\n")
        workload.stdin.flush()
        output = []
        done = None
        for line in workload.stdout:
            if line.startswith("TLB-CLOUD-DONE"):
                done = line.strip()
                break
            output.append(line)
            sys.stdout.write(line)
            sys.stdout.flush()
        remote_rc = None
        if done is not None:
            remote_rc = int(done.rsplit("=", 1)[1])
        qmp.command("stop")
        after_wall = time.monotonic_ns()
        measurement_ended_utc = utc_now()
        after_cpu = helpers.process_cpu_seconds(qemu.pid)
        after_text = qmp.hmp("info jit")
        if perf:
            helpers.stop_perf(perf)
        before = helpers.parse_tlb(before_text)
        after = helpers.parse_tlb(after_text)
        helpers.validate_tlb_config(after, args.tlb_entries,
                                    args.victim_tlb)
        report = {
            "name": args.name, "command": cmd,
            "guest_command": args.command,
            "guest_prepare_command": guest_prepare_command,
            "remote_done": done,
            "remote_returncode": remote_rc,
            "qemu_version": subprocess.check_output(
                [str(args.qemu), "--version"], text=True).splitlines()[0],
            "provenance": provenance,
            "measurement_started_utc": measurement_started_utc,
            "measurement_ended_utc": measurement_ended_utc,
            "wall_seconds": (after_wall - before_wall) / 1e9,
            "qemu_cpu_seconds": after_cpu - before_cpu,
            "qemu_nice": qemu_nice,
            "tlb_before": before, "tlb_after": after,
            "tlb_delta": helpers.subtract(after, before) if after else {},
            "info_jit_before": before_text, "info_jit_after": after_text,
        }
        (result_dir / "workload.log").write_text("".join(output))
        (result_dir / "prepare.log").write_text("".join(prepare_output))
        (result_dir / "result.json").write_text(
            json.dumps(report, indent=2, sort_keys=True) + "\n")
        if perf_path.exists() and args.perf_report:
            flat = subprocess.run(
                ["sudo", "-n", "perf", "report", "--stdio",
                 "--no-children", "--call-graph", "none", "--sort",
                 "symbol,dso", "--field-separator", ";", "--fields",
                 "overhead,symbol,dso", "-i", str(perf_path)],
                text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
            (result_dir / "perf-report.txt").write_text(flat.stdout)
        if perf_path.exists():
            raw = subprocess.run(
                ["sudo", "-n", "perf", "script", "-G", "-i",
                 str(perf_path), "-F", "period,ip,sym,dso"],
                text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
            (result_dir / "perf-script.txt").write_text(raw.stdout)
        print(json.dumps(report["tlb_delta"], indent=2, sort_keys=True))
        qmp.command("cont")
        if workload.poll() is None:
            workload.stdin.write("\n")
            workload.stdin.flush()
            workload.wait(timeout=30)
        if args.snapshot:
            qmp.command("quit")
            qemu.wait(timeout=10)
        else:
            subprocess.run(ssh + ["sudo", "poweroff"], timeout=20,
                           stdout=subprocess.DEVNULL,
                           stderr=subprocess.DEVNULL)
            try:
                qemu.wait(timeout=60)
            except subprocess.TimeoutExpired:
                qmp.command("quit")
                qemu.wait(timeout=10)
        if done is None:
            raise RuntimeError("remote workload exited before DONE marker")
        if remote_rc != 0:
            raise RuntimeError(f"remote workload failed with rc={remote_rc}")
    except Exception as error:
        failure = {
            "name": args.name,
            "error_type": type(error).__name__,
            "error": str(error),
            "command": cmd,
            "guest_command": args.command,
            "guest_prepare_command": guest_prepare_command,
            "qemu_returncode": qemu.poll(),
            "provenance": provenance,
        }
        (result_dir / "failure.json").write_text(
            json.dumps(failure, indent=2, sort_keys=True) + "\n"
        )
        raise
    finally:
        if perf and perf.poll() is None:
            helpers.stop_perf(perf)
        if qemu.poll() is None:
            qemu.terminate()
            try:
                qemu.wait(timeout=10)
            except subprocess.TimeoutExpired:
                qemu.kill()
                qemu.wait(timeout=10)
        qmp_path.unlink(missing_ok=True)


if __name__ == "__main__":
    main()
