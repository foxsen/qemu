#!/usr/bin/env python3
"""Run one initramfs workload with exact QMP-delimited measurements."""

import argparse
import json
import os
from pathlib import Path
import re
import socket
import subprocess
import sys
import time


class QMP:
    def __init__(self, path: Path):
        self.sock = connect_unix(path)
        self.file = self.sock.makefile("rwb", buffering=0)
        self._read_response()
        self.command("qmp_capabilities")

    def _read_response(self):
        while True:
            line = self.file.readline()
            if not line:
                raise RuntimeError("QMP connection closed")
            message = json.loads(line)
            if "return" in message or "error" in message or "QMP" in message:
                return message

    def command(self, execute, arguments=None):
        request = {"execute": execute}
        if arguments:
            request["arguments"] = arguments
        self.file.write(json.dumps(request).encode() + b"\n")
        response = self._read_response()
        if "error" in response:
            raise RuntimeError(f"QMP {execute} failed: {response['error']}")
        return response.get("return")

    def hmp(self, command):
        return self.command("human-monitor-command", {"command-line": command})


def connect_unix(path: Path, timeout=30):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        try:
            sock.connect(str(path))
            return sock
        except (FileNotFoundError, ConnectionRefusedError):
            sock.close()
            time.sleep(0.05)
    raise TimeoutError(f"socket did not appear: {path}")


def prepare_result_dir(path):
    if path.exists() and any(path.iterdir()):
        raise FileExistsError(f"result directory is not empty: {path}")
    path.mkdir(parents=True, exist_ok=True)


def parse_tlb(text):
    result = {}
    match = re.search(
        r"SoftMMU TLB config fixed_entries=(\d+) victim=(on|off) "
        r"current_min=(\d+) current_max=(\d+)", text
    )
    if match:
        result["tlb_config"] = {
            "fixed_entries": int(match.group(1)),
            "victim": match.group(2),
            "current_min": int(match.group(3)),
            "current_max": int(match.group(4)),
        }
    for kind, miss, victim, fill in re.findall(
        r"TLB (load|store|fetch)\s+l1_miss=(\d+) victim_hit=(\d+) "
        r"fill_call=(\d+)", text
    ):
        result[kind] = {
            "l1_miss": int(miss),
            "victim_hit": int(victim),
            "fill_call": int(fill),
        }
    origins = {}
    for origin, kind, miss, victim, fill in re.findall(
        r"TLB origin=(helper|probe|atomic) access=(load|store|fetch) "
        r"l1_miss=(\d+) victim_hit=(\d+) fill_call=(\d+)", text
    ):
        origins.setdefault(origin, {})[kind] = {
            "l1_miss": int(miss),
            "victim_hit": int(victim),
            "fill_call": int(fill),
        }
    if origins:
        result["origins"] = origins
    for origin, kind, hits in re.findall(
        r"TLB large-page origin=(helper|probe|atomic) "
        r"access=(load|store|fetch) hit=(\d+)", text
    ):
        result.setdefault("origins", {}).setdefault(origin, {}).setdefault(
            kind, {"l1_miss": 0, "victim_hit": 0, "fill_call": 0}
        )["large_page_hit"] = int(hits)
    ptw = {}
    for stage, kind, walks, restarts, levels in re.findall(
        r"PTW stage=(primary|nested) access=(load|store|fetch) "
        r"walks=(\d+) full_restarts=(\d+) "
        r"levels=([^\r\n]+)", text
    ):
        ptw.setdefault(stage, {})[kind] = {
            "walks": int(walks),
            "full_restarts": int(restarts),
            "levels": {
                level: int(count)
                for level, count in re.findall(r"(\d+):(\d+)", levels)
            },
        }
    if ptw:
        result["ptw"] = ptw
    for key, pattern in {
        "full_flush": r"TLB full flushes\s+(\d+)",
        "partial_flush": r"TLB partial flushes\s+(\d+)",
        "elided_flush": r"TLB elided flushes\s+(\d+)",
        "fill_installs": r"TLB fill installs\s+(\d+)",
    }.items():
        match = re.search(pattern, text)
        if match:
            result[key] = int(match.group(1))
    match = re.search(r"TLB fill page bits ([^\n]*)", text)
    if match:
        result["page_bits"] = {
            bits: int(count)
            for bits, count in re.findall(r"(\d+):(\d+)", match.group(1))
        }
    match = re.search(
        r"Large-page cache mode=(off|on|probe|adaptive) lookup=(\d+) "
        r"match=(\d+) hit=(\d+) insert=(\d+) eviction=(\d+) "
        r"flush=(\d+)", text
    )
    if match:
        result["large_page_cache"] = {
            "mode": match.group(1),
            "lookup": int(match.group(2)),
            "match": int(match.group(3)),
            "hit": int(match.group(4)),
            "insert": int(match.group(5)),
            "eviction": int(match.group(6)),
            "flush": int(match.group(7)),
        }
        adaptive = re.search(
            r"Large-page adaptive sample_windows=(\d+) "
            r"active_windows=(\d+) bypass_windows=(\d+) "
            r"bypassed_lookups=(\d+) bypass_cpus=(\d+)", text
        )
        if adaptive:
            result["large_page_cache"]["adaptive"] = {
                "sample_windows": int(adaptive.group(1)),
                "active_windows": int(adaptive.group(2)),
                "bypass_windows": int(adaptive.group(3)),
                "bypassed_lookups": int(adaptive.group(4)),
                "bypass_cpus": int(adaptive.group(5)),
            }
    match = re.search(
        r"PTW cache mode=(off|on|probe|adaptive) flush=(\d+)", text
    )
    if match:
        result["ptw_cache"] = {
            "mode": match.group(1),
            "flush": int(match.group(2)),
            "levels": {},
        }
        adaptive = re.search(
            r"PTW adaptive sample_windows=(\d+) active_windows=(\d+) "
            r"bypass_windows=(\d+) bypassed_lookups=(\d+) "
            r"bypass_cpus=(\d+)", text
        )
        if adaptive:
            result["ptw_cache"]["adaptive"] = {
                "sample_windows": int(adaptive.group(1)),
                "active_windows": int(adaptive.group(2)),
                "bypass_windows": int(adaptive.group(3)),
                "bypassed_lookups": int(adaptive.group(4)),
                "bypass_cpus": int(adaptive.group(5)),
            }
        for level, lookup, matched, hit, insert, eviction in re.findall(
            r"PTW cache level=([234]) lookup=(\d+) match=(\d+) "
            r"hit=(\d+) insert=(\d+) eviction=(\d+)", text
        ):
            result["ptw_cache"]["levels"][level] = {
                "lookup": int(lookup),
                "match": int(matched),
                "hit": int(hit),
                "insert": int(insert),
                "eviction": int(eviction),
            }
    return result


def subtract(after, before):
    if isinstance(after, dict):
        before = before if isinstance(before, dict) else {}
        return {key: (value if key in {"tlb_config", "bypass_cpus"} else
                      subtract(value, before.get(key, 0)))
                for key, value in after.items()}
    if isinstance(after, (int, float)):
        return after - before
    return after


def validate_tlb_config(parsed, fixed_entries, victim):
    config = parsed.get("tlb_config")
    if not config:
        raise ValueError("info jit did not report the SoftMMU TLB config")
    if config["fixed_entries"] != fixed_entries:
        raise ValueError(
            "SoftMMU TLB fixed-entry mismatch: "
            f"expected {fixed_entries}, got {config['fixed_entries']}"
        )
    if fixed_entries and (
            config["current_min"] != fixed_entries or
            config["current_max"] != fixed_entries):
        raise ValueError(
            "SoftMMU TLB resized despite fixed configuration: "
            f"min={config['current_min']} max={config['current_max']}"
        )
    if config["victim"] != victim:
        raise ValueError(
            "SoftMMU victim-TLB mismatch: "
            f"expected {victim}, got {config['victim']}"
        )


def process_cpu_seconds(pid):
    ticks = os.sysconf(os.sysconf_names["SC_CLK_TCK"])
    total = 0
    for stat_path in Path(f"/proc/{pid}/task").glob("*/stat"):
        fields = stat_path.read_text().split()
        total += int(fields[13]) + int(fields[14])
    return total / ticks


def set_process_nice(pid, requested):
    if not -20 <= requested <= 19:
        raise ValueError("nice value must be between -20 and 19")
    if requested < 0:
        subprocess.run(
            ["sudo", "-n", "renice", "-n", str(requested), "-p", str(pid)],
            check=True, stdout=subprocess.DEVNULL,
        )
    else:
        os.setpriority(os.PRIO_PROCESS, pid, requested)
    actual = os.getpriority(os.PRIO_PROCESS, pid)
    if actual != requested:
        raise RuntimeError(
            f"QEMU nice mismatch: expected {requested}, got {actual}"
        )
    return actual


def descendants(pid):
    found = []
    pending = [pid]
    while pending:
        parent = pending.pop()
        path = Path(f"/proc/{parent}/task/{parent}/children")
        try:
            children = [int(item) for item in path.read_text().split()]
        except FileNotFoundError:
            continue
        found.extend(children)
        pending.extend(children)
    return found


def stop_perf(perf):
    targets = descendants(perf.pid)
    targets.append(perf.pid)
    subprocess.run(["sudo", "-n", "kill", "-INT", *map(str, targets)],
                   check=False)
    try:
        return perf.wait(timeout=30)
    except subprocess.TimeoutExpired:
        subprocess.run(["sudo", "-n", "kill", "-TERM", *map(str, targets)],
                       check=False)
        return perf.wait(timeout=10)


def main():
    here = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser()
    parser.add_argument("--qemu", type=Path,
                        default=here.parent / "build-tlb-profile" /
                        "qemu-system-x86_64")
    parser.add_argument("--mode", default="seq",
                        choices=["dense", "seq", "random", "conflict",
                                 "mprotect"])
    parser.add_argument("--mib", type=int, default=128)
    parser.add_argument("--passes", type=int, default=128)
    parser.add_argument("--page-mode", default="nohuge",
                        choices=["huge", "nohuge"])
    parser.add_argument("--name")
    parser.add_argument("--cpu", default="2")
    parser.add_argument("--nice", type=int, default=0)
    parser.add_argument("--perf", action="store_true")
    parser.add_argument("--perf-frequency", type=int, default=997)
    parser.add_argument("--perf-event", default="cpu_core/cycles/u")
    parser.add_argument("--perf-callgraph", action="store_true")
    parser.add_argument("--perfmap", action=argparse.BooleanOptionalAction,
                        default=True,
                        help="emit JIT symbols; disable for long workloads")
    parser.add_argument("--perf-report", action="store_true",
                        help="also build the redundant flat perf report")
    parser.add_argument("--plugin-mem", action="store_true")
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
    args = parser.parse_args()

    if args.tlb_entries < 0 or (
            args.tlb_entries and
            args.tlb_entries & (args.tlb_entries - 1)):
        parser.error("--tlb-entries must be zero or a power of two")

    name = args.name or (f"{args.mode}-{args.mib}m-{args.passes}p-"
                         f"{args.page_mode}")
    result_dir = here / "results" / name
    try:
        prepare_result_dir(result_dir)
    except FileExistsError as error:
        parser.error(f"{error}; use a new --name")
    # AF_UNIX paths are limited to 108 bytes on Linux.  Result names are
    # intentionally descriptive, so use short per-process control paths.
    qmp_path = Path("/tmp") / f"qemu-tlb-{os.getpid()}-qmp.sock"
    serial_path = Path("/tmp") / f"qemu-tlb-{os.getpid()}-serial.sock"
    for path in (qmp_path, serial_path):
        path.unlink(missing_ok=True)

    cmd = [
        "taskset", "-c", args.cpu, str(args.qemu),
        "-accel", "tcg,thread=single", "-machine", "pc",
        "-cpu", "max", "-smp", "1", "-m", "512M",
        "-no-user-config", "-nodefaults", "-display", "none",
        "-qmp", f"unix:{qmp_path},server=on,wait=off",
        "-chardev", f"socket,id=serial,path={serial_path},server=on,wait=off",
        "-serial", "chardev:serial", "-no-reboot",
        "-kernel", str(here / "images/debian-bookworm-amd64-linux"),
        "-initrd", str(here / "images/tlb-initramfs.img"),
        "-append", ("console=ttyS0 panic=-1 quiet tlb_handshake=1 "
                    f"tlb_mode={args.mode} tlb_mib={args.mib} "
                    f"tlb_passes={args.passes} "
                    f"tlb_page_mode={args.page_mode}"),
    ]
    if args.perfmap:
        cmd.append("-perfmap")
    if args.plugin_mem:
        plugin = args.qemu.parent / "tests/plugin/libmem.so"
        cmd.extend(["-plugin", f"{plugin},inline=true,track=rw"])

    stderr_path = result_dir / "qemu.stderr"
    qemu_env = os.environ.copy()
    qemu_env["QEMU_SOFTMMU_LP_CACHE"] = args.large_page_cache
    qemu_env["QEMU_X86_PTW_CACHE"] = args.ptw_cache
    qemu_env["QEMU_SOFTMMU_VICTIM_TLB"] = args.victim_tlb
    if args.tlb_entries:
        qemu_env["QEMU_SOFTMMU_TLB_ENTRIES"] = str(args.tlb_entries)
    else:
        qemu_env.pop("QEMU_SOFTMMU_TLB_ENTRIES", None)
    with stderr_path.open("wb") as stderr_file:
        qemu = subprocess.Popen(cmd, stderr=stderr_file, env=qemu_env)
    qemu_nice = set_process_nice(qemu.pid, args.nice)
    qmp = QMP(qmp_path)
    serial = connect_unix(serial_path)
    console = bytearray()
    before_text = after_text = ""
    before_cpu = after_cpu = 0.0
    before_wall = after_wall = 0
    perf = None
    perf_path = result_dir / "perf.data"

    try:
        while qemu.poll() is None:
            data = serial.recv(4096)
            if not data:
                break
            console.extend(data)
            sys.stdout.buffer.write(data)
            sys.stdout.buffer.flush()
            decoded = console.decode(errors="replace")
            if not before_text and "TLB-BENCH-READY" in decoded:
                qmp.command("stop")
                before_text = qmp.hmp("info jit")
                validate_tlb_config(parse_tlb(before_text),
                                    args.tlb_entries, args.victim_tlb)
                before_cpu = process_cpu_seconds(qemu.pid)
                before_wall = time.monotonic_ns()
                if args.perf:
                    perf_cmd = [
                        "sudo", "-n", "perf", "record", "-e",
                        args.perf_event, "-F", str(args.perf_frequency),
                        "-p", str(qemu.pid),
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
                serial.sendall(b"\n")
            elif before_text and not after_text and "TLB-BENCH-DONE" in decoded:
                qmp.command("stop")
                after_wall = time.monotonic_ns()
                after_cpu = process_cpu_seconds(qemu.pid)
                after_text = qmp.hmp("info jit")
                if perf:
                    stop_perf(perf)
                qmp.command("quit")
                break
        qemu.wait(timeout=30)
    finally:
        if qemu.poll() is None:
            qemu.terminate()
            qemu.wait(timeout=10)
        serial.close()
        for path in (qmp_path, serial_path):
            path.unlink(missing_ok=True)

    (result_dir / "console.log").write_bytes(console)
    before = parse_tlb(before_text)
    after = parse_tlb(after_text)
    if after:
        validate_tlb_config(after, args.tlb_entries, args.victim_tlb)
    report = {
        "name": name,
        "command": cmd,
        "qemu_version": subprocess.check_output(
            [str(args.qemu), "--version"], text=True).splitlines()[0],
        "host_uname": subprocess.check_output(["uname", "-a"], text=True).strip(),
        "workload": vars(args) | {"qemu": str(args.qemu)},
        "wall_seconds": (after_wall - before_wall) / 1e9,
        "qemu_cpu_seconds": after_cpu - before_cpu,
        "qemu_nice": qemu_nice,
        "tlb_before": before,
        "tlb_after": after,
        "tlb_delta": subtract(after, before) if after else {},
        "info_jit_before": before_text,
        "info_jit_after": after_text,
        "qemu_exit_status": qemu.returncode,
    }
    (result_dir / "result.json").write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n")

    if perf_path.exists() and args.perf_report:
        perf_report = subprocess.run(
            ["sudo", "-n", "perf", "report", "--stdio", "--no-children",
             "--call-graph", "none", "--sort", "symbol,dso",
             "--field-separator", ";", "--fields", "overhead,symbol,dso",
             "-i", str(perf_path)],
            text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            check=False,
        )
        (result_dir / "perf-report.txt").write_text(perf_report.stdout)
    if perf_path.exists():
        perf_script = subprocess.run(
            ["sudo", "-n", "perf", "script", "-G", "-i", str(perf_path),
             "-F", "period,ip,sym,dso"],
            text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            check=False,
        )
        (result_dir / "perf-script.txt").write_text(perf_script.stdout)
    print(json.dumps(report["tlb_delta"], indent=2, sort_keys=True))
    print(f"result: {result_dir / 'result.json'}")


if __name__ == "__main__":
    main()
