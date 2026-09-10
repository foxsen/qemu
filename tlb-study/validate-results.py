#!/usr/bin/env python3
"""Reject incomplete or internally inconsistent TLB study results."""

import argparse
from datetime import datetime, timezone
from importlib.machinery import SourceFileLoader
import json
import math
from pathlib import Path
import re
import sys


def has_sha256(record):
    return isinstance(record, dict) and re.fullmatch(
        r"[0-9a-f]{64}", str(record.get("sha256", ""))
    ) is not None


def parse_utc(value):
    try:
        timestamp = datetime.fromisoformat(value)
    except (TypeError, ValueError):
        return None
    if timestamp.utcoffset() != timezone.utc.utcoffset(timestamp):
        return None
    return timestamp


def manifest_directories(path):
    document = json.loads(path.read_text())
    if document.get("schema_version") != 1:
        raise ValueError(f"unsupported manifest schema in {path}")
    base = path.resolve().parent
    directories = []
    for row in document.get("rows", []):
        directories.extend((base / row["perf_result"],
                            base / row["counter_result"]))
    return directories


def result_problems(summary, raw, result_dir, require_provenance=False,
                    require_ptw=False, require_nested_ptw=False,
                    require_linux_build=False,
                    qemu_version=None):
    problems = []
    if (result_dir / "failure.json").exists():
        problems.append("failure.json is present")
    if summary.get("workload_succeeded") is False:
        problems.append(
            f"remote workload failed with rc={summary.get('remote_returncode')}"
        )

    kind = summary.get("measurement_kind")
    if kind in {None, "unknown", "mixed_instrumentation"}:
        problems.append(f"invalid measurement kind: {kind}")
    if kind == "perf_timing":
        perf = summary.get("perf", {})
        if perf.get("sample_records_total", 0) <= 0:
            problems.append("perf result has no parsed sample records")
        if perf.get("sample_period_total", 0) <= 0:
            problems.append("perf result has no sample-period weight")
        named = perf.get("softmmu_named_share")
        core = perf.get("softmmu_core")
        support = perf.get("softmmu_support")
        if None not in (named, core, support) and abs(
                named - core - support) > 1e-9:
            problems.append("SoftMMU core/support shares do not sum to total")
        percentage_fields = (
            "softmmu_core", "softmmu_support", "softmmu_named_share",
            "jit_guest", "other", "classified_overhead",
        )
        for field in percentage_fields:
            value = perf.get(field)
            if (not isinstance(value, (int, float)) or
                    isinstance(value, bool) or not math.isfinite(value) or
                    not 0.0 <= value <= 100.0):
                problems.append(f"invalid perf percentage: {field}")
        classified = perf.get("classified_overhead")
        if (isinstance(classified, (int, float)) and
                not isinstance(classified, bool) and
                math.isfinite(classified) and
                abs(classified - 100.0) > 0.1):
            problems.append("perf category shares do not sum to 100 percent")
        category_values = [
            perf.get(field) for field in
            ("softmmu_core", "softmmu_support", "jit_guest", "other")
        ]
        if (all(isinstance(value, (int, float)) and
               not isinstance(value, bool) and math.isfinite(value)
               for value in category_values) and
                isinstance(classified, (int, float)) and
                not isinstance(classified, bool) and
                math.isfinite(classified) and
                abs(sum(category_values) - classified) > 1e-9):
            problems.append(
                "perf category shares do not match classified total"
            )
    if kind in {"instrumented_counter", "instrumented_window"}:
        tlb = summary.get("tlb")
        if not tlb:
            problems.append("counter result has no TLB delta")
        elif tlb.get("refill_balance_residual") != 0:
            problems.append("global refill balance residual is nonzero")
        for origin, accesses in (tlb or {}).get(
                "origin_refill_balance_residuals", {}).items():
            for access, residual in accesses.items():
                if residual != 0:
                    problems.append(
                        f"refill residual is nonzero for {origin}/{access}"
                    )
        origins = (tlb or {}).get("origins", {})
        if origins:
            for origin, accesses in origins.items():
                for access, counters in accesses.items():
                    for event, value in counters.items():
                        if value < 0:
                            problems.append(
                                f"negative TLB counter for "
                                f"{origin}/{access}/{event}"
                            )
            for event in ("l1_miss", "victim_hit", "fill_call"):
                origin_total = sum(
                    counters.get(event, 0)
                    for accesses in origins.values()
                    for counters in accesses.values()
                )
                if origin_total != tlb.get(event):
                    problems.append(
                        f"origin {event} total does not match global counter"
                    )
            origin_cache_hits = sum(
                counters.get("large_page_hit", 0)
                for accesses in origins.values()
                for counters in accesses.values()
            )
            if origin_cache_hits != tlb.get(
                    "large_page_cache", {}).get("hit", 0):
                problems.append(
                    "origin large-page hit total does not match global counter"
                )
        if tlb:
            for event in ("l1_miss", "victim_hit", "fill_call",
                          "fill_installs"):
                value = tlb.get(event)
                if value is not None and value < 0:
                    problems.append(f"negative global TLB counter: {event}")
            installs = tlb.get("fill_installs")
            fills = tlb.get("fill_call")
            cache_hits = tlb.get("large_page_cache", {}).get("hit", 0)
            if (installs is not None and fills is not None and
                    installs > fills + cache_hits):
                problems.append("TLB fill installs exceed fill calls")
            page_bits = tlb.get("page_bits")
            if isinstance(page_bits, dict) and installs is not None:
                for bits, count in page_bits.items():
                    try:
                        valid_bits = 12 <= int(bits) < 64
                    except (TypeError, ValueError):
                        valid_bits = False
                    if not valid_bits:
                        problems.append(f"invalid TLB page-size bucket: {bits}")
                    if count < 0:
                        problems.append(
                            f"negative TLB page-size bucket count: {bits}"
                        )
                if sum(page_bits.values()) != installs:
                    problems.append(
                        "TLB fill page-size buckets do not sum to installs"
                    )
        ptw_cache = summary.get("ptw_cache", {})
        mode = ptw_cache.get("mode")
        if mode not in {None, "off", "on", "probe"}:
            problems.append(f"invalid PTW cache mode: {mode}")
        flushes = ptw_cache.get("flush", 0)
        if not isinstance(flushes, int) or flushes < 0:
            problems.append("invalid PTW cache flush count")
        for level, counters in ptw_cache.get("levels", {}).items():
            if str(level) not in {"2", "3", "4"}:
                problems.append(f"invalid PTW cache level: {level}")
            values = {
                event: counters.get(event, 0)
                for event in ("lookup", "match", "hit", "insert", "eviction")
            }
            if any(not isinstance(value, int) or value < 0
                   for value in values.values()):
                problems.append(f"invalid PTW cache counter at level {level}")
                continue
            if values["hit"] > values["match"]:
                problems.append(f"PTW cache hits exceed matches at level {level}")
            if values["match"] > values["lookup"]:
                problems.append(
                    f"PTW cache matches exceed lookups at level {level}"
                )
            if values["eviction"] > values["insert"]:
                problems.append(
                    f"PTW cache evictions exceed inserts at level {level}"
                )
            if mode == "off" and any(values.values()):
                problems.append(
                    f"disabled PTW cache has activity at level {level}"
                )
            if mode == "probe" and values["hit"]:
                problems.append(f"probe PTW cache has hits at level {level}")
    if kind == "instrumented_window":
        if summary.get("guest_mem_accesses", 0) <= 0:
            problems.append("window result has no guest memory denominator")
        if summary.get("marker_windows") != 1:
            problems.append("window result does not contain exactly one window")
        if summary.get("marker_active_at_exit") is not False:
            problems.append("window marker was active at QEMU exit")

    if require_ptw or require_nested_ptw:
        ptw = summary.get("ptw", {})
        tables = summary.get("tlb_tables_after", [])
        if not tables:
            problems.append("result has no TLB capacity snapshot")
        table_keys = set()
        for table in tables:
            key = (table.get("cpu"), table.get("mmu"))
            if key in table_keys:
                problems.append("TLB snapshot has duplicate CPU/MMU table")
            table_keys.add(key)
            entries = table.get("entries", 0)
            used = table.get("used", -1)
            window_max = table.get("window_max", -1)
            if (entries < 64 or entries > (1 << 22) or
                    entries & (entries - 1)):
                problems.append("TLB table capacity is not a valid power of two")
            if not 0 <= used <= entries:
                problems.append("TLB used entries exceed table capacity")
            if not 0 <= window_max <= entries:
                problems.append("TLB window maximum exceeds table capacity")
            if not 0 <= table.get("mmu", -1) < 16:
                problems.append("TLB snapshot has invalid MMU index")
            if table.get("cpu", -1) < 0:
                problems.append("TLB snapshot has invalid CPU index")
        if tables:
            capacity = summary.get("tlb_capacity_after", {})
            expected_capacity = {
                "largest_table_entries": max(
                    table.get("entries", 0) for table in tables
                ),
                "reported_table_entries": sum(
                    table.get("entries", 0) for table in tables
                ),
                "largest_current_window_occupancy": max(
                    table.get("window_max", 0) for table in tables
                ),
            }
            for field, expected in expected_capacity.items():
                if capacity.get(field) != expected:
                    problems.append(
                        f"TLB capacity summary has inconsistent {field}"
                    )
        walks = ptw.get("walks", 0)
        if walks <= 0:
            problems.append("result has no PTW walks")
        else:
            stages = ptw.get("by_stage", {})
            primary = stages.get("primary", {})
            if primary.get("walks", 0) <= 0:
                problems.append("result has no primary-stage PTW walks")
            if (require_nested_ptw and
                    stages.get("nested", {}).get("walks", 0) <= 0):
                problems.append("result has no nested-stage PTW walks")
            if sum(stage.get("walks", 0) for stage in stages.values()) != walks:
                problems.append("PTW stage walk totals are inconsistent")
            if sum(stage.get("full_restarts", 0)
                   for stage in stages.values()) != ptw.get(
                       "full_restarts", 0):
                problems.append("PTW stage restart totals are inconsistent")
            if sum(stage.get("total_level_visits", 0)
                   for stage in stages.values()) != ptw.get(
                       "total_level_visits", 0):
                problems.append("PTW stage level totals are inconsistent")
            for stage_name, stage in stages.items():
                accesses = stage.get("by_access")
                if not isinstance(accesses, dict) or not accesses:
                    problems.append(
                        f"PTW stage {stage_name} has no access breakdown"
                    )
                    continue
                if sum(row.get("walks", 0)
                       for row in accesses.values()) != stage.get("walks", 0):
                    problems.append(
                        f"PTW stage {stage_name} access walks are inconsistent"
                    )
                if sum(row.get("full_restarts", 0)
                       for row in accesses.values()) != stage.get(
                           "full_restarts", 0):
                    problems.append(
                        f"PTW stage {stage_name} access restarts are inconsistent"
                    )
                access_levels = sum(
                    sum(row.get("levels", {}).values())
                    for row in accesses.values()
                )
                if access_levels != stage.get("total_level_visits", 0):
                    problems.append(
                        f"PTW stage {stage_name} access levels are inconsistent"
                    )
            if ptw.get("total_level_visits", 0) < walks:
                problems.append("PTW level visits are fewer than walks")
            restarts = ptw.get("full_restarts", 0)
            if ptw.get("total_level_visits", 0) < walks + restarts:
                problems.append("PTW level visits are fewer than walk attempts")
            max_level_visits = 5 * (walks + restarts)
            if ptw.get("total_level_visits", 0) > max_level_visits:
                problems.append("PTW level visits exceed x86 walk bound")
            fills = summary.get("tlb", {}).get("fill_call")
            if fills is not None and walks > fills:
                problems.append("PTW walks exceed target TLB fill calls")
    if require_linux_build:
        log_path = result_dir / "workload.log"
        log = log_path.read_text(errors="replace") if log_path.exists() else ""
        command = raw.get("guest_command", "")
        single_make = "-j1 vmlinux bzImage" in command
        separate_makes = (
            "-j1 vmlinux" in command and "-j1 bzImage" in command
        )
        if not (single_make or separate_makes):
            problems.append("guest command is not a make -j1 two-artifact build")
        if "Kernel: arch/x86/boot/bzImage is ready" not in log:
            problems.append("bzImage ready marker is missing")
        if "TLB-LINUX-BUILD-PASS" not in log:
            problems.append("final Linux build pass marker is missing")
        for artifact in ("vmlinux", "arch/x86/boot/bzImage"):
            if not re.search(rf"^[0-9a-f]{{64}}  {re.escape(artifact)}$",
                             log, re.MULTILINE):
                problems.append(f"SHA-256 is missing for {artifact}")
        if re.search(r"undefined reference|make(?:\[\d+\])?: \*\*\*|Error \d+",
                     log):
            problems.append("Linux build log contains a fatal error")
    if require_provenance:
        provenance = raw.get("provenance", {})
        for field in ("qemu_binary", "harness", "guest_disk", "seed",
                      "copied_inputs", "git", "host", "guest",
                      "run_options"):
            if field not in provenance:
                problems.append(f"provenance is missing {field}")
        for field in ("qemu_binary", "seed"):
            record = provenance.get(field)
            if isinstance(record, dict) and not has_sha256(record):
                problems.append(f"provenance {field} has no valid sha256")
        harness = provenance.get("harness")
        if isinstance(harness, dict):
            for field in ("runner", "qmp_helpers"):
                record = harness.get(field)
                if not has_sha256(record):
                    problems.append(
                        f"provenance harness has no valid {field} sha256"
                    )
        git = provenance.get("git")
        if isinstance(git, dict):
            if re.fullmatch(
                    r"(?:[0-9a-f]{40}|[0-9a-f]{64})",
                    str(git.get("head", ""))) is None:
                problems.append("provenance git has no valid HEAD")
            if git.get("status_returncode") != 0:
                problems.append("provenance git status failed")
            if git.get("tracked_diff_returncode") != 0:
                problems.append("provenance git diff failed")
            if re.fullmatch(
                    r"[0-9a-f]{64}",
                    str(git.get("tracked_diff_sha256", ""))) is None:
                problems.append(
                    "provenance git has no valid tracked diff sha256"
                )
        copied_inputs = provenance.get("copied_inputs")
        if isinstance(copied_inputs, list):
            for index, record in enumerate(copied_inputs):
                if not has_sha256(record):
                    problems.append(
                        f"provenance copied_inputs[{index}] has no valid sha256"
                    )
        run_options = provenance.get("run_options")
        if isinstance(run_options, dict):
            for field in ("cpu", "memory", "smp", "ssh_port", "snapshot"):
                if field not in run_options:
                    problems.append(
                        f"provenance run_options is missing {field}"
                    )
        host = provenance.get("host")
        if isinstance(host, dict) and isinstance(run_options, dict):
            if host.get("taskset_cpu_list") != run_options.get("cpu"):
                problems.append(
                    "provenance host CPU does not match run_options CPU"
                )
        guest = provenance.get("guest")
        if isinstance(guest, dict):
            for field in ("uname", "os_release", "packages"):
                record = guest.get(field)
                if not isinstance(record, dict) or record.get(
                        "returncode") != 0:
                    problems.append(
                        f"provenance guest {field} command failed"
                    )
        guest_disk = provenance.get("guest_disk")
        if isinstance(guest_disk, dict):
            qemu_img = guest_disk.get("qemu_img_info", {})
            if qemu_img.get("returncode") != 0:
                problems.append("provenance guest disk inspection failed")
            snapshot = (
                run_options.get("snapshot")
                if isinstance(run_options, dict) else None
            )
            launch = guest_disk.get("launch_image", {})
            if snapshot and (
                    not guest_disk.get("launch_image_hash_is_identity") or
                    not isinstance(launch, dict) or
                    not has_sha256(launch)):
                problems.append(
                    "snapshot guest disk is missing immutable launch hash"
                )
            chain = guest_disk.get("backing_chain")
            if not isinstance(chain, list) or not chain:
                problems.append("provenance guest disk has no backing chain")
            else:
                for index, record in enumerate(chain):
                    needs_hash = bool(snapshot) or index > 0
                    if needs_hash and not has_sha256(record):
                        problems.append(
                            f"provenance backing_chain[{index}] is missing "
                            "sha256"
                        )
        started = parse_utc(raw.get("measurement_started_utc"))
        ended = parse_utc(raw.get("measurement_ended_utc"))
        if started is None or ended is None:
            problems.append("measurement has no valid UTC interval")
        elif ended < started:
            problems.append("measurement UTC interval is reversed")
        if raw.get("wall_seconds", 0) <= 0:
            problems.append("measurement has no positive wall time")
        if raw.get("qemu_cpu_seconds", -1) < 0:
            problems.append("measurement has invalid QEMU CPU time")
    if qemu_version and qemu_version not in raw.get("qemu_version", ""):
        problems.append(f"QEMU version does not contain {qemu_version!r}")
    return problems


def main():
    here = Path(__file__).resolve().parent
    analyzer = SourceFileLoader(
        "analyze_results", str(here / "analyze-results.py")
    ).load_module()
    parser = argparse.ArgumentParser()
    parser.add_argument("result_dir", nargs="*", type=Path)
    parser.add_argument("--manifest", action="append", type=Path, default=[])
    parser.add_argument("--require-provenance", action="store_true")
    parser.add_argument("--require-ptw", action="store_true")
    parser.add_argument("--require-nested-ptw", action="store_true")
    parser.add_argument("--require-linux-build", action="store_true")
    parser.add_argument("--qemu-version")
    args = parser.parse_args()

    directories = list(args.result_dir)
    try:
        for manifest in args.manifest:
            directories.extend(manifest_directories(manifest))
    except (OSError, KeyError, ValueError, json.JSONDecodeError) as error:
        parser.error(str(error))
    if not directories:
        parser.error("provide result directories or --manifest")

    failed = False
    for result_dir in dict.fromkeys(path.resolve() for path in directories):
        try:
            raw = json.loads((result_dir / "result.json").read_text())
            summary = analyzer.summarize(result_dir)
            problems = result_problems(
                summary, raw, result_dir,
                require_provenance=args.require_provenance,
                require_ptw=args.require_ptw,
                require_nested_ptw=args.require_nested_ptw,
                require_linux_build=args.require_linux_build,
                qemu_version=args.qemu_version,
            )
        except (OSError, ValueError, json.JSONDecodeError) as error:
            problems = [str(error)]
            summary = {}
        if problems:
            failed = True
            for problem in problems:
                print(f"FAIL {result_dir}: {problem}", file=sys.stderr)
        else:
            print(f"OK   {result_dir} [{summary['measurement_kind']}]")
    if failed:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
