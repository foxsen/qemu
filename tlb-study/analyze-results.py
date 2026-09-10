#!/usr/bin/env python3
"""Summarize boundary-delimited TLB counters and flat perf samples."""

import argparse
import json
from pathlib import Path
import re


SOFTMMU_CORE = re.compile(
    r"^(?:helper_.*_mmu|do_(?:ld|st).*_mmu|"
    r"atomic_mmu_lookup|cpu_atomic_.*_mmu|"
    r"cpu_(?:ld|st).*_(?:mmu|mmuidx_ra)|"
    r"mmu_lookup\d*|victim_tlb_hit|"
    r"tlb_(?:fill|set_page|flush|mmu_|reset_dirty).*|"
    r"x86_cpu_tlb_fill|probe_access.*|ptw_translate.*|mmu_translate.*|"
    r"address_space_translate_(?:cached.*|for_iotlb|internal|iommu))$"
)
SOFTMMU_SUPPORT = re.compile(
    r"^(?:cpu_physical_memory_get_dirty.*|memory_region_get_ram.*|"
    r"qemu_map_ram_ptr|get_ptr_rcu_reader|find_next_bit|"
    r"(?:cpu|x86)_asidx_from_attrs|get_pg_mode)$"
)
PERF_MAP = re.compile(r"(?:^|/)perf-\d+\.map$")
QEMU_DSO = re.compile(r"(?:^|/)qemu-system-[^/]+$")


def is_jit_sample(symbol, dso):
    return (symbol.startswith("guest-") or symbol == "tcg-prologue-buffer" or
            (symbol in {"[unknown]", "unknown"} and
             (dso in {"[unknown]", "unknown"} or PERF_MAP.search(dso))))


def is_qemu_sample(dso):
    """Limit named SoftMMU matches to the QEMU executable.

    Empty DSO names are accepted for legacy flat reports that did not retain
    the DSO column. Raw perf-script data always contains the DSO and therefore
    cannot accidentally classify a same-named host-kernel function.
    """
    return not dso or bool(QEMU_DSO.search(dso))


def parse_console(path):
    text = path.read_text(errors="replace")
    result = {}
    match = re.search(
        r"result .*? elapsed_ns=(\d+) accesses=(\d+) "
        r"ns_per_access=([0-9.]+)", text
    )
    if match:
        result.update({
            "guest_elapsed_ns": int(match.group(1)),
            "guest_accesses": int(match.group(2)),
            "guest_ns_per_access": float(match.group(3)),
        })
    stress_matches = re.findall(
        r"^stress-ng: metrc:.*?\btlb-shootdown\s+(\d+)\s+"
        r"([0-9.]+)\s+[0-9.]+\s+[0-9.]+\s+"
        r"([0-9.]+)\s+([0-9.]+)\s*$",
        text, re.MULTILINE,
    )
    if stress_matches:
        bogo_ops, real_seconds, real_rate, cpu_rate = stress_matches[-1]
        result.update({
            "stress_bogo_ops": int(bogo_ops),
            "stress_real_seconds": float(real_seconds),
            "stress_bogo_ops_per_second": float(real_rate),
            "stress_bogo_ops_per_cpu_second": float(cpu_rate),
        })
    sysbench_matches = re.findall(
        r"MiB transferred \(([0-9.]+) MiB/sec\)", text
    )
    if sysbench_matches:
        result["sysbench_mib_per_second"] = float(sysbench_matches[-1])
    dacapo_matches = re.findall(
        r"^===== DaCapo .*? PASSED in (\d+) msec =====$",
        text, re.MULTILINE,
    )
    if dacapo_matches:
        result["dacapo_msec"] = int(dacapo_matches[-1])
    return result


def parse_perf(path):
    totals = {"softmmu_core": 0.0, "softmmu_support": 0.0,
              "jit_guest": 0.0, "other": 0.0}
    symbols = []
    for line in path.read_text(errors="replace").splitlines():
        flat = re.match(
            r"^\s*([0-9.]+)%\s*;\s*([^;]+)\s*;\s*([^;]+)\s*;?\s*$",
            line,
        )
        legacy = re.match(
            r"^\s*([0-9.]+)%\s+.*?\s+\[(?:\.|k)\]\s+(.+?)\s*$", line
        )
        match = flat or legacy
        if not match:
            continue
        overhead = float(match.group(1))
        symbol = match.group(2).strip()
        dso = match.group(3).strip() if flat else ""
        if is_jit_sample(symbol, dso):
            category = "jit_guest"
        elif is_qemu_sample(dso) and SOFTMMU_CORE.match(symbol):
            category = "softmmu_core"
        elif is_qemu_sample(dso) and SOFTMMU_SUPPORT.match(symbol):
            category = "softmmu_support"
        else:
            category = "other"
        totals[category] += overhead
        symbols.append((overhead, symbol, category))
    totals["softmmu_named_share"] = (
        totals["softmmu_core"] + totals["softmmu_support"]
    )
    totals["classified_overhead"] = sum(
        totals[key] for key in ("softmmu_core", "softmmu_support",
                                "jit_guest", "other")
    )
    totals["top_softmmu_symbols"] = [
        {"symbol": symbol, "overhead_percent": overhead,
         "category": category}
        for overhead, symbol, category in sorted(symbols, reverse=True)
        if category.startswith("softmmu")
    ][:15]
    return totals


def parse_perf_script(path):
    weights = {"softmmu_core": 0, "softmmu_support": 0,
               "jit_guest": 0, "other": 0}
    records = {key: 0 for key in weights}
    softmmu_symbols = {}
    for line in path.read_text(errors="replace").splitlines():
        match = re.match(
            r"^\s*(\d+)\s+[0-9a-f]+\s+(.+?)\s+\((.*?)\)\s*$", line
        )
        if not match:
            continue
        weight = int(match.group(1))
        symbol = match.group(2).strip()
        dso = match.group(3).strip()
        if is_jit_sample(symbol, dso):
            category = "jit_guest"
        elif is_qemu_sample(dso) and SOFTMMU_CORE.match(symbol):
            category = "softmmu_core"
        elif is_qemu_sample(dso) and SOFTMMU_SUPPORT.match(symbol):
            category = "softmmu_support"
        else:
            category = "other"
        weights[category] += weight
        records[category] += 1
        if category.startswith("softmmu"):
            key = (symbol, category)
            softmmu_symbols[key] = softmmu_symbols.get(key, 0) + weight
    total = sum(weights.values())
    result = ({key: value * 100 / total for key, value in weights.items()}
              if total else {key: 0.0 for key in weights})
    result["softmmu_named_share"] = (
        result["softmmu_core"] + result["softmmu_support"]
    )
    result["classified_overhead"] = 100.0 if total else 0.0
    result["sample_period_total"] = total
    result["sample_records_total"] = sum(records.values())
    result["sample_records_by_category"] = records
    result["top_softmmu_symbols"] = [
        {"symbol": symbol, "overhead_percent": weight * 100 / total,
         "category": category}
        for (symbol, category), weight in sorted(
            softmmu_symbols.items(), key=lambda item: item[1], reverse=True
        )[:15]
    ] if total else []
    return result


def sum_access(stats, key):
    return sum(stats.get(kind, {}).get(key, 0)
               for kind in ("load", "store", "fetch"))


def sum_guest_data_path(stats, key):
    origins = stats.get("origins", {})
    return sum(origins.get(origin, {}).get(kind, {}).get(key, 0)
               for origin in ("helper", "atomic")
               for kind in ("load", "store"))


def refill_balance(stats):
    balances = {}
    for origin, kinds in stats.get("origins", {}).items():
        balances[origin] = {}
        for kind, counters in kinds.items():
            balances[origin][kind] = (
                counters.get("l1_miss", 0) -
                counters.get("victim_hit", 0) -
                counters.get("fill_call", 0) -
                counters.get("large_page_hit", 0)
            )
    return balances


def summarize_ptw(ptw):
    """Aggregate stage- and access-separated x86 page-walk counters."""
    stages = {}
    for stage, accesses in ptw.items():
        rows = list(accesses.values())
        walks = sum(row.get("walks", 0) for row in rows)
        full_restarts = sum(row.get("full_restarts", 0) for row in rows)
        levels = {
            str(level): sum(row.get("levels", {}).get(str(level), 0)
                            for row in rows)
            for level in range(1, 6)
        }
        level_visits = sum(levels.values())
        stages[stage] = {
            "walks": walks,
            "full_restarts": full_restarts,
            "level_visits": levels,
            "total_level_visits": level_visits,
            "level_visits_per_walk": (
                level_visits / walks if walks else None
            ),
            "full_restarts_per_walk": (
                full_restarts / walks if walks else None
            ),
            "by_access": accesses,
        }

    walks = sum(stage["walks"] for stage in stages.values())
    full_restarts = sum(stage["full_restarts"] for stage in stages.values())
    levels = {
        str(level): sum(stage["level_visits"][str(level)]
                        for stage in stages.values())
        for level in range(1, 6)
    }
    level_visits = sum(levels.values())
    return {
        "walks": walks,
        "full_restarts": full_restarts,
        "level_visits": levels,
        "total_level_visits": level_visits,
        "level_visits_per_walk": level_visits / walks if walks else None,
        "full_restarts_per_walk": (
            full_restarts / walks if walks else None
        ),
        "by_stage": stages,
    }


def parse_window_count(*paths):
    text = "".join(path.read_text(errors="replace")
                   for path in paths if path.exists())
    matches = re.findall(
        r"window mem accesses: (\d+) windows: (\d+) active: (\d+)",
        text,
    )
    if not matches:
        return {}
    accesses, windows, active = matches[-1]
    return {
        "guest_mem_accesses": int(accesses),
        "marker_windows": int(windows),
        "marker_active_at_exit": bool(int(active)),
    }


def parse_tlb_tables(text):
    return [
        {
            "cpu": int(cpu),
            "mmu": int(mmu),
            "entries": int(entries),
            "used": int(used),
            "window_max": int(window_max),
        }
        for cpu, mmu, entries, used, window_max in re.findall(
            r"TLB cpu=(\d+) mmu=(\d+) entries=(\d+) used=(\d+) "
            r"window_max=(\d+)",
            text or "",
        )
    ]


def summarize(result_dir):
    result = json.loads((result_dir / "result.json").read_text())
    workload_log = result_dir / "workload.log"
    guest_output = {
        **parse_console(result_dir / "console.log"),
        **(parse_console(workload_log) if workload_log.is_file() else {}),
    }
    summary = {
        "name": result.get("name", result_dir.name),
        "wall_seconds": result.get("wall_seconds"),
        **guest_output,
        **parse_window_count(result_dir / "qemu.stdout",
                             result_dir / "qemu.stderr",
                             result_dir / "plugin.log"),
    }
    if "remote_returncode" in result:
        summary["remote_returncode"] = result["remote_returncode"]
        summary["workload_succeeded"] = (
            result.get("remote_done") is not None and
            result["remote_returncode"] == 0
        )
    tables = parse_tlb_tables(result.get("info_jit_after", ""))
    if tables:
        summary["tlb_tables_after"] = tables
        summary["tlb_capacity_after"] = {
            "largest_table_entries": max(table["entries"]
                                         for table in tables),
            "reported_table_entries": sum(table["entries"]
                                          for table in tables),
            "largest_current_window_occupancy": max(
                table["window_max"] for table in tables
            ),
        }
    delta = result.get("tlb_delta", {})
    tlb_config = result.get("tlb_after", {}).get("tlb_config")
    if tlb_config:
        summary["tlb_config"] = tlb_config
    accesses = summary.get("guest_accesses", 0)
    if delta:
        l1_miss = sum_access(delta, "l1_miss")
        victim_hit = sum_access(delta, "victim_hit")
        fill_call = sum_access(delta, "fill_call")
        large_page_cache = delta.get("large_page_cache", {})
        large_page_hit = large_page_cache.get("hit", 0)
        summary["tlb"] = {
            "l1_miss": l1_miss,
            "victim_hit": victim_hit,
            "fill_call": fill_call,
            "refill_balance_residual": (
                l1_miss - victim_hit - fill_call - large_page_hit
            ),
            "origin_refill_balance_residuals": refill_balance(delta),
            "fill_installs": delta.get("fill_installs", 0),
            "page_bits": delta.get("page_bits", {}),
            "origins": delta.get("origins", {}),
            "large_page_cache": large_page_cache,
        }
        if l1_miss:
            summary["tlb"]["victim_hits_per_l1_miss"] = (
                victim_hit / l1_miss
            )
            summary["tlb"]["fill_calls_per_l1_miss"] = (
                fill_call / l1_miss
            )
        data_l1_miss = sum_guest_data_path(delta, "l1_miss")
        data_victim_hit = sum_guest_data_path(delta, "victim_hit")
        data_fill_call = sum_guest_data_path(delta, "fill_call")
        if data_l1_miss:
            summary["tlb"]["data_victim_hits_per_l1_miss"] = (
                data_victim_hit / data_l1_miss
            )
            summary["tlb"]["data_fill_calls_per_l1_miss"] = (
                data_fill_call / data_l1_miss
            )
        helper = delta.get("origins", {}).get("helper", {})
        helper_load_miss = helper.get("load", {}).get("l1_miss", 0)
        helper_load_fill = helper.get("load", {}).get("fill_call", 0)
        if accesses:
            summary["tlb"]["helper_load_l1_miss_per_access"] = (
                helper_load_miss / accesses
            )
            summary["tlb"]["helper_load_fill_per_access"] = (
                helper_load_fill / accesses
            )
            summary["tlb"]["all_fill_per_access"] = (
                summary["tlb"]["fill_call"] / accesses
            )
        guest_mem = summary.get("guest_mem_accesses", 0)
        if guest_mem:
            summary["tlb"]["all_l1_miss_events_per_guest_mem_access"] = (
                summary["tlb"]["l1_miss"] / guest_mem
            )
            summary["tlb"]["all_fill_events_per_guest_mem_access"] = (
                summary["tlb"]["fill_call"] / guest_mem
            )
            summary["tlb"]["data_l1_miss_per_guest_mem_access"] = (
                data_l1_miss / guest_mem
            )
            summary["tlb"]["data_fill_per_guest_mem_access"] = (
                data_fill_call / guest_mem
            )
        ptw = delta.get("ptw", {})
        if ptw:
            summary["ptw"] = summarize_ptw(ptw)
        if delta.get("ptw_cache"):
            summary["ptw_cache"] = delta["ptw_cache"]
    perf_script = result_dir / "perf-script.txt"
    perf_report = result_dir / "perf-report.txt"
    if perf_script.exists():
        summary["perf"] = parse_perf_script(perf_script)
    elif perf_report.exists():
        summary["perf"] = parse_perf(perf_report)
    has_window = "guest_mem_accesses" in summary
    has_perf = "perf" in summary
    has_profile = bool(delta.get("origins"))
    if has_perf and (has_window or has_profile):
        summary["measurement_kind"] = "mixed_instrumentation"
    elif has_perf:
        summary["measurement_kind"] = "perf_timing"
    elif has_window:
        summary["measurement_kind"] = "instrumented_window"
    elif has_profile:
        summary["measurement_kind"] = "instrumented_counter"
    elif ("remote_returncode" in result and
          isinstance(result.get("wall_seconds"), (int, float))):
        summary["measurement_kind"] = "wall_timing"
    else:
        summary["measurement_kind"] = "unclassified"
    return summary


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("result_dirs", nargs="+", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    summaries = [summarize(path) for path in args.result_dirs]
    text = json.dumps(summaries, indent=2, sort_keys=True) + "\n"
    if args.output:
        args.output.write_text(text)
    print(text, end="")


if __name__ == "__main__":
    main()
