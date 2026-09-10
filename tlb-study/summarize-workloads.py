#!/usr/bin/env python3
"""Join independent perf and counter runs into a compact workload table."""

import argparse
import csv
from importlib.machinery import SourceFileLoader
import io
import json
from pathlib import Path


FIELDS = (
    "workload", "perf_result", "counter_result", "perf_measurement_kind",
    "counter_measurement_kind", "wall_times_comparable", "perf_wall_seconds",
    "perf_sample_records", "softmmu_named_percent", "softmmu_core_percent",
    "softmmu_support_percent", "guest_mem_accesses",
    "data_l1_miss_per_mem", "data_fill_per_mem",
    "data_victim_per_l1_miss", "data_fill_per_l1_miss",
    "ptw_walks", "ptw_walks_per_mem", "ptw_level_visits_per_walk",
    "ptw_full_restarts_per_walk", "primary_ptw_walks",
    "nested_ptw_walks", "nested_ptw_share",
    "largest_table_entries_after", "refill_balance_residual",
)


def build_row(analyzer, label, perf_dir, counter_dir):
    perf = analyzer.summarize(perf_dir)
    counter = analyzer.summarize(counter_dir)
    if perf.get("workload_succeeded") is False:
        raise ValueError(f"{perf_dir} contains a failed workload")
    if counter.get("workload_succeeded") is False:
        raise ValueError(f"{counter_dir} contains a failed workload")
    if perf.get("measurement_kind") != "perf_timing":
        raise ValueError(f"{perf_dir} is not an uninstrumented perf result")
    if counter.get("measurement_kind") not in {
            "instrumented_counter", "instrumented_window"}:
        raise ValueError(f"{counter_dir} is not a counter/window result")
    perf_stats = perf.get("perf", {})
    tlb = counter.get("tlb", {})
    ptw = counter.get("ptw", {})
    ptw_stages = ptw.get("by_stage", {})
    ptw_walks = ptw.get("walks")
    guest_mem = counter.get("guest_mem_accesses")
    primary_walks = ptw_stages.get("primary", {}).get("walks")
    nested_walks = ptw_stages.get("nested", {}).get("walks")
    capacity = counter.get("tlb_capacity_after", {})
    return {
        "workload": label,
        "perf_result": str(perf_dir),
        "counter_result": str(counter_dir),
        "perf_measurement_kind": perf["measurement_kind"],
        "counter_measurement_kind": counter["measurement_kind"],
        "wall_times_comparable": False,
        "perf_wall_seconds": perf.get("wall_seconds"),
        "perf_sample_records": perf_stats.get("sample_records_total"),
        "softmmu_named_percent": perf_stats.get("softmmu_named_share"),
        "softmmu_core_percent": perf_stats.get("softmmu_core"),
        "softmmu_support_percent": perf_stats.get("softmmu_support"),
        "guest_mem_accesses": guest_mem,
        "data_l1_miss_per_mem": tlb.get(
            "data_l1_miss_per_guest_mem_access"
        ),
        "data_fill_per_mem": tlb.get("data_fill_per_guest_mem_access"),
        "data_victim_per_l1_miss": tlb.get(
            "data_victim_hits_per_l1_miss"
        ),
        "data_fill_per_l1_miss": tlb.get("data_fill_calls_per_l1_miss"),
        "ptw_walks": ptw_walks,
        "ptw_walks_per_mem": (
            ptw_walks / guest_mem if ptw_walks is not None and guest_mem else None
        ),
        "ptw_level_visits_per_walk": ptw.get("level_visits_per_walk"),
        "ptw_full_restarts_per_walk": ptw.get("full_restarts_per_walk"),
        "primary_ptw_walks": primary_walks,
        "nested_ptw_walks": nested_walks,
        "nested_ptw_share": (
            nested_walks / ptw_walks
            if nested_walks is not None and ptw_walks else None
        ),
        "largest_table_entries_after": capacity.get(
            "largest_table_entries"
        ),
        "refill_balance_residual": tlb.get("refill_balance_residual"),
    }


def percent(value, digits=3):
    return "-" if value is None else f"{100 * value:.{digits}f}%"


def markdown(rows):
    output = [
        "| workload | SoftMMU time | data L1 miss / mem | data refill / mem "
        "| victim / L1 miss | refill / L1 miss | PTW / mem | levels / PTW "
        "| nested PTW | ending max table |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for row in rows:
        softmmu = row["softmmu_named_percent"]
        levels_per_walk = row["ptw_level_visits_per_walk"]
        levels_text = (
            "-" if levels_per_walk is None else f"{levels_per_walk:.2f}"
        )
        output.append(
            f"| {row['workload']} | "
            f"{('-' if softmmu is None else f'{softmmu:.2f}%')} | "
            f"{percent(row['data_l1_miss_per_mem'])} | "
            f"{percent(row['data_fill_per_mem'], 4)} | "
            f"{percent(row['data_victim_per_l1_miss'], 2)} | "
            f"{percent(row['data_fill_per_l1_miss'], 2)} | "
            f"{percent(row['ptw_walks_per_mem'], 4)} | "
            f"{levels_text} | "
            f"{percent(row['nested_ptw_share'], 2)} | "
            f"{row['largest_table_entries_after'] or '-'} |"
        )
    return "\n".join(output) + "\n"


def csv_text(rows):
    stream = io.StringIO()
    writer = csv.DictWriter(stream, fieldnames=FIELDS)
    writer.writeheader()
    writer.writerows(rows)
    return stream.getvalue()


def manifest_rows(path):
    document = json.loads(path.read_text())
    if document.get("schema_version") != 1:
        raise ValueError(f"unsupported manifest schema in {path}")
    base = path.resolve().parent
    rows = []
    for entry in document.get("rows", []):
        if set(entry) != {"label", "perf_result", "counter_result"}:
            raise ValueError(f"invalid result row in {path}: {entry}")
        rows.append((
            entry["label"],
            base / entry["perf_result"],
            base / entry["counter_result"],
        ))
    if not rows:
        raise ValueError(f"result manifest has no rows: {path}")
    return rows


def main():
    here = Path(__file__).resolve().parent
    analyzer = SourceFileLoader(
        "analyze_results", str(here / "analyze-results.py")
    ).load_module()
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--row", action="append", nargs=3,
        metavar=("LABEL", "PERF_DIR", "COUNTER_DIR"),
    )
    parser.add_argument(
        "--manifest", type=Path,
        help="JSON manifest containing canonical result-directory pairs",
    )
    parser.add_argument("--format", choices=("markdown", "csv", "json"),
                        default="markdown")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    if not args.row and not args.manifest:
        parser.error("provide --row or --manifest")
    requested = list(args.row or [])
    if args.manifest:
        try:
            requested.extend(manifest_rows(args.manifest))
        except (OSError, ValueError, json.JSONDecodeError) as error:
            parser.error(str(error))

    rows = []
    for label, perf_name, counter_name in requested:
        perf_dir = Path(perf_name)
        counter_dir = Path(counter_name)
        for path in (perf_dir, counter_dir):
            if not (path / "result.json").is_file():
                parser.error(f"result.json missing from {path}")
        try:
            rows.append(build_row(analyzer, label, perf_dir, counter_dir))
        except ValueError as error:
            parser.error(str(error))

    if args.format == "json":
        text = json.dumps(rows, indent=2, sort_keys=True) + "\n"
    elif args.format == "csv":
        text = csv_text(rows)
    else:
        text = markdown(rows)
    if args.output:
        args.output.write_text(text)
    print(text, end="")


if __name__ == "__main__":
    main()
