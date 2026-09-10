#!/usr/bin/env python3
"""Summarize repeated optimization-suite wall-time results."""

import argparse
from importlib.machinery import SourceFileLoader
from pathlib import Path
import re
import statistics


NAME = re.compile(r"^opt-(.+)-(base|lp|ptw|both)-r\d+$")


def ratio(numerator, denominator):
    return numerator / denominator if denominator else 0.0


def coefficient_of_variation(values):
    if len(values) < 2:
        return 0.0
    mean = statistics.mean(values)
    return statistics.stdev(values) / mean if mean else 0.0


def primary_metric(item):
    metrics = (
        ("sysbench MiB/s", "sysbench_mib_per_second", True),
        ("stress bogo/s", "stress_bogo_ops_per_second", True),
        ("DaCapo ms", "dacapo_msec", False),
        ("nested ns/access", "guest_ns_per_access", False),
    )
    for label, field, higher_is_better in metrics:
        if field in item:
            return label, field, higher_is_better
    return "wall s", "wall_seconds", False


def main():
    here = Path(__file__).resolve().parent
    analyzer = SourceFileLoader(
        "analyze_results", str(here / "analyze-results.py")
    ).load_module()
    parser = argparse.ArgumentParser()
    parser.add_argument("result_dirs", nargs="+", type=Path)
    args = parser.parse_args()
    groups = {}
    for path in args.result_dirs:
        item = analyzer.summarize(path)
        match = NAME.match(item["name"])
        if not match:
            parser.error(f"unexpected result name: {item['name']}")
        if item.get("workload_succeeded") is not True:
            parser.error(f"workload did not succeed: {path}")
        groups.setdefault(match.groups(), []).append((item, path))

    baselines = {}
    for (workload, variant), items in groups.items():
        if variant != "base":
            continue
        label, field, higher_is_better = primary_metric(items[0][0])
        values = [item[field] for item, _ in items]
        baselines[workload] = (label, field, higher_is_better,
                               statistics.median(values))
    print("| workload | variant | n | primary metric | median | CV | "
          "performance ratio | wall median (s) | wall CV | LP hit/lookup | "
          "PTW hit/lookup | TLB entries | victim | THP always |")
    print("|---|---|---:|---|---:|---:|---:|---:|---:|---:|---:|"
          "---:|:---:|:---:|")
    for (workload, variant), items in sorted(groups.items()):
        walls = [item["wall_seconds"] for item, _ in items]
        wall_median = statistics.median(walls)
        wall_cv = coefficient_of_variation(walls)
        baseline = baselines.get(workload)
        label, field, higher_is_better = primary_metric(items[0][0])
        if not all(primary_metric(item)[:2] == (label, field)
                   for item, _ in items):
            parser.error(f"inconsistent primary metric for {items}")
        values = [item[field] for item, _ in items]
        median = statistics.median(values)
        cv = coefficient_of_variation(values)
        lp_hits = lp_lookups = ptw_hits = ptw_lookups = 0
        thp_ok = True
        configs = set()
        for item, path in items:
            lp = item.get("tlb", {}).get("large_page_cache", {})
            lp_hits += lp.get("hit", 0)
            lp_lookups += lp.get("lookup", 0)
            ptw = item.get("ptw_cache", {}).get("levels", {})
            ptw_hits += sum(row.get("hit", 0) for row in ptw.values())
            ptw_lookups += sum(row.get("lookup", 0) for row in ptw.values())
            config = item.get("tlb_config", {})
            configs.add((config.get("fixed_entries"),
                         config.get("victim")))
            prep = path / "prepare.log"
            thp_ok &= prep.is_file() and "[always]" in prep.read_text()
        if len(configs) != 1 or None in next(iter(configs)):
            parser.error(f"inconsistent or missing TLB config for {items}")
        tlb_entries, victim = configs.pop()
        if baseline:
            base_label, base_field, base_higher, base_median = baseline
            if (label, field, higher_is_better) != (
                    base_label, base_field, base_higher):
                parser.error(f"primary metric differs from baseline: {items}")
            if higher_is_better:
                performance_ratio = ratio(median, base_median)
            else:
                performance_ratio = ratio(base_median, median)
        else:
            performance_ratio = None
        ratio_text = (f"{performance_ratio:.3f}x"
                      if performance_ratio is not None else "-")
        print(f"| {workload} | {variant} | {len(items)} | {label} | "
              f"{median:.3f} | {100 * cv:.2f}% | {ratio_text} | "
              f"{wall_median:.3f} | {100 * wall_cv:.2f}% | "
              f"{100 * ratio(lp_hits, lp_lookups):.2f}% | "
              f"{100 * ratio(ptw_hits, ptw_lookups):.2f}% | "
              f"{tlb_entries or 'dynamic'} | {victim} | "
              f"{'yes' if thp_ok else 'no'} |")


if __name__ == "__main__":
    main()
