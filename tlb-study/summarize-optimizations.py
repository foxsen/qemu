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

    baselines = {
        workload: {
            "wall": statistics.median(
                item["wall_seconds"] for item, _ in items
            ),
            "stress_rate": statistics.median(
                item["stress_bogo_ops_per_second"] for item, _ in items
            ) if all("stress_bogo_ops_per_second" in item
                     for item, _ in items) else None,
        }
        for (workload, variant), items in groups.items() if variant == "base"
    }
    print("| workload | variant | n | wall median (s) | "
          "stress bogo ops/s | performance ratio | LP hit/lookup | "
          "PTW hit/lookup | THP always |")
    print("|---|---|---:|---:|---:|---:|---:|---:|:---:|")
    for (workload, variant), items in sorted(groups.items()):
        walls = [item["wall_seconds"] for item, _ in items]
        median = statistics.median(walls)
        baseline = baselines.get(workload)
        stress_rates = [item["stress_bogo_ops_per_second"]
                        for item, _ in items
                        if "stress_bogo_ops_per_second" in item]
        stress_rate = (statistics.median(stress_rates)
                       if len(stress_rates) == len(items) else None)
        lp_hits = lp_lookups = ptw_hits = ptw_lookups = 0
        thp_ok = True
        for item, path in items:
            lp = item.get("tlb", {}).get("large_page_cache", {})
            lp_hits += lp.get("hit", 0)
            lp_lookups += lp.get("lookup", 0)
            ptw = item.get("ptw_cache", {}).get("levels", {})
            ptw_hits += sum(row.get("hit", 0) for row in ptw.values())
            ptw_lookups += sum(row.get("lookup", 0) for row in ptw.values())
            prep = path / "prepare.log"
            thp_ok &= prep.is_file() and "[always]" in prep.read_text()
        if baseline and stress_rate is not None:
            performance_ratio = stress_rate / baseline["stress_rate"]
        elif baseline:
            performance_ratio = baseline["wall"] / median
        else:
            performance_ratio = None
        stress_text = f"{stress_rate:.2f}" if stress_rate is not None else "-"
        ratio_text = (f"{performance_ratio:.3f}x"
                      if performance_ratio is not None else "-")
        print(f"| {workload} | {variant} | {len(items)} | {median:.3f} | "
              f"{stress_text} | {ratio_text} | "
              f"{100 * ratio(lp_hits, lp_lookups):.2f}% | "
              f"{100 * ratio(ptw_hits, ptw_lookups):.2f}% | "
              f"{'yes' if thp_ok else 'no'} |")


if __name__ == "__main__":
    main()
