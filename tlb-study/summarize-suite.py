#!/usr/bin/env python3
"""Aggregate repeated result directories into a compact Markdown table."""

import argparse
from pathlib import Path
import re
import statistics

from importlib.machinery import SourceFileLoader


def ci95(values):
    if len(values) < 2:
        return 0.0
    # Normal approximation is reported as a descriptive interval, not a test.
    return 1.96 * statistics.stdev(values) / len(values) ** 0.5


def fmt(values, scale=1.0):
    values = [value * scale for value in values]
    return (f"{statistics.median(values):.3f} "
            f"(mean {statistics.mean(values):.3f} +/- {ci95(values):.3f})")


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
        key = re.sub(r"-r\d+$", "", item["name"])
        groups.setdefault(key, []).append(item)
    print("| workload | n | guest ns/access | helper load L1 miss/access | "
          "helper load fill/access | named SoftMMU share |")
    print("|---|---:|---:|---:|---:|---:|")
    for key, items in sorted(groups.items()):
        ns = [item["guest_ns_per_access"] for item in items
              if "guest_ns_per_access" in item]
        misses = [item["tlb"]["helper_load_l1_miss_per_access"]
                  for item in items if item.get("tlb", {}).get("origins")]
        fills = [item["tlb"]["helper_load_fill_per_access"]
                 for item in items if item.get("tlb", {}).get("origins")]
        perf = [item["perf"]["softmmu_named_share"]
                for item in items if "perf" in item]
        print(f"| {key} | {len(items)} | {fmt(ns) if ns else '-'} | "
              f"{fmt(misses, 100) + '%' if misses else '-'} | "
              f"{fmt(fills, 100) + '%' if fills else '-'} | "
              f"{fmt(perf) + '%' if perf else '-'} |")


if __name__ == "__main__":
    main()
