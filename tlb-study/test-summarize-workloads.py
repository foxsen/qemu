#!/usr/bin/env python3
"""Regression tests for joining independent workload measurements."""

import csv
from importlib.machinery import SourceFileLoader
import io
import json
from pathlib import Path
import tempfile
import unittest


HERE = Path(__file__).resolve().parent
SUMMARY = SourceFileLoader(
    "summarize_workloads", str(HERE / "summarize-workloads.py")
).load_module()


class FakeAnalyzer:
    @staticmethod
    def summarize(path):
        if path.name == "perf":
            return {
                "measurement_kind": "perf_timing",
                "wall_seconds": 10.0,
                "perf": {
                    "sample_records_total": 1000,
                    "softmmu_named_share": 12.5,
                    "softmmu_core": 10.0,
                    "softmmu_support": 2.5,
                },
            }
        return {
            "measurement_kind": "instrumented_window",
            "wall_seconds": 99.0,
            "guest_mem_accesses": 10000,
            "tlb": {
                "data_l1_miss_per_guest_mem_access": 0.01,
                "data_fill_per_guest_mem_access": 0.0025,
                "data_victim_hits_per_l1_miss": 0.75,
                "data_fill_calls_per_l1_miss": 0.25,
                "refill_balance_residual": 0,
            },
            "tlb_capacity_after": {"largest_table_entries": 2048},
            "ptw": {
                "walks": 100,
                "level_visits_per_walk": 3.5,
                "full_restarts_per_walk": 0.01,
                "by_stage": {
                    "primary": {"walks": 80},
                    "nested": {"walks": 20},
                },
            },
        }


class WorkloadSummaryTest(unittest.TestCase):
    def setUp(self):
        self.row = SUMMARY.build_row(
            FakeAnalyzer(), "example", Path("perf"), Path("counter")
        )

    def test_joins_perf_time_with_counter_rates(self):
        self.assertEqual(self.row["perf_wall_seconds"], 10.0)
        self.assertFalse(self.row["wall_times_comparable"])
        self.assertEqual(self.row["softmmu_named_percent"], 12.5)
        self.assertEqual(self.row["softmmu_core_percent"], 10.0)
        self.assertEqual(self.row["softmmu_support_percent"], 2.5)
        self.assertEqual(self.row["data_fill_per_mem"], 0.0025)
        self.assertEqual(self.row["ptw_walks_per_mem"], 0.01)
        self.assertEqual(self.row["nested_ptw_share"], 0.2)
        self.assertEqual(self.row["largest_table_entries_after"], 2048)

    def test_markdown_scales_only_fraction_fields(self):
        text = SUMMARY.markdown([self.row])
        self.assertIn("| example | 12.50% | 1.000% | 0.2500% |", text)
        self.assertIn(
            "75.00% | 25.00% | 1.0000% | 3.50 | 20.00% | 2048",
            text,
        )

    def test_csv_round_trip(self):
        parsed = next(csv.DictReader(io.StringIO(SUMMARY.csv_text([self.row]))))
        self.assertEqual(parsed["workload"], "example")
        self.assertEqual(parsed["perf_wall_seconds"], "10.0")

    def test_rejects_instrumented_result_in_perf_slot(self):
        class WrongAnalyzer(FakeAnalyzer):
            @staticmethod
            def summarize(path):
                result = FakeAnalyzer.summarize(path)
                if path.name == "perf":
                    result["measurement_kind"] = "instrumented_counter"
                return result

        with self.assertRaisesRegex(ValueError, "not an uninstrumented perf"):
            SUMMARY.build_row(
                WrongAnalyzer(), "bad", Path("perf"), Path("counter")
            )

    def test_rejects_failed_workload(self):
        class FailedAnalyzer(FakeAnalyzer):
            @staticmethod
            def summarize(path):
                result = FakeAnalyzer.summarize(path)
                if path.name == "counter":
                    result["workload_succeeded"] = False
                return result

        with self.assertRaisesRegex(ValueError, "failed workload"):
            SUMMARY.build_row(
                FailedAnalyzer(), "bad", Path("perf"), Path("counter")
            )

    def test_manifest_paths_are_relative_to_manifest(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            manifest = root / "results.json"
            manifest.write_text(json.dumps({
                "schema_version": 1,
                "rows": [{
                    "label": "example",
                    "perf_result": "results/perf",
                    "counter_result": "results/counter",
                }],
            }))
            self.assertEqual(SUMMARY.manifest_rows(manifest), [(
                "example",
                root / "results/perf",
                root / "results/counter",
            )])


if __name__ == "__main__":
    unittest.main()
