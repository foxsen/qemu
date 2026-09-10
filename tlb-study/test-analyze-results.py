#!/usr/bin/env python3
"""Focused regression tests for perf sample classification."""

from importlib.machinery import SourceFileLoader
import json
from pathlib import Path
import tempfile
import unittest


HERE = Path(__file__).resolve().parent
ANALYZER = SourceFileLoader(
    "analyze_results", str(HERE / "analyze-results.py")
).load_module()


class PerfClassificationTest(unittest.TestCase):
    def test_cloud_result_records_failed_remote_command(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            (root / "console.log").write_text("")
            (root / "result.json").write_text(json.dumps({
                "name": "failed",
                "remote_done": "TLB-CLOUD-DONE rc=2",
                "remote_returncode": 2,
            }))
            summary = ANALYZER.summarize(root)
            self.assertFalse(summary["workload_succeeded"])
            self.assertEqual(summary["remote_returncode"], 2)

    def test_plain_cloud_result_is_wall_timing(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            (root / "console.log").write_text("")
            (root / "result.json").write_text(json.dumps({
                "name": "timing",
                "remote_done": "TLB-CLOUD-DONE rc=0",
                "remote_returncode": 0,
                "wall_seconds": 1.25,
            }))
            summary = ANALYZER.summarize(root)
            self.assertEqual(summary["measurement_kind"], "wall_timing")

    def test_nested_guest_result_is_read_from_workload_log(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            (root / "console.log").write_text("")
            (root / "workload.log").write_text(
                "result mode=random mib=64 passes=32 elapsed_ns=262144000 "
                "accesses=524288 ns_per_access=500.000 checksum=1\n"
            )
            (root / "result.json").write_text(json.dumps({
                "name": "nested", "remote_done": "TLB-CLOUD-DONE rc=0",
                "remote_returncode": 0, "wall_seconds": 12.0,
            }))
            summary = ANALYZER.summarize(root)
            self.assertEqual(summary["guest_ns_per_access"], 500.0)

    def test_stress_throughput_is_read_from_workload_log(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            (root / "console.log").write_text("")
            (root / "workload.log").write_text(
                "stress-ng: metrc: [494] tlb-shootdown      4606     "
                "11.01      5.70      5.20       418.33         422.40\n"
            )
            (root / "result.json").write_text(json.dumps({
                "name": "stress", "remote_done": "TLB-CLOUD-DONE rc=0",
                "remote_returncode": 0, "wall_seconds": 12.0,
            }))
            summary = ANALYZER.summarize(root)
            self.assertEqual(summary["stress_bogo_ops"], 4606)
            self.assertEqual(summary["stress_bogo_ops_per_second"], 418.33)
            self.assertEqual(summary["stress_bogo_ops_per_cpu_second"], 422.40)

    def test_ptw_summary_separates_primary_and_nested_walks(self):
        summary = ANALYZER.summarize_ptw({
            "primary": {
                "load": {
                    "walks": 10,
                    "full_restarts": 2,
                    "levels": {"1": 8, "2": 10, "3": 10, "4": 10},
                },
            },
            "nested": {
                "store": {
                    "walks": 3,
                    "full_restarts": 1,
                    "levels": {"1": 3, "2": 3, "3": 3, "4": 3},
                },
            },
        })
        self.assertEqual(summary["walks"], 13)
        self.assertEqual(summary["full_restarts"], 3)
        self.assertEqual(summary["level_visits"]["1"], 11)
        self.assertEqual(summary["by_stage"]["primary"]["walks"], 10)
        self.assertEqual(summary["by_stage"]["nested"]["walks"], 3)
        self.assertAlmostEqual(summary["level_visits_per_walk"], 50 / 13)

    def test_tlb_table_parser(self):
        tables = ANALYZER.parse_tlb_tables(
            "TLB cpu=0 mmu=2 entries=4096 used=10 window_max=126\n"
        )
        self.assertEqual(tables, [{
            "cpu": 0, "mmu": 2, "entries": 4096,
            "used": 10, "window_max": 126,
        }])

    def test_refill_balance_exposes_exceptional_fill(self):
        stats = {"origins": {"atomic": {"load": {
            "l1_miss": 10, "victim_hit": 3, "fill_call": 8,
        }}}}
        self.assertEqual(
            ANALYZER.refill_balance(stats)["atomic"]["load"], -1
        )

    def test_large_page_hit_balances_refill_path(self):
        stats = {"origins": {"helper": {"load": {
            "l1_miss": 10, "victim_hit": 2, "fill_call": 3,
            "large_page_hit": 5,
        }}}}
        self.assertEqual(
            ANALYZER.refill_balance(stats)["helper"]["load"], 0
        )

    def test_qemu_dso_filter(self):
        self.assertTrue(ANALYZER.is_qemu_sample("/tmp/qemu-system-x86_64"))
        self.assertTrue(ANALYZER.is_qemu_sample("qemu-system-aarch64"))
        self.assertFalse(ANALYZER.is_qemu_sample("[kernel.kallsyms]"))
        self.assertFalse(ANALYZER.is_qemu_sample("/usr/lib/libc.so.6"))

    def test_perf_script_does_not_count_host_kernel_tlb_symbol(self):
        samples = """\
100 1 probe_access_internal (/tmp/qemu-system-x86_64)
200 2 tlb_flush_page ([kernel.kallsyms])
300 3 cpu_atomic_fetch_addq_le_mmu (/tmp/qemu-system-x86_64)
400 4 guest-1000 (/tmp/perf-1.map)
"""
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "perf-script.txt"
            path.write_text(samples)
            result = ANALYZER.parse_perf_script(path)
        self.assertEqual(result["sample_period_total"], 1000)
        self.assertEqual(result["sample_records_total"], 4)
        self.assertEqual(result["sample_records_by_category"], {
            "softmmu_core": 2,
            "softmmu_support": 0,
            "jit_guest": 1,
            "other": 1,
        })
        self.assertAlmostEqual(result["softmmu_core"], 40.0)
        self.assertAlmostEqual(result["jit_guest"], 40.0)
        self.assertAlmostEqual(result["other"], 20.0)


if __name__ == "__main__":
    unittest.main()
