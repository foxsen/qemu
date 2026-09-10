#!/usr/bin/env python3
"""Focused tests for result acceptance gates."""

from importlib.machinery import SourceFileLoader
from pathlib import Path
import tempfile
import unittest


HERE = Path(__file__).resolve().parent
VALIDATOR = SourceFileLoader(
    "validate_results", str(HERE / "validate-results.py")
).load_module()


class ResultValidationTest(unittest.TestCase):
    def test_sha256_format_is_strict(self):
        self.assertTrue(VALIDATOR.has_sha256({"sha256": "a" * 64}))
        self.assertFalse(VALIDATOR.has_sha256({"sha256": "A" * 64}))
        self.assertFalse(VALIDATOR.has_sha256({"sha256": "not-a-hash"}))

    def test_accepts_consistent_window_result(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            summary = {
                "measurement_kind": "instrumented_window",
                "workload_succeeded": True,
                "guest_mem_accesses": 100,
                "marker_windows": 1,
                "marker_active_at_exit": False,
                "tlb": {
                    "refill_balance_residual": 0,
                    "origin_refill_balance_residuals": {
                        "helper": {"load": 0},
                    },
                },
            }
            self.assertEqual(
                VALIDATOR.result_problems(summary, {}, root), []
            )

    def test_rejects_failed_and_unbalanced_result(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            summary = {
                "measurement_kind": "instrumented_counter",
                "workload_succeeded": False,
                "remote_returncode": 2,
                "tlb": {
                    "refill_balance_residual": 1,
                    "origin_refill_balance_residuals": {},
                },
            }
            problems = VALIDATOR.result_problems(summary, {}, root)
            self.assertTrue(any("failed" in item for item in problems))
            self.assertTrue(any("balance" in item for item in problems))

    def test_origin_totals_must_cover_global_tlb_counters(self):
        with tempfile.TemporaryDirectory() as temp:
            summary = {
                "measurement_kind": "instrumented_counter",
                "tlb": {
                    "l1_miss": 10,
                    "victim_hit": 4,
                    "fill_call": 6,
                    "refill_balance_residual": 0,
                    "origin_refill_balance_residuals": {
                        "helper": {"load": 0},
                    },
                    "origins": {"helper": {"load": {
                        "l1_miss": 9, "victim_hit": 4, "fill_call": 5,
                    }}},
                },
            }
            problems = VALIDATOR.result_problems(summary, {}, Path(temp))
            self.assertIn(
                "origin l1_miss total does not match global counter", problems
            )
            self.assertIn(
                "origin fill_call total does not match global counter", problems
            )

    def test_fill_installs_and_page_size_buckets_are_bounded(self):
        with tempfile.TemporaryDirectory() as temp:
            summary = {
                "measurement_kind": "instrumented_counter",
                "tlb": {
                    "fill_call": 5,
                    "victim_hit": -1,
                    "fill_installs": 6,
                    "page_bits": {"12": 4, "21": 1, "64": -1},
                    "refill_balance_residual": 0,
                    "origin_refill_balance_residuals": {},
                },
            }
            problems = VALIDATOR.result_problems(summary, {}, Path(temp))
            self.assertIn("TLB fill installs exceed fill calls", problems)
            self.assertIn(
                "negative global TLB counter: victim_hit", problems
            )
            self.assertIn(
                "TLB fill page-size buckets do not sum to installs", problems
            )
            self.assertIn("invalid TLB page-size bucket: 64", problems)
            self.assertIn(
                "negative TLB page-size bucket count: 64", problems
            )

    def test_ptw_cache_counters_are_bounded(self):
        with tempfile.TemporaryDirectory() as temp:
            summary = {
                "measurement_kind": "instrumented_counter",
                "tlb": {
                    "refill_balance_residual": 0,
                    "origin_refill_balance_residuals": {},
                },
                "ptw_cache": {
                    "mode": "probe",
                    "flush": 0,
                    "levels": {"2": {
                        "lookup": 3, "match": 4, "hit": 1,
                        "insert": 1, "eviction": 2,
                    }},
                },
            }
            problems = VALIDATOR.result_problems(
                summary, {}, Path(temp)
            )
            self.assertIn(
                "PTW cache matches exceed lookups at level 2", problems
            )
            self.assertIn(
                "PTW cache evictions exceed inserts at level 2", problems
            )
            self.assertIn("probe PTW cache has hits at level 2", problems)

    def test_provenance_and_ptw_can_be_required(self):
        with tempfile.TemporaryDirectory() as temp:
            problems = VALIDATOR.result_problems(
                {"measurement_kind": "instrumented_counter", "tlb": {
                    "refill_balance_residual": 0,
                    "origin_refill_balance_residuals": {},
                }},
                {"qemu_version": "QEMU 8.2.9"}, Path(temp),
                require_provenance=True, require_ptw=True,
                qemu_version="8.2.9",
            )
            self.assertTrue(any("provenance" in item for item in problems))
            self.assertIn("result has no PTW walks", problems)

    def test_nested_ptw_can_be_required(self):
        with tempfile.TemporaryDirectory() as temp:
            summary = {
                "measurement_kind": "instrumented_counter",
                "tlb": {
                    "fill_call": 1,
                    "refill_balance_residual": 0,
                    "origin_refill_balance_residuals": {},
                },
                "ptw": {
                    "walks": 1,
                    "full_restarts": 0,
                    "total_level_visits": 1,
                    "by_stage": {"primary": {
                        "walks": 1,
                        "full_restarts": 0,
                        "total_level_visits": 1,
                        "by_access": {"load": {
                            "walks": 1, "full_restarts": 0,
                            "levels": {"1": 1},
                        }},
                    }},
                },
                "tlb_tables_after": [{
                    "cpu": 0, "mmu": 0, "entries": 64,
                    "used": 1, "window_max": 1,
                }],
                "tlb_capacity_after": {
                    "largest_table_entries": 64,
                    "reported_table_entries": 64,
                    "largest_current_window_occupancy": 1,
                },
            }
            problems = VALIDATOR.result_problems(
                summary, {}, Path(temp), require_nested_ptw=True
            )
            self.assertIn("result has no nested-stage PTW walks", problems)

    def test_ptw_requires_primary_stage_and_level_visits(self):
        with tempfile.TemporaryDirectory() as temp:
            summary = {
                "measurement_kind": "instrumented_counter",
                "tlb": {
                    "refill_balance_residual": 0,
                    "origin_refill_balance_residuals": {},
                },
                "ptw": {
                    "walks": 10,
                    "total_level_visits": 5,
                    "by_stage": {"nested": {"walks": 10}},
                },
                "tlb_tables_after": [{
                    "cpu": -1, "mmu": 16, "entries": 100,
                    "used": 101, "window_max": 102,
                }, {
                    "cpu": -1, "mmu": 16, "entries": 128,
                    "used": 0, "window_max": 0,
                }],
            }
            problems = VALIDATOR.result_problems(
                summary, {}, Path(temp), require_ptw=True
            )
            self.assertIn("result has no primary-stage PTW walks", problems)
            self.assertIn("PTW level visits are fewer than walks", problems)
            self.assertIn(
                "TLB table capacity is not a valid power of two", problems
            )
            self.assertIn(
                "TLB used entries exceed table capacity", problems
            )
            self.assertIn(
                "TLB window maximum exceeds table capacity", problems
            )
            self.assertIn("TLB snapshot has invalid MMU index", problems)
            self.assertIn("TLB snapshot has invalid CPU index", problems)
            self.assertIn(
                "TLB snapshot has duplicate CPU/MMU table", problems
            )
            self.assertIn(
                "TLB capacity summary has inconsistent largest_table_entries",
                problems,
            )

    def test_accepts_consistent_ptw_stage_totals(self):
        with tempfile.TemporaryDirectory() as temp:
            summary = {
                "measurement_kind": "instrumented_counter",
                "tlb": {
                    "fill_call": 10,
                    "refill_balance_residual": 0,
                    "origin_refill_balance_residuals": {},
                },
                "ptw": {
                    "walks": 10,
                    "full_restarts": 1,
                    "total_level_visits": 31,
                    "by_stage": {
                        "primary": {
                            "walks": 8,
                            "full_restarts": 1,
                            "total_level_visits": 25,
                            "by_access": {"load": {
                                "walks": 8,
                                "full_restarts": 1,
                                "levels": {"1": 8, "2": 8, "3": 8,
                                           "4": 1},
                            }},
                        },
                        "nested": {
                            "walks": 2,
                            "full_restarts": 0,
                            "total_level_visits": 6,
                            "by_access": {"store": {
                                "walks": 2,
                                "full_restarts": 0,
                                "levels": {"1": 2, "2": 2, "3": 2},
                            }},
                        },
                    },
                },
                "tlb_tables_after": [{
                    "cpu": 0, "mmu": 2, "entries": 1024,
                    "used": 10, "window_max": 20,
                }],
                "tlb_capacity_after": {
                    "largest_table_entries": 1024,
                    "reported_table_entries": 1024,
                    "largest_current_window_occupancy": 20,
                },
            }
            self.assertEqual(VALIDATOR.result_problems(
                summary, {}, Path(temp), require_ptw=True
            ), [])

    def test_ptw_walks_cannot_exceed_target_fills(self):
        with tempfile.TemporaryDirectory() as temp:
            summary = {
                "measurement_kind": "instrumented_counter",
                "tlb": {
                    "fill_call": 5,
                    "refill_balance_residual": 0,
                    "origin_refill_balance_residuals": {},
                },
                "ptw": {
                    "walks": 10,
                    "total_level_visits": 40,
                    "by_stage": {"primary": {"walks": 10}},
                },
            }
            problems = VALIDATOR.result_problems(
                summary, {}, Path(temp), require_ptw=True
            )
            self.assertIn(
                "PTW walks exceed target TLB fill calls", problems
            )

    def test_ptw_level_visits_obey_x86_five_level_bound(self):
        with tempfile.TemporaryDirectory() as temp:
            summary = {
                "measurement_kind": "instrumented_counter",
                "tlb": {
                    "fill_call": 2,
                    "refill_balance_residual": 0,
                    "origin_refill_balance_residuals": {},
                },
                "ptw": {
                    "walks": 2,
                    "full_restarts": 0,
                    "total_level_visits": 11,
                    "by_stage": {"primary": {"walks": 2}},
                },
            }
            problems = VALIDATOR.result_problems(
                summary, {}, Path(temp), require_ptw=True
            )
            self.assertIn(
                "PTW level visits exceed x86 walk bound", problems
            )

    def test_each_ptw_restart_visits_at_least_one_level(self):
        with tempfile.TemporaryDirectory() as temp:
            summary = {
                "measurement_kind": "instrumented_counter",
                "tlb": {
                    "fill_call": 2,
                    "refill_balance_residual": 0,
                    "origin_refill_balance_residuals": {},
                },
                "ptw": {
                    "walks": 2,
                    "full_restarts": 3,
                    "total_level_visits": 4,
                    "by_stage": {"primary": {"walks": 2}},
                },
            }
            problems = VALIDATOR.result_problems(
                summary, {}, Path(temp), require_ptw=True
            )
            self.assertIn(
                "PTW level visits are fewer than walk attempts", problems
            )

    def test_rejects_inconsistent_softmmu_time_breakdown(self):
        with tempfile.TemporaryDirectory() as temp:
            summary = {
                "measurement_kind": "perf_timing",
                "perf": {
                    "sample_records_total": 10,
                    "sample_period_total": 100,
                    "softmmu_named_share": 12.0,
                    "softmmu_core": 10.0,
                    "softmmu_support": 1.0,
                },
            }
            problems = VALIDATOR.result_problems(
                summary, {}, Path(temp)
            )
            self.assertIn(
                "SoftMMU core/support shares do not sum to total", problems
            )

    def test_rejects_invalid_perf_percentages(self):
        with tempfile.TemporaryDirectory() as temp:
            summary = {
                "measurement_kind": "perf_timing",
                "perf": {
                    "sample_records_total": 10,
                    "sample_period_total": 100,
                    "softmmu_named_share": 12.0,
                    "softmmu_core": 10.0,
                    "softmmu_support": 2.0,
                    "jit_guest": 60.0,
                    "other": -1.0,
                    "classified_overhead": 71.0,
                },
            }
            problems = VALIDATOR.result_problems(
                summary, {}, Path(temp)
            )
            self.assertIn("invalid perf percentage: other", problems)
            self.assertIn(
                "perf category shares do not sum to 100 percent", problems
            )

    def test_rejects_nonfinite_and_mismatched_perf_percentages(self):
        with tempfile.TemporaryDirectory() as temp:
            summary = {
                "measurement_kind": "perf_timing",
                "perf": {
                    "sample_records_total": 10,
                    "sample_period_total": 100,
                    "softmmu_named_share": 12.0,
                    "softmmu_core": 10.0,
                    "softmmu_support": 2.0,
                    "jit_guest": float("nan"),
                    "other": 88.0,
                    "classified_overhead": 99.0,
                },
            }
            problems = VALIDATOR.result_problems(
                summary, {}, Path(temp)
            )
            self.assertIn("invalid perf percentage: jit_guest", problems)

            summary["perf"]["jit_guest"] = 0.0
            problems = VALIDATOR.result_problems(
                summary, {}, Path(temp)
            )
            self.assertIn(
                "perf category shares do not match classified total", problems
            )

    def test_provenance_requires_structured_machine_options(self):
        with tempfile.TemporaryDirectory() as temp:
            provenance = {
                field: {} for field in (
                    "qemu_binary", "guest_disk", "seed", "git", "host",
                    "guest", "run_options",
                )
            }
            provenance["copied_inputs"] = []
            problems = VALIDATOR.result_problems(
                {"measurement_kind": "instrumented_counter", "tlb": {
                    "refill_balance_residual": 0,
                    "origin_refill_balance_residuals": {},
                }},
                {
                    "provenance": provenance,
                    "measurement_started_utc": "2026-09-09T00:00:00+00:00",
                    "measurement_ended_utc": "2026-09-09T00:00:01+00:00",
                    "wall_seconds": 1.0,
                    "qemu_cpu_seconds": 0.9,
                }, Path(temp),
                require_provenance=True,
            )
            self.assertIn(
                "provenance run_options is missing memory", problems
            )

    def test_accepts_complete_hashed_provenance(self):
        with tempfile.TemporaryDirectory() as temp:
            file_record = {"sha256": "a" * 64}
            provenance = {
                "qemu_binary": file_record,
                "harness": {
                    "runner": file_record,
                    "qmp_helpers": file_record,
                },
                "guest_disk": {
                    "launch_image": file_record,
                    "launch_image_hash_is_identity": True,
                    "qemu_img_info": {"returncode": 0},
                    "backing_chain": [file_record],
                },
                "seed": file_record,
                "copied_inputs": [file_record],
                "git": {
                    "head": "c" * 40,
                    "status_returncode": 0,
                    "tracked_diff_returncode": 0,
                    "tracked_diff_sha256": "b" * 64,
                },
                "host": {"taskset_cpu_list": "2"},
                "guest": {
                    field: {"returncode": 0}
                    for field in ("uname", "os_release", "packages")
                },
                "run_options": {
                    "cpu": "2", "memory": "1G", "smp": 1,
                    "ssh_port": 2222, "snapshot": True,
                },
            }
            problems = VALIDATOR.result_problems(
                {"measurement_kind": "instrumented_counter", "tlb": {
                    "refill_balance_residual": 0,
                    "origin_refill_balance_residuals": {},
                }},
                {
                    "provenance": provenance,
                    "measurement_started_utc": "2026-09-09T00:00:00+00:00",
                    "measurement_ended_utc": "2026-09-09T00:00:01+00:00",
                    "wall_seconds": 1.0,
                    "qemu_cpu_seconds": 0.9,
                }, Path(temp),
                require_provenance=True,
            )
            self.assertEqual(problems, [])

    def test_accepts_complete_linux_build_evidence(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            digest = "a" * 64
            (root / "workload.log").write_text(
                "Kernel: arch/x86/boot/bzImage is ready  (#1)\n"
                f"{digest}  vmlinux\n"
                f"{digest}  arch/x86/boot/bzImage\n"
                "TLB-LINUX-BUILD-PASS\n"
            )
            summary = {"measurement_kind": "instrumented_counter", "tlb": {
                "refill_balance_residual": 0,
                "origin_refill_balance_residuals": {},
            }}
            raw = {
                "guest_command": "make -j1 vmlinux bzImage",
            }
            self.assertEqual(VALIDATOR.result_problems(
                summary, raw, root, require_linux_build=True
            ), [])
            raw["guest_command"] = (
                "make -j1 vmlinux && make -j1 bzImage"
            )
            self.assertEqual(VALIDATOR.result_problems(
                summary, raw, root, require_linux_build=True
            ), [])

    def test_rejects_incomplete_linux_build_evidence(self):
        with tempfile.TemporaryDirectory() as temp:
            problems = VALIDATOR.result_problems(
                {"measurement_kind": "perf_timing", "perf": {
                    "sample_records_total": 1, "sample_period_total": 1,
                }},
                {"guest_command": "make -j8"}, Path(temp),
                require_linux_build=True,
            )
            self.assertTrue(any("make -j1" in item for item in problems))
            self.assertTrue(any("pass marker" in item for item in problems))


if __name__ == "__main__":
    unittest.main()
