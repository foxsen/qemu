#!/usr/bin/env python3
"""Focused regression tests for cloud-run provenance helpers."""

from importlib.machinery import SourceFileLoader
from pathlib import Path
import subprocess
import tempfile
from types import SimpleNamespace
import unittest


HERE = Path(__file__).resolve().parent
RUNNER = SourceFileLoader(
    "run_cloud_profile", str(HERE / "run-cloud-profile.py")
).load_module()
HELPERS = RUNNER.load_helpers(HERE)


class ProvenanceTest(unittest.TestCase):
    def test_micro_runner_refuses_nonempty_result_directory(self):
        with tempfile.TemporaryDirectory() as temp:
            result_dir = Path(temp) / "result"
            HELPERS.prepare_result_dir(result_dir)
            (result_dir / "stale.json").write_text("{}\n")
            with self.assertRaises(FileExistsError):
                HELPERS.prepare_result_dir(result_dir)

    def test_ptw_counter_parser(self):
        parsed = HELPERS.parse_tlb(
            "PTW stage=primary access=load walks=10 full_restarts=2 "
            "levels=1:8,2:10,3:10,4:10,5:0\n"
            "PTW stage=nested access=store walks=4 full_restarts=1 "
            "levels=1:4,2:4,3:4,4:4,5:0\n"
        )
        row = parsed["ptw"]["primary"]["load"]
        self.assertEqual(row["walks"], 10)
        self.assertEqual(row["full_restarts"], 2)
        self.assertEqual(row["levels"]["1"], 8)
        self.assertEqual(parsed["ptw"]["nested"]["store"]["walks"], 4)

    def test_qcow2_backing_chain_and_snapshot_identity(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            base = root / "base.qcow2"
            overlay = root / "overlay.qcow2"
            subprocess.run(
                ["qemu-img", "create", "-q", "-f", "qcow2",
                 str(base), "1M"],
                check=True,
            )
            subprocess.run(
                ["qemu-img", "create", "-q", "-f", "qcow2",
                 "-F", "qcow2", "-b", str(base), str(overlay)],
                check=True,
            )

            mutable = RUNNER.disk_record(overlay, exact_hash=False)
            self.assertEqual(mutable["qemu_img_info"]["returncode"], 0)
            self.assertFalse(mutable["launch_image_hash_is_identity"])
            self.assertNotIn("sha256", mutable["launch_image"])
            self.assertEqual(len(mutable["backing_chain"]), 2)
            self.assertNotIn("sha256", mutable["backing_chain"][0])
            self.assertIn("sha256", mutable["backing_chain"][1])

            snapshot = RUNNER.disk_record(overlay, exact_hash=True)
            self.assertTrue(snapshot["launch_image_hash_is_identity"])
            self.assertIn("sha256", snapshot["launch_image"])
            self.assertIn("sha256", snapshot["backing_chain"][0])

    def test_host_cpu_list_parsing(self):
        record = RUNNER.host_record("2-3")
        self.assertEqual(record["taskset_cpu_list"], "2-3")
        self.assertEqual(record["first_logical_cpu"], 2)

    def test_git_record_identifies_tracked_diff_content(self):
        record = RUNNER.git_record(HERE.parent)
        self.assertEqual(record["tracked_diff_returncode"], 0)
        self.assertEqual(len(record["tracked_diff_sha256"]), 64)
        int(record["tracked_diff_sha256"], 16)

    def test_run_options_include_machine_configuration(self):
        options = RUNNER.run_options_record(SimpleNamespace(
            cpu="2", nice=-20, memory="1G", smp=1, ssh_port=2222,
            perf=True, perf_frequency=99, perf_event="cpu_core/cycles/u",
            perf_callgraph=False,
            perfmap=False, plugin_window=False, snapshot=True,
            large_page_cache="on",
            ptw_cache="probe",
            tlb_entries=4096,
            victim_tlb="off",
            guest_thp="always",
        ))
        self.assertEqual(options["cpu"], "2")
        self.assertEqual(options["nice"], -20)
        self.assertEqual(options["memory"], "1G")
        self.assertEqual(options["smp"], 1)
        self.assertEqual(options["ssh_port"], 2222)
        self.assertEqual(options["perf_frequency"], 99)
        self.assertEqual(options["perf_event"], "cpu_core/cycles/u")
        self.assertEqual(options["large_page_cache"], "on")
        self.assertEqual(options["ptw_cache"], "probe")
        self.assertEqual(options["tlb_entries"], 4096)
        self.assertEqual(options["victim_tlb"], "off")
        self.assertEqual(options["guest_thp"], "always")

    def test_guest_thp_policy_is_part_of_prepare_command(self):
        command = RUNNER.combined_prepare_command("echo ready", "always")
        self.assertIn("transparent_hugepage/enabled", command)
        self.assertIn("always", command)
        self.assertTrue(command.endswith("&& echo ready"))
        self.assertEqual(
            RUNNER.combined_prepare_command("echo ready", "leave"),
            "echo ready",
        )

    def test_large_page_cache_counter_parser_and_delta(self):
        parsed = HELPERS.parse_tlb(
            "Large-page cache mode=on lookup=20 match=15 hit=15 "
            "insert=5 eviction=1 flush=2\n"
        )
        self.assertEqual(parsed["large_page_cache"]["mode"], "on")
        self.assertEqual(parsed["large_page_cache"]["hit"], 15)
        delta = HELPERS.subtract(parsed, {
            "large_page_cache": {
                "mode": "on", "lookup": 2, "match": 1, "hit": 1,
                "insert": 1, "eviction": 0, "flush": 1,
            },
        })
        self.assertEqual(delta["large_page_cache"]["mode"], "on")
        self.assertEqual(delta["large_page_cache"]["lookup"], 18)

    def test_ptw_cache_counter_parser(self):
        parsed = HELPERS.parse_tlb(
            "PTW cache mode=on flush=4\n"
            "PTW cache level=2 lookup=20 match=15 hit=15 "
            "insert=5 eviction=1\n"
        )
        self.assertEqual(parsed["ptw_cache"]["mode"], "on")
        self.assertEqual(parsed["ptw_cache"]["flush"], 4)
        self.assertEqual(parsed["ptw_cache"]["levels"]["2"]["hit"], 15)

    def test_fixed_tlb_config_parser_validation_and_delta(self):
        parsed = HELPERS.parse_tlb(
            "SoftMMU TLB config fixed_entries=4096 victim=off "
            "current_min=4096 current_max=4096\n"
        )
        self.assertEqual(parsed["tlb_config"]["fixed_entries"], 4096)
        self.assertEqual(parsed["tlb_config"]["victim"], "off")
        HELPERS.validate_tlb_config(parsed, 4096, "off")
        delta = HELPERS.subtract(parsed, parsed)
        self.assertEqual(delta["tlb_config"], parsed["tlb_config"])
        with self.assertRaises(ValueError):
            HELPERS.validate_tlb_config(parsed, 2048, "off")
        with self.assertRaises(ValueError):
            HELPERS.validate_tlb_config(parsed, 4096, "on")


if __name__ == "__main__":
    unittest.main()
