#!/usr/bin/env python3
"""Focused tests for the GraphBIG wrapper."""

import argparse
from importlib.machinery import SourceFileLoader
from pathlib import Path
import unittest


HERE = Path(__file__).resolve().parent
RUNNER = SourceFileLoader(
    "run_graphbig", str(HERE / "run-graphbig.py")
).load_module()


class GraphbigRunnerTest(unittest.TestCase):
    def test_small_run_verifies_then_rebuilds(self):
        prepare, command = RUNNER.graphbig_commands(
            "bfs", "graphbig-v3.2-fc1ef159.tar.gz"
        )
        self.assertIn("PFM=0 OMP=0 verify", prepare)
        self.assertIn("PFM=0 OMP=0 -j1 all", prepare)
        self.assertIn("../../dataset/small", command)
        self.assertIn("grep -Fq 'BFS finish'", command)

    def test_external_dataset_is_not_verified_as_small(self):
        prepare, command = RUNNER.graphbig_commands(
            "pr", "source.tar.gz", "dataset.tar.xz", "twitter"
        )
        self.assertNotIn(" verify", prepare)
        self.assertIn("tar -xaf dataset.tar.xz", prepare)
        self.assertIn("graphbig-data/twitter", command)

    def test_dataset_subdir_rejects_parent_traversal(self):
        with self.assertRaises(argparse.ArgumentTypeError):
            RUNNER.safe_subdir("../outside")


if __name__ == "__main__":
    unittest.main()
