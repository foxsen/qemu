#!/usr/bin/env python3
"""Focused tests for workload artifact integrity handling."""

import hashlib
from importlib.machinery import SourceFileLoader
from pathlib import Path
import tempfile
import unittest


HERE = Path(__file__).resolve().parent
FETCHER = SourceFileLoader(
    "fetch_workloads", str(HERE / "fetch-workloads.py")
).load_module()


class WorkloadIntegrityTest(unittest.TestCase):
    def test_verify_accepts_expected_digest(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "artifact"
            path.write_bytes(b"verified payload")
            expected = hashlib.sha256(path.read_bytes()).hexdigest()
            self.assertEqual(FETCHER.verify(path, expected), expected)

    def test_verify_rejects_mismatch(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "artifact"
            path.write_bytes(b"unexpected payload")
            with self.assertRaisesRegex(RuntimeError, "digest mismatch"):
                FETCHER.verify(path, "0" * 64)

    def test_fetch_does_not_replace_existing_bad_file(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "artifact"
            path.write_bytes(b"keep me")
            with self.assertRaisesRegex(RuntimeError, "digest mismatch"):
                FETCHER.fetch(path, {
                    "url": "https://invalid.example/artifact",
                    "sha256": "0" * 64,
                })
            self.assertEqual(path.read_bytes(), b"keep me")


if __name__ == "__main__":
    unittest.main()
