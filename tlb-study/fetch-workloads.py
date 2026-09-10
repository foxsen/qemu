#!/usr/bin/env python3
"""Fetch public study inputs and verify their pinned SHA-256 digests."""

import argparse
import hashlib
import os
from pathlib import Path
import shutil
import tempfile
import urllib.request


WORKLOADS = {
    "linux": {
        "name": "linux-3.12.9.tar.xz",
        "url": "https://cdn.kernel.org/pub/linux/kernel/v3.x/"
               "linux-3.12.9.tar.xz",
        "sha256": "6a3e9f1abbaeaad34cddf0ddd69d60877765003faccf151b99d33e073566f5cb",
    },
    "dacapo": {
        "name": "dacapo-9.12-bach.jar",
        "url": "https://sourceforge.net/projects/dacapobench/files/archive/"
               "9.12-bach/dacapo-9.12-bach.jar/download",
        "sha256": "33ea1a464480d486e30b172ff07787880152b8de748eb080058494d27e0fd0a9",
    },
    "gapbs": {
        "name": "gapbs-v1.5-b5e3e19c.tar.gz",
        "url": "https://github.com/sbeamer/gapbs/archive/"
               "b5e3e19c2845f22fb338f4a4bc4b1ccee861d026.tar.gz",
        "sha256": "b494c44636b0cbcb683d14a7d2f447f12442b5a6eb9cb1c527a1ea8c8cd7762e",
    },
}


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def verify(path, expected):
    if not path.is_file():
        raise FileNotFoundError(path)
    actual = sha256(path)
    if actual != expected:
        raise RuntimeError(
            f"digest mismatch for {path}: expected {expected}, got {actual}"
        )
    return actual


def fetch(destination, record):
    if destination.exists():
        verify(destination, record["sha256"])
        print(f"verified {destination}")
        return
    destination.parent.mkdir(parents=True, exist_ok=True)
    temp_name = None
    try:
        with tempfile.NamedTemporaryFile(
                dir=destination.parent, prefix=f".{destination.name}.",
                delete=False) as output:
            temp_name = output.name
            request = urllib.request.Request(
                record["url"], headers={"User-Agent": "qemu-tlb-study/1"}
            )
            with urllib.request.urlopen(request) as response:
                shutil.copyfileobj(response, output)
        temp_path = Path(temp_name)
        verify(temp_path, record["sha256"])
        os.replace(temp_path, destination)
        temp_name = None
        print(f"downloaded {destination}")
    finally:
        if temp_name:
            Path(temp_name).unlink(missing_ok=True)


def main():
    here = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "workload", nargs="*", choices=WORKLOADS,
        help="items to fetch; omit to process all public workloads",
    )
    parser.add_argument("--check", action="store_true",
                        help="verify existing files without downloading")
    args = parser.parse_args()

    for key in args.workload or WORKLOADS:
        record = WORKLOADS[key]
        destination = here / "workloads" / record["name"]
        if args.check:
            verify(destination, record["sha256"])
            print(f"verified {destination}")
        else:
            fetch(destination, record)


if __name__ == "__main__":
    main()
