#!/usr/bin/env python3
"""Package selected, locally licensed SPEC CPU2006 train workloads.

The generated archive is ignored by Git.  This script never changes the SPEC
installation and intentionally contains no benchmark source or input data.
"""

import argparse
import hashlib
import json
from pathlib import Path
import shutil
import tarfile
import tempfile


BENCHMARKS = {
    "429.mcf": {
        "binary": "benchspec/CPU2006/429.mcf/exe/mcf_base.x64.Ofast.ld64",
        "inputs": [
            "benchspec/CPU2006/429.mcf/data/train/input/inp.in",
        ],
        "command": ["./mcf", "inp.in"],
    },
    "471.omnetpp": {
        "binary": (
            "benchspec/CPU2006/471.omnetpp/exe/"
            "omnetpp_base.x64.Ofast.ld64"
        ),
        "inputs": [
            "benchspec/CPU2006/471.omnetpp/data/train/input/omnetpp.ini",
        ],
        "command": ["./omnetpp", "omnetpp.ini"],
    },
    "483.xalancbmk": {
        "binary": (
            "benchspec/CPU2006/483.xalancbmk/exe/"
            "Xalan_base.x64.Ofast.ld64"
        ),
        "inputs": [
            "benchspec/CPU2006/483.xalancbmk/data/train/input/allbooks.xml",
            "benchspec/CPU2006/483.xalancbmk/data/train/input/xalanc.xsl",
        ],
        "command": ["./Xalan", "-v", "allbooks.xml", "xalanc.xsl"],
    },
}


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def add_reproducible_tree(archive, root):
    for path in sorted(root.rglob("*")):
        relative = path.relative_to(root.parent)
        info = archive.gettarinfo(str(path), arcname=str(relative))
        info.uid = 0
        info.gid = 0
        info.uname = "root"
        info.gname = "root"
        info.mtime = 0
        if path.is_dir():
            info.mode = 0o755
        elif info.mode & 0o111:
            info.mode = 0o755
        else:
            info.mode = 0o644
        if path.is_file():
            with path.open("rb") as stream:
                archive.addfile(info, stream)
        else:
            archive.addfile(info)


def main():
    here = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(
        description="package selected SPEC CPU2006 train workloads"
    )
    parser.add_argument(
        "--spec-root", type=Path, required=True,
        help="root of a licensed SPEC CPU2006 installation",
    )
    parser.add_argument(
        "--output", type=Path,
        default=here / "workloads/spec2006-train-x86_64.tar.xz",
    )
    args = parser.parse_args()
    spec_root = args.spec_root.resolve()
    output = args.output.resolve()

    missing = []
    for benchmark in BENCHMARKS.values():
        for relative in [benchmark["binary"], *benchmark["inputs"]]:
            if not (spec_root / relative).is_file():
                missing.append(relative)
    if missing:
        parser.error("missing SPEC files:\n  " + "\n  ".join(missing))

    output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="qemu-spec2006-") as temp:
        package = Path(temp) / "spec2006-train"
        files = []
        runs = []
        for name, benchmark in BENCHMARKS.items():
            destination = package / name
            destination.mkdir(parents=True)
            sources = [benchmark["binary"], *benchmark["inputs"]]
            for index, relative in enumerate(sources):
                source = spec_root / relative
                target_name = Path(relative).name
                if index == 0:
                    target_name = benchmark["command"][0].removeprefix("./")
                target = destination / target_name
                shutil.copyfile(source, target)
                target.chmod(0o755 if index == 0 else 0o644)
                files.append({
                    "path": str(target.relative_to(package)),
                    "sha256": sha256(target),
                    "size": target.stat().st_size,
                })
            runs.append({
                "benchmark": name,
                "cwd": name,
                "argv": benchmark["command"],
                "input": "train",
            })

        manifest = {
            "format": 1,
            "suite": "SPEC CPU2006",
            "notice": "Locally licensed data; do not redistribute.",
            "files": sorted(files, key=lambda item: item["path"]),
            "runs": runs,
        }
        (package / "manifest.json").write_text(
            json.dumps(manifest, indent=2, sort_keys=True) + "\n"
        )
        with tarfile.open(output, "w:xz") as archive:
            add_reproducible_tree(archive, package)

    print(f"created {output}")
    print(f"sha256 {sha256(output)}")


if __name__ == "__main__":
    main()
