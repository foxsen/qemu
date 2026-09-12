#!/usr/bin/env python3
"""Run GraphBIG CPU workloads under the cloud harness."""

import argparse
from pathlib import Path, PurePosixPath
import re
import shlex
import subprocess
import sys


KERNELS = {
    "pr": {
        "directory": "bench_pageRank",
        "binary": "pagerank",
        "arguments": "--quad 0.001 --damp 0.85 --maxiter 100",
        "completion": "Page Rank finish",
    },
    "gc": {
        "directory": "bench_graphColoring",
        "binary": "graphcoloring",
        "arguments": "",
        "completion": "Graph Coloring Results:",
    },
    "sssp": {
        "directory": "bench_shortestPath",
        "binary": "sssp",
        "arguments": "--root 31",
        "completion": "Shortest Path: source-",
    },
    "tc": {
        "directory": "bench_triangleCount",
        "binary": "tc",
        "arguments": "",
        "completion": "total triangle count:",
    },
    "bfs": {
        "directory": "bench_BFS",
        "binary": "bfs",
        "arguments": "--root 31",
        "completion": "BFS finish",
    },
    "cc": {
        "directory": "bench_connectedComp",
        "binary": "connectedcomponent",
        "arguments": "",
        "completion": "total component num",
    },
    "bc": {
        "directory": "bench_betweennessCentr",
        "binary": "bc",
        "arguments": "--undirected",
        "completion": "== finish",
    },
}


def safe_subdir(value):
    path = PurePosixPath(value)
    if path.is_absolute() or ".." in path.parts:
        raise argparse.ArgumentTypeError(
            "dataset subdirectory must be a relative path without '..'"
        )
    return str(path)


def graphbig_commands(kernel, source_name, dataset_name=None,
                      dataset_subdir=".", verify_small=True):
    config = KERNELS[kernel]
    bench = f"graphbig-run/benchmark/{config['directory']}"
    prepare = [
        "cd ~/tlb-workloads",
        "rm -rf graphbig-run graphbig-data",
        "mkdir graphbig-run",
        (f"tar -xzf {shlex.quote(source_name)} -C graphbig-run "
         "--strip-components=1"),
    ]
    if dataset_name is None:
        dataset = "../../dataset/small"
        if verify_small:
            prepare.append(
                f"make -C {bench} PFM=0 OMP=0 verify"
            )
            prepare.append(f"make -C {bench} clean")
    else:
        prepare.extend([
            "mkdir graphbig-data",
            (f"tar -xaf {shlex.quote(dataset_name)} "
             "-C graphbig-data"),
        ])
        dataset = f"~/tlb-workloads/graphbig-data/{dataset_subdir}"
    prepare.append(f"make -C {bench} PFM=0 OMP=0 -j1 all")

    arguments = config["arguments"]
    command = (
        f"cd ~/tlb-workloads/{bench} && "
        f"./{config['binary']} --threadnum 1 --dataset {dataset} "
        f"{arguments} > graphbig.out 2> graphbig.err && "
        "grep -Eq '^== [0-9]+ vertices  +[0-9]+ edges$' graphbig.out && "
        f"grep -Fq {shlex.quote(config['completion'])} graphbig.out"
    )
    return " && ".join(prepare), command


def main():
    here = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(
        description="run pinned GraphBIG v3.2 CPU workloads sequentially"
    )
    parser.add_argument(
        "--mode", choices=("perf", "profile", "window"), required=True,
        help="uninstrumented timing, TLB counters, or exact memory denominator",
    )
    parser.add_argument(
        "--kernel", action="append", choices=KERNELS,
        help="kernel to run; repeat the option, or omit it to run all seven",
    )
    parser.add_argument(
        "--source", type=Path,
        default=here / "workloads/graphbig-v3.2-fc1ef159.tar.gz",
    )
    parser.add_argument(
        "--dataset-archive", type=Path,
        help="optional tar archive containing vertex.csv and edge.csv",
    )
    parser.add_argument(
        "--dataset-subdir", type=safe_subdir, default=".",
        help=("dataset directory inside --dataset-archive "
              "(default: archive root)"),
    )
    parser.add_argument(
        "--dataset-tag", default="small",
        help="safe label recorded in result names",
    )
    parser.add_argument("--memory", default="2G")
    parser.add_argument("--cpu", default="2")
    parser.add_argument("--nice", type=int, default=0)
    parser.add_argument("--perf-frequency", type=int, default=99)
    parser.add_argument(
        "--large-page-cache",
        choices=("off", "on", "probe", "adaptive"), default="off",
    )
    parser.add_argument(
        "--ptw-cache", choices=("off", "on", "probe", "adaptive"),
        default="off",
    )
    parser.add_argument("--tlb-entries", type=int, default=0)
    parser.add_argument(
        "--victim-tlb", choices=("on", "off"), default="on",
    )
    parser.add_argument(
        "--guest-thp", choices=("leave", "always", "madvise", "never"),
        default="leave",
    )
    parser.add_argument(
        "--verify-small", action=argparse.BooleanOptionalAction, default=True,
        help="run GraphBIG's reference-output gate before a bundled-small run",
    )
    parser.add_argument(
        "--name-suffix", default="",
        help="suffix for repetitions, for example r01 or baseline",
    )
    parser.add_argument("--snapshot", action=argparse.BooleanOptionalAction,
                        default=True)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    safe_label = r"[A-Za-z0-9][A-Za-z0-9._-]*"
    if not re.fullmatch(safe_label, args.dataset_tag):
        parser.error("--dataset-tag contains unsupported characters")
    if args.name_suffix and not re.fullmatch(safe_label, args.name_suffix):
        parser.error("--name-suffix contains unsupported characters")
    if args.tlb_entries < 0 or (
            args.tlb_entries and
            args.tlb_entries & (args.tlb_entries - 1)):
        parser.error("--tlb-entries must be zero or a power of two")
    if args.dataset_archive is None and args.dataset_tag != "small":
        parser.error("a non-small --dataset-tag requires --dataset-archive")

    source = args.source.resolve()
    if not source.is_file():
        parser.error(
            f"source archive does not exist: {source}; "
            "run fetch-workloads.py graphbig"
        )
    dataset = None
    if args.dataset_archive is not None:
        dataset = args.dataset_archive.resolve()
        if not dataset.is_file():
            parser.error(f"dataset archive does not exist: {dataset}")

    qemu = (here.parent / ("build-tlb-base" if args.mode == "perf"
                           else "build-tlb-profile") /
            "qemu-system-x86_64").resolve()
    if not qemu.is_file():
        parser.error(f"QEMU binary does not exist: {qemu}")

    for kernel in args.kernel or list(KERNELS):
        prepare, guest_command = graphbig_commands(
            kernel, source.name,
            dataset.name if dataset is not None else None,
            args.dataset_subdir, args.verify_small,
        )
        suffix = f"-{args.name_suffix}" if args.name_suffix else ""
        command = [
            sys.executable, str(here / "run-cloud-profile.py"),
            "--qemu", str(qemu),
            "--name", (f"graphbig-{kernel}-{args.dataset_tag}-"
                       f"{args.mode}{suffix}"),
            "--cpu", args.cpu, "--nice", str(args.nice),
            "--memory", args.memory,
            "--copy-to-workloads", str(source),
            "--prepare-command", prepare,
            "--command", guest_command,
            "--large-page-cache", args.large_page_cache,
            "--ptw-cache", args.ptw_cache,
            "--tlb-entries", str(args.tlb_entries),
            "--victim-tlb", args.victim_tlb,
            "--guest-thp", args.guest_thp,
        ]
        if dataset is not None:
            command.extend(["--copy-to-workloads", str(dataset)])
        if args.snapshot:
            command.append("--snapshot")
        if args.mode == "perf":
            command.extend([
                "--perf", "--perf-frequency", str(args.perf_frequency),
                "--no-perfmap",
            ])
        elif args.mode == "window":
            command.append("--plugin-window")
        print(shlex.join(command), flush=True)
        if not args.dry_run:
            subprocess.run(command, check=True)


if __name__ == "__main__":
    main()
