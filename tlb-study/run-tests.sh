#!/bin/sh
set -eu

study_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)

python3 "$study_dir/test-analyze-results.py"
python3 "$study_dir/test-fetch-workloads.py"
python3 "$study_dir/test-run-cloud-profile.py"
python3 "$study_dir/test-summarize-workloads.py"
python3 "$study_dir/test-validate-results.py"
python3 -m py_compile \
    "$study_dir/analyze-results.py" \
    "$study_dir/fetch-workloads.py" \
    "$study_dir/prepare-spec2006.py" \
    "$study_dir/run-cloud-profile.py" \
    "$study_dir/run-gapbs.py" \
    "$study_dir/run-linux-build.py" \
    "$study_dir/run-profile.py" \
    "$study_dir/run-optimization-suite.py" \
    "$study_dir/run-spec2006.py" \
    "$study_dir/summarize-optimizations.py" \
    "$study_dir/summarize-workloads.py" \
    "$study_dir/validate-results.py"
