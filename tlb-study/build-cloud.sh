#!/bin/sh
set -eu

study_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
base=$study_dir/images/debian-12-genericcloud-amd64.qcow2
overlay=$study_dir/images/debian-12-tlb-overlay.qcow2
seed=$study_dir/images/tlb-seed.iso

test -f "$base"
if test ! -f "$overlay"; then
    qemu-img create -f qcow2 -F qcow2 -b "$base" "$overlay" 8G
fi
xorriso -as mkisofs -quiet -V cidata -J -r -o "$seed" \
    "$study_dir/cloud/user-data" "$study_dir/cloud/meta-data"
printf '%s\n' "$overlay" "$seed"
