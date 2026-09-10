#!/bin/sh
set -eu

study_dir=$(CDPATH= cd -- "$(dirname "$0")" && pwd)
root_dir="$study_dir/initramfs/root"

cc -static -O2 -g -Wall -Wextra \
   -o "$study_dir/guest/pagewalk-bench" \
   "$study_dir/guest/pagewalk-bench.c"

mkdir -p "$root_dir/bin" "$root_dir/dev" "$root_dir/proc" "$root_dir/sys"
cp /usr/bin/busybox "$root_dir/bin/busybox"
cp "$study_dir/guest/pagewalk-bench" "$root_dir/pagewalk-bench"
cp "$study_dir/guest/init" "$root_dir/init"
chmod 0755 "$root_dir/init" "$root_dir/pagewalk-bench"

(cd "$root_dir" && find . -print | cpio -o -H newc) \
    | gzip -9 > "$study_dir/images/tlb-initramfs.img"
echo "$study_dir/images/tlb-initramfs.img"
