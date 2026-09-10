#!/bin/sh
set -eu

study_dir=$(CDPATH= cd -- "$(dirname "$0")" && pwd)
qemu=${QEMU:-$study_dir/../build-tlb-base/qemu-system-x86_64}
mode=${TLB_MODE:-seq}
mib=${TLB_MIB:-128}
passes=${TLB_PASSES:-128}
page_mode=${TLB_PAGE_MODE:-nohuge}
result_dir=${RESULT_DIR:-$study_dir/results}
name=${RUN_NAME:-${mode}-${mib}m-${passes}p-${page_mode}}

mkdir -p "$result_dir"
exec "$qemu" \
    -accel tcg,thread=single \
    -machine pc \
    -cpu max -smp 1 -m 512M \
    -display none -serial stdio -monitor none -no-reboot \
    -kernel "$study_dir/images/debian-bookworm-amd64-linux" \
    -initrd "$study_dir/images/tlb-initramfs.img" \
    -append "console=ttyS0 panic=-1 tlb_mode=$mode tlb_mib=$mib tlb_passes=$passes tlb_page_mode=$page_mode" \
    2>"$result_dir/$name.stderr" | tee "$result_dir/$name.console"
