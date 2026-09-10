#!/bin/sh
set -eu

study_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
qemu=${QEMU:-$study_dir/../build-tlb-base/qemu-system-x86_64}
ssh_port=${SSH_PORT:-2222}

exec "$qemu" \
    -accel tcg,thread=single -machine pc -cpu max -smp 1 -m 1G \
    -no-user-config -display none -serial mon:stdio \
    -drive "if=virtio,format=qcow2,file=$study_dir/images/debian-12-tlb-overlay.qcow2" \
    -drive "if=virtio,format=raw,readonly=on,file=$study_dir/images/tlb-seed.iso" \
    -netdev "user,id=net0,hostfwd=tcp:127.0.0.1:$ssh_port-:22" \
    -device virtio-net-pci,netdev=net0
