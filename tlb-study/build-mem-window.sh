#!/bin/sh
set -eu

here=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
repo=$(dirname "$here")
out="$here/build"
mkdir -p "$out"

cc -O2 -fPIC -shared -Wall -Wextra \
    -I"$repo/include/qemu" $(pkg-config --cflags glib-2.0) \
    -o "$out/mem-window.so" "$here/plugins/mem-window.c" \
    $(pkg-config --libs glib-2.0)
cc -O2 -fno-pie -no-pie -Wall -Wextra \
    -o "$out/tlb-marker" "$here/guest/tlb-marker.c"

start=$(nm -n "$out/tlb-marker" | awk '$3 == "tlb_window_start" { print "0x" $1 }')
stop=$(nm -n "$out/tlb-marker" | awk '$3 == "tlb_window_stop" { print "0x" $1 }')
test -n "$start" && test -n "$stop"
printf '%s\n' "plugin=$out/mem-window.so,start=$start,stop=$stop"
