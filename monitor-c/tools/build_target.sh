#!/bin/sh
set -eu
cd "$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)"
if [ "$#" -ne 1 ] || [ ! -x "$1" ]; then
    echo 'Usage: sh tools/build_target.sh /absolute/path/to/loongarch64-linux-gnu-gcc' >&2
    exit 2
fi
case "$1" in /*) ;; *) echo 'Use an absolute compiler path' >&2; exit 2;; esac
cmake -S . -B build/board99 -DCMAKE_SYSTEM_NAME=Linux \
    -DCMAKE_C_COMPILER="$1" -DCMAKE_EXE_LINKER_FLAGS=-static \
    -DCMAKE_BUILD_TYPE=Release 
cmake --build build/board99 -j2
echo 'Built build/board99/zxjb14a_monitor_c (not executed or installed)'
