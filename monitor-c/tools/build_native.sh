#!/bin/sh
set -eu
cd "$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)"
cmake -S . -B build/native -DCMAKE_BUILD_TYPE=Debug
cmake --build build/native -j2
