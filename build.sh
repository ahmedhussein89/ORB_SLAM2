#!/bin/bash
set -e

echo "Uncompressing vocabulary ..."
[ -f Vocabulary/ORBvoc.txt ] || tar -xf Vocabulary/ORBvoc.txt.tar.gz -C Vocabulary

cmake -S . -B build -DCMAKE_BUILD_TYPE=Release "$@"
cmake --build build -j"$(nproc)"
