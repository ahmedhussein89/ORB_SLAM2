#!/bin/bash
set -e

echo "Uncompressing vocabulary ..."
[ -f Vocabulary/ORBvoc.txt ] || tar -xf Vocabulary/ORBvoc.txt.tar.gz -C Vocabulary

for d in Thirdparty/DBoW2 Thirdparty/g2o .; do
    echo "Configuring and building $d ..."
    cmake -S "$d" -B "$d/build" -DCMAKE_BUILD_TYPE=Release
    cmake --build "$d/build" -j"$(nproc)"
done
