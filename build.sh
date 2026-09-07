#!/bin/bash
set -e

echo "Uncompressing vocabulary ..."
[ -f Vocabulary/ORBvoc.txt ] || tar -xf Vocabulary/ORBvoc.txt.tar.gz -C Vocabulary

cabin build --release "$@"
