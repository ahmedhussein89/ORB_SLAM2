#!/bin/bash
set -e

echo "Uncompressing vocabulary ..."
[ -f Vocabulary/ORBvoc.txt ] || tar -xf Vocabulary/ORBvoc.txt.tar.gz -C Vocabulary

# WSL2 leaves no headroom: saturating all cores takes the WSL service down
# mid-build. Override by exporting CABIN_BUILD_JOBS yourself.
export CABIN_BUILD_JOBS="${CABIN_BUILD_JOBS:-12}"

cabin build --release "$@"
