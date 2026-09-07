#!/bin/bash
# Generate synthetic sequences, run all three sensor modes, check the recovered motion.
#
# Not a regression test: there is no baseline to compare against and a planar scene
# exercises no loop closure or relocalisation. It answers "does this build still run
# and produce geometrically sane output", which is what catches a broken port.
#
#   tools/smoke_test.sh [work_dir]
set -u

R="$(cd "$(dirname "$0")/.." && pwd)"
WORK="${1:-${TMPDIR:-/tmp}/orbslam2_smoke}"
VOC="$R/Vocabulary/ORBvoc.txt"
fail=0

[ -f "$VOC" ] || { echo "missing $VOC (run build.sh to untar it)"; exit 1; }

# check <name> <actual> <expected> <tolerance-fraction>
check() {
    awk -v n="$1" -v a="$2" -v e="$3" -v t="$4" 'BEGIN{
        d = (a-e)/e; if (d<0) d=-d
        printf "  %-22s %8.4f  expected %8.4f  (%+.1f%%)  %s\n", n, a, e, d*100, (d<=t?"PASS":"FAIL")
        exit(d<=t?0:1)
    }'
}

run() {                      # run <mode> <settings> <extra-args...>
    local mode=$1 settings=$2; shift 2
    local seq="$WORK/$mode" out="$WORK/${mode}_out"
    rm -rf "$seq" "$out"; mkdir -p "$out"
    echo "== $mode =="
    python3 "$R/tools/make_synth_seq.py" "$mode" "$seq" | sed 's/^/  /'
    ( cd "$out" && "$@" "$VOC" "$settings" "$seq" ${EXTRA:-} ) > "$out/log" 2>&1
    local rc=$?
    if [ $rc -ne 0 ]; then echo "  binary exited $rc -- see $out/log"; fail=1; return; fi
    grep -qiE "track lost|reseting" "$out/log" && { echo "  tracking lost"; fail=1; }
}

echo "work dir: $WORK"

# Monocular: no metric scale, so check the direction ratio instead.
run mono "$R/Examples/Monocular/TUM1.yaml" "$R/Examples/Monocular/mono_tum"
ratio=$(tail -1 "$WORK/mono_out/KeyFrameTrajectory.txt" | awk '{print $2/$3}')
check "mono x:y ratio" "$ratio" 2.0 0.15 || fail=1

# Stereo: KITTI trajectory is a row-major 3x4 matrix, so tx is field 4.
run stereo "$R/Examples/Stereo/KITTI00-02.yaml" "$R/Examples/Stereo/stereo_kitti"
tx=$(tail -1 "$WORK/stereo_out/CameraTrajectory.txt" | awk '{print $4}')
check "stereo tx (m)" "$tx" 1.3633 0.10 || fail=1

# RGB-D: TUM trajectory is "t tx ty tz qx qy qz qw", so tx is field 2.
EXTRA="$WORK/rgbd/associations.txt" \
    run rgbd "$R/Examples/RGB-D/TUM1.yaml" "$R/Examples/RGB-D/rgbd_tum"
tx=$(tail -1 "$WORK/rgbd_out/CameraTrajectory.txt" | awk '{print $2}')
check "rgbd tx (m)" "$tx" 0.2861 0.10 || fail=1

echo
[ $fail -eq 0 ] && echo "smoke test PASSED" || echo "smoke test FAILED"
exit $fail
