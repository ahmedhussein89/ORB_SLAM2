# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

ORB-SLAM2 (Mur-Artal & Tardós), a real-time monocular/stereo/RGB-D SLAM library. GPLv3. C++20, [cabin](https://cabinpkg.com) (`cabin.toml`).

This fork is mid-migration off the original's dependencies: ROS and Pangolin are **gone**, the build targets OpenCV 4 / GCC 15 and moved from CMake to cabin, and the library is now headless. A GLFW + Dear ImGui GUI is planned as a *separate executable* talking to a snapshot API — do not reintroduce a GUI dependency into the core library.

## Build

```bash
./build.sh          # untars the vocabulary, then `cabin build --release`
```

**This machine is WSL2 with 16 cores; saturating all of them takes the WSL service
down mid-build.** `build.sh` therefore exports `CABIN_BUILD_JOBS=12` unless you set it
yourself. Invoking cabin directly, pass `-j12` (or export the same variable). The same
cap applies to long dataset runs -- prefix them with `taskset -c 0-11`.

It is idempotent; rerunning is fine. For an incremental rebuild after editing `src/` or `include/`:

```bash
cabin build --release      # or `cabin check` for a fast syntax-only pass
```

One package, `cabin.toml` at the root. The library is two targets, not one:

- `orbslam2_core` -- `src/core/` + `src/features/`: Map, MapPoint, KeyFrame, Frame,
  Converter, ORBextractor, ORBmatcher, KeyFrameDatabase. These are mutually cyclic
  (Frame needs ORBextractor, ORBmatcher needs Frame) so they cannot be split further.
- `orbslam2_slam` -- `src/tracking/`, `src/mapping/`, `src/loop_closing/`,
  `src/optimization/`, `src/system/`. Depends on `orbslam2_core`, publicly.
  **Nothing depends back on it**; that one-way edge is the point of the split, so do
  not add a `#include "orbslam/system/..."` (or tracking/mapping/loop_closing) to
  anything under `src/core/` or `src/features/`.

Plus `DBoW2`, `g2o`, the six examples, and `orbslam2_tests`.
**Nothing is globbed** -- a new `.cpp` file has to be added to a `sources` array by hand.

Output: static `lib{DBoW2,g2o,orbslam2_core,orbslam2_slam}.a` and all binaries in
`build/release/packages/ORB_SLAM2/` (cabin has no shared-library kind, and it does not
write into `Examples/` the way CMake did). Every example takes
`<vocabulary> <settings.yaml> <sequence path...>`, e.g.

```bash
./build/release/packages/ORB_SLAM2/mono_tum Vocabulary/ORBvoc.txt Examples/Monocular/TUM1.yaml PATH_TO_SEQUENCE
```

`rgbd_tum` additionally needs an RGB/depth associations file (some are provided in `Examples/RGB-D/associations/`); the EuRoC examples additionally need a timestamps file from `Examples/*/EuRoC_TimeStamps/`.

Deps: OpenCV 4, Eigen3 3.1+, and GoogleTest/GoogleMock for the tests. All three are
`system = true` in `cabin.toml`, i.e. found through **pkg-config** (`opencv4`, `eigen3`,
`gmock_main`) rather than fetched -- install them with the OS package manager. A missing
`.pc` file is what "package not found" from `cabin build` means. No Pangolin, no ROS.

### -march=native is an ABI flag here

Use `cabin build --profile native` (a `[profile.native]` inheriting `release`); there is no
per-target flag knob in cabin, so the flag necessarily applies to DBoW2, g2o,
`orbslam2_core` and `orbslam2_slam` at once — which is what makes it safe, because **the setting must be identical across them**. It raises `EIGEN_MAX_ALIGN_BYTES` (16 → 64 on AVX-512), which changes the alignment and size of Eigen-containing types such as `g2o::SE3Quat`. Mixing the setting across those libraries segfaults inside `g2o::BaseVertex::push` during the first global bundle adjustment, with no build-time warning. Do not build the Thirdparty sources standalone with a different setting.

## Testing

Two layers. Unit tests (GoogleTest, one file per translation unit in `tests/`, flat -- they are
not split by module the way `src/` is):

```bash
cabin test
```

`test` targets are built only by `cabin test`, so `build.sh` is unaffected. The whole suite is one
binary, `build/dev/packages/ORB_SLAM2/orbslam2_tests`, and runs in a few seconds. `cabin test`
reports one pass/fail per target from the exit status -- it does not count gtest cases, so read
the `[  PASSED  ] N tests.` line to see the real count. It never loads
`Vocabulary/ORBvoc.txt`: `tests/synthetic.hpp` trains a 729-word vocabulary in memory from ORB
descriptors of a synthetic dot plane, and supplies the frames, keyframes, calibration and
settings file every test shares. Read that header before adding a test -- it records the
traps (`Frame`'s mutable statics, the deliberate leaks, the bounded-wait helper for the
thread tests).

ORB extraction is bit-reproducible and `ORBextractor.IsDeterministic` guards that; everything
downstream of RANSAC or g2o is not, so those tests seed `rand()` and assert directions,
magnitudes and inlier counts rather than exact numbers. `Tracking`, `LocalMapping`, `LoopClosing`, `KeyFrameDatabase` and `System` are covered for
lifecycle, wiring and short end-to-end runs, not for SLAM quality.

The end-to-end smoke test:

```bash
./tools/smoke_test.sh
```

Generates synthetic sequences (`tools/make_synth_seq.py`), runs all three sensor modes, and checks the recovered motion. The camera pans a known number of pixels across a plane at a known depth, so for stereo and RGB-D — which recover true metric scale — the expected translation follows from the intrinsics and is checked in metres. Monocular has no scale, so it checks the x:y direction ratio. It fails on a wrong magnitude, a sign flip, or no motion.

This catches a broken build or a broken port. It is **not** a regression test: there is no recorded baseline, and a planar 50-frame scene exercises no loop closure, no relocalization and no scale drift.

For a real check that a change did not regress SLAM, run a KITTI sequence and score it.
`tools/kitti_eval.py <estimate> <ground-truth>` (numpy only -- no `evo`, no gnuplot, the
C++ devkit is not compiled) prints the official KITTI segment translation/rotation error
plus an unaligned ATE RMSE. `--selfcheck` scores the ground truth against itself and must
print zero.

The odometry zips are in `../dataset/`; extract per sequence (`unzip -n
data_odometry_gray.zip 'dataset/sequences/00/*'`) rather than unpacking all 22 GB. Pair
the settings file correctly -- **00-02 use `KITTI00-02.yaml`, 04-12 use
`KITTI04-12.yaml`** (different `fx` and `Camera.bf`; the wrong one silently rescales the
whole trajectory). `SaveTrajectoryKITTI` writes a *relative* path, so run each sequence
from its own output directory. `runs/` is gitignored.

```bash
D=../dataset/dataset
mkdir -p runs/00 && (cd runs/00 && taskset -c 0-11 ../../build/release/packages/ORB_SLAM2/stereo_kitti \
  ../../Vocabulary/ORBvoc.txt ../../Examples/Stereo/KITTI00-02.yaml $D/sequences/00 | tee run.log)
python3 tools/kitti_eval.py runs/00/CameraTrajectory.txt $D/poses/00.txt
```

Measured on this port (stereo), against the paper's ~0.70 % for 00 and ~0.9-1.0 % for 04:

| seq | frames | translation | rotation | ATE RMSE | samples |
|-----|--------|-------------|----------|----------|---------|
| 04  | 271    | 0.44-0.50 % | ~0.09 deg/100m | 0.76-0.96 m | 7 |
| 00  | 4541   | 0.694-0.705 % | 0.25 deg/100m | 7.2-7.7 m | 3 |

Those are *ranges over repeated runs of the same binary*, not a tolerance. Seq 00 also
closes 4 loops and reports `New map created with 848 points` every time; a change in
either is a stronger regression signal than a tenth of a percent of drift.

Seq 00's ATE looks high against the paper's ~1.3 m; the segment metric is the meaningful
one over a 3.7 km trajectory with loop-closure BA jumps. Nothing is bit-deterministic
(RANSAC + thread interleaving), so compare magnitudes, and treat any `LOST` /
relocalization spam in `run.log` as a failure regardless of the final number.

Settings `.yaml` files carry camera intrinsics and ORB extractor params (`ORBextractor.nFeatures`, `nLevels`, `iniThFAST`/`minThFAST`). Tuning SLAM behaviour usually means editing these, not the code. The `Viewer.*` keys are leftovers from the Pangolin viewer and are no longer read by anything.

## Architecture

Source layout mirrors the module split (`.claude/plans/migration.md` §9). Headers live at
`include/orbslam/<module>/<Name>.hpp`, sources at `src/<module>/<Name>.cpp`, and every
include spells the full path: `#include "orbslam/core/Frame.hpp"`.

```
core/         Map MapPoint KeyFrame Frame Converter   -- orbslam2_core
features/     ORBextractor ORBmatcher ORBVocabulary KeyFrameDatabase
tracking/     Tracking Initializer                    -- orbslam2_slam
mapping/      LocalMapping
loop_closing/ LoopClosing
optimization/ Optimizer PnPsolver Sim3Solver
system/       System
```


`System` (src/system/System.cpp) is the only public entry point. Its constructor builds the map and spawns two long-lived threads; the caller's thread becomes the tracking thread by calling `TrackMonocular` / `TrackStereo` / `TrackRGBD` per frame, each returning the camera pose `Tcw`.

The three threads and what they own:

- **Tracking** (`src/tracking/Tracking.cpp`, runs in the caller's thread): extracts ORB, estimates the pose per frame, decides when to insert a KeyFrame. State machine `eTrackingState` = SYSTEM_NOT_READY / NO_IMAGES_YET / NOT_INITIALIZED / OK / LOST. Monocular init goes through `Initializer` (homography + fundamental model selection); stereo/RGB-D initialize from one frame via `StereoInitialization`. On LOST it relocalizes via `KeyFrameDatabase` + `PnPsolver`.
- **LocalMapping** (`src/mapping/LocalMapping.cpp`): consumes new KeyFrames, triangulates new MapPoints, culls redundant points and KeyFrames, runs `Optimizer::LocalBundleAdjustment`.
- **LoopClosing** (`src/loop_closing/LoopClosing.cpp`): DBoW2 place recognition, `Sim3Solver` + `Optimizer::OptimizeSim3` to compute the loop transform, `OptimizeEssentialGraph`, then a full BA in a *fourth* detached thread.

The threads are cross-wired after construction (`SetLocalMapper` / `SetLoopCloser` / `SetTracker`) and communicate through the shared `Map` plus request/ack flag pairs guarded by per-object mutexes (`RequestStop`/`isStopped`, `RequestReset`, `SetNotStop`). **Every shared object is mutex-protected internally** — `Map` (`mMutexMap`), `KeyFrame` (separate `mMutexPose`, `mMutexConnections`, `mMutexFeatures`), `MapPoint` (`mMutexPos`, `mMutexFeatures`). When touching these classes, take the same lock the surrounding methods take; deadlocks here are the main hazard in this codebase.

Supporting pieces: `ORBextractor` (FAST on a scale pyramid with an octree-based quadtree distribution), `ORBmatcher` (all matching variants: projection, BoW, epipolar, Sim3), `Optimizer` (all g2o graph construction — pose-only, local BA, global BA, essential graph, Sim3), `Converter` (cv::Mat ↔ Eigen/g2o types — used everywhere at the g2o boundary).

Localization Mode (`ActivateLocalizationMode`) stops LocalMapping and puts Tracking in `mbOnlyTracking` — the map is frozen and only the pose is estimated.

## Conventions

- Naming is Hungarian-ish and consistent: `mp` member pointer, `mv` member vector, `mb` member bool, `mn` member int, `mT` transform, `n`/`v`/`b`/`p` locals. Match it.
- Poses are `cv::Mat` 4x4 CV_32F world-to-camera (`Tcw`) unless the name says otherwise (`Twc` is the inverse).
- The public API in `include/` mirrors the paper; keep changes to it minimal — downstream users link against it. `System`'s constructor still takes `bUseViewer`; it is accepted and ignored, kept so the signature did not break.
- C-style casts were converted to `static_cast` everywhere **except `PnPsolver.cpp`**, which is left alone with the rest of its numerics.
- `PnPsolver`'s EPnP uses the OpenCV **C API** (`CvMat`, `cvSVD`, `cvSolve`, `cvInvert`) via `opencv2/core/core_c.h`. That API still ships in OpenCV 4, so it needs the header, not a rewrite. Leave the numerics alone.
- `Thirdparty/DBoW2` and `Thirdparty/g2o` are *modified* forks, not upstream. Do not swap them for packaged versions without treating it as an algorithm change. They are targets of the root package rather than sub-packages, because cabin probes `system = true` deps only for the primary package -- as path dependencies they would not see OpenCV or Eigen.
- `Thirdparty/g2o/config.h` used to be generated by `configure_file`. Cabin has no configure step, so the resolved header (both `#cmakedefine`s undefined, i.e. no OpenMP) is committed. Editing it is how you would turn `G2O_OPENMP` on -- along with adding `-fopenmp` to a profile.
- `include/`, `src/`, `Examples/` and `tests/` are clang-formatted (Google style,
  `.clang-format`); keep them that way (`cabin fmt`).
- **`cabin fmt` reformats `Thirdparty/` too, and there is no way to stop it here.**
  It walks the whole workspace, and `.clang-format-ignore` (which the repo carries)
  only takes effect from clang-format 18 — this machine has 15. So the workflow is
  `cabin fmt && git checkout -- Thirdparty/`. That revert is safe *only because the
  vendored patches below are committed* — run it with uncommitted Thirdparty work in
  the tree and you will silently lose it. Commit vendored edits before formatting.
- **Public headers are self-contained** and no longer depend on include order. Every
  header under `include/orbslam/` compiles standalone (`g++ -fsyntax-only` on a file
  that includes only it), spells `std::` explicitly, and pulls its own `<vector>`,
  `<mutex>`, etc. This retired the old hazard where headers borrowed an unqualified
  `vector` from the `using namespace std;` that `Thirdparty/DBoW2`'s
  `TemplatedVocabulary.h` and `Timestamp.h` leak. If you add a header, keep that
  property — it is cheap to check and it is what makes the module split honest.
  `.clang-format` still sets `SortIncludes: Never` / `IncludeBlocks: Preserve`, because
  the **`.cpp` files** were not audited: they still carry a file-scope
  `using namespace std;` and have not been proven order-independent.
- Headers use `#pragma once`, not include guards.
- **The build is warning-clean under `-Wall -Wextra`, `Thirdparty/` included — keep it
  that way.** Where a vendored warning could not be fixed without touching semantics,
  it is suppressed at the exact declaration with a scoped
  `#pragma GCC diagnostic push/ignored/pop` and a comment saying why: Eigen's
  deprecated `AlignedBit` in g2o's `base_vertex.h` / `base_binary_edge.h` (the
  expression decides Hessian-block alignment and must not be modernised), and
  `-Woverloaded-virtual` in DBoW2's `FClass.h` (a documentation-only base that `FORB`
  deliberately hides). Do not widen those to file-level suppressions.
- `Frame` declares `operator=(const Frame&) = default` on purpose. Its copy *constructor*
  deep-copies the `cv::Mat` members and routes the pose through `SetPose()` (skipping an
  empty `mTcw`, so the target keeps its old pose); assignment has always been the plain
  memberwise version and `Tracking` depends on that. The explicit `= default` keeps the
  behaviour and silences `-Wdeprecated-copy`. Do not "fix" it to mirror the constructor.
- Diagnostics end in `'\n'`, not `std::endl` (SL.io.50) — so the examples set
  `std::cout << std::unitbuf` in `main`. Without it a redirected long run buffers and
  prints nothing until it exits, which makes `tee`/`tail -f` on a KITTI run useless.
- **`System` deliberately never frees the objects it `new`s** (vocabulary, map, KFDB,
  tracker, local mapper, loop closer). `LoopClosing` can leave a *detached* global-BA
  thread running past `Shutdown()`, so freeing the `Map` under it is a use-after-free —
  an intermittent crash in the `Tracking` tests is exactly what happens if you convert
  those members to `unique_ptr`. Reclaiming them means making that BA thread joinable
  first, which is a behaviour change, not a refactor. The two *thread* members are
  `std::jthread` by value and do get joined (`~System` calls the idempotent `Shutdown()`
  first); `LoopClosing` holds its BA thread by value and `~LoopClosing` detaches a live
  one rather than letting `~thread` call `terminate()`.

## Fixed upstream bugs

Divergences from upstream ORB-SLAM2, each covered by a test:

- `Converter::toSE3Quat(const g2o::Sim3&)` was declared in `include/Converter.h` and defined nowhere; the declaration is gone. Nothing called it, and nothing could have -- it never linked.
- `Initializer::Initialize` draws eight matches per RANSAC iteration with `RandomInt(0, N-1)`, which underflowed and indexed out of bounds below eight matches. It now returns false. `Tracking` never reached this (it bails out below 100 matches).
- `Sim3Solver::ComputeSim3` normalised the rotation axis with `vec / norm(vec)`, returning an all-NaN transform when the true rotation was exactly the identity. Guarded; a zero angle-axis vector is already the identity for `Rodrigues`.
- `ORBextractor::DistributeOctTree` sorted `(node size, ExtractorNode*)` pairs, so ties broke on heap addresses and roughly 2-3% of keypoints moved between two extractions of the same image. It now `stable_sort`s on the size alone, which makes extraction bit-reproducible.
- The four `read()` overloads in `g2o`'s `types_six_dof_expmap.cpp` /
  `types_seven_dof_expmap.cpp` filled a 2x2 information matrix with
  `for (int i=0;i<2;i++) for (int j=i;j<2;j++)`. GCC miscounts that trip count over an
  Eigen 2x2 and flags the (provably in-bounds) body as undefined behaviour
  (`-Waggressive-loop-optimizations`). Unrolled to three explicit reads plus the
  mirrored entry -- same values, same order. ORB-SLAM2 never calls them.
- `System::mTrackingState` was never initialised, so `GetTrackingState()` returned garbage before the first `Track*` call. Now `Tracking::SYSTEM_NOT_READY`.

## Known upstream bugs, not yet fixed

- `LoadImages` in every example uses `while(!f.eof())`. If the sequence path is wrong the stream never opens, `eof()` never becomes true, and the binary spins at 100% CPU forever with no output instead of reporting the bad path.

## Core Guidelines: applied, and deliberately not

The tree follows the C++ Core Guidelines subset that does not touch behaviour: `#pragma
once` and self-contained headers (SF.8/SF.11), no `using namespace` in a header (SF.7),
`nullptr` (ES.47), `static_cast` over C-style casts (ES.48, outside `PnPsolver`), `'\n'`
over `std::endl` (SL.io.50), `explicit` single-argument constructors (C.46), and Rule of
Five on the two classes that own threads (C.21). C++20 is on; `std::jthread` is used
where a thread is owned.

Three things are knowingly *not* done, each because it is an API or behaviour change
rather than a refactor — `.claude/plans/migration.md` §31 is explicit that those must be
separate steps:

- `enum eSensor` / `eTrackingState` are unscoped enums (Enum.3 wants `enum class`).
  They are public API that downstream code compares against ints.
- `Frame`'s mutable file-scope statics (`nNextId`, the undistortion bounds, the grid
  parameters) are global mutable state (I.2). They are load-bearing for the algorithm and
  are documented as a trap in `tests/synthetic.hpp`.
- The SLAM classes wire themselves together with raw `Set*` setters rather than
  constructor injection, and there are no interfaces to depend on (D of SOLID). The
  module split is the structural half of that work; introducing seams is migration.md
  milestones 4-7 and needs the dataset regression baseline in place first.
