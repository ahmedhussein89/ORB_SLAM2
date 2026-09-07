# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

ORB-SLAM2 (Mur-Artal & Tardós), a real-time monocular/stereo/RGB-D SLAM library. GPLv3. C++17, CMake.

This fork is mid-migration off the original's dependencies: ROS and Pangolin are **gone**, the build targets OpenCV 4 / GCC 15 / CMake 4, and the library is now headless. A GLFW + Dear ImGui GUI is planned as a *separate executable* talking to a snapshot API — do not reintroduce a GUI dependency into the core library.

## Build

```bash
./build.sh          # untars the vocabulary, then configures/builds everything in one tree
```

It is idempotent; rerunning is fine. For an incremental rebuild after editing `src/` or `include/`:

```bash
cmake --build build -j
```

Output: `lib/lib{ORB_SLAM2,DBoW2,g2o}.so`, binaries in `Examples/{Monocular,Stereo,RGB-D}/`. Every example takes `<vocabulary> <settings.yaml> <sequence path...>`, e.g.

```bash
./Examples/Monocular/mono_tum Vocabulary/ORBvoc.txt Examples/Monocular/TUM1.yaml PATH_TO_SEQUENCE
```

`rgbd_tum` additionally needs an RGB/depth associations file (some are provided in `Examples/RGB-D/associations/`); the EuRoC examples additionally need a timestamps file from `Examples/*/EuRoC_TimeStamps/`.

Deps: OpenCV 4, Eigen3 3.1+. No Pangolin, no ROS.

### -march=native is an ABI flag here

It is off by default, behind `ORBSLAM2_NATIVE_ARCH`. DBoW2 and g2o are `add_subdirectory`'d from the top-level `CMakeLists.txt`, so the one option propagates to all three — but **the setting must be identical across them**. It raises `EIGEN_MAX_ALIGN_BYTES` (16 → 64 on AVX-512), which changes the alignment and size of Eigen-containing types such as `g2o::SE3Quat`. Mixing the setting across those three libraries segfaults inside `g2o::BaseVertex::push` during the first global bundle adjustment, with no build-time warning. Do not build the Thirdparty projects standalone with a different setting.

## Testing

```bash
./tools/smoke_test.sh
```

Generates synthetic sequences (`tools/make_synth_seq.py`), runs all three sensor modes, and checks the recovered motion. The camera pans a known number of pixels across a plane at a known depth, so for stereo and RGB-D — which recover true metric scale — the expected translation follows from the intrinsics and is checked in metres. Monocular has no scale, so it checks the x:y direction ratio. It fails on a wrong magnitude, a sign flip, or no motion.

This catches a broken build or a broken port. It is **not** a regression test: there is no recorded baseline, and a planar 50-frame scene exercises no loop closure, no relocalization and no scale drift.

For a real check that a change did not regress SLAM, run a TUM/KITTI/EuRoC sequence and compare trajectories: on shutdown the examples call `System::SaveTrajectoryTUM` / `SaveTrajectoryKITTI` / `SaveKeyFrameTrajectoryTUM`. Results are not bit-deterministic (RANSAC + thread interleaving), so compare magnitudes, not exact numbers.

Settings `.yaml` files carry camera intrinsics and ORB extractor params (`ORBextractor.nFeatures`, `nLevels`, `iniThFAST`/`minThFAST`). Tuning SLAM behaviour usually means editing these, not the code. The `Viewer.*` keys are leftovers from the Pangolin viewer and are no longer read by anything.

## Architecture

`System` (src/System.cc) is the only public entry point. Its constructor builds the map and spawns two long-lived threads; the caller's thread becomes the tracking thread by calling `TrackMonocular` / `TrackStereo` / `TrackRGBD` per frame, each returning the camera pose `Tcw`.

The three threads and what they own:

- **Tracking** (`Tracking.cc`, runs in the caller's thread): extracts ORB, estimates the pose per frame, decides when to insert a KeyFrame. State machine `eTrackingState` = SYSTEM_NOT_READY / NO_IMAGES_YET / NOT_INITIALIZED / OK / LOST. Monocular init goes through `Initializer` (homography + fundamental model selection); stereo/RGB-D initialize from one frame via `StereoInitialization`. On LOST it relocalizes via `KeyFrameDatabase` + `PnPsolver`.
- **LocalMapping** (`LocalMapping.cc`): consumes new KeyFrames, triangulates new MapPoints, culls redundant points and KeyFrames, runs `Optimizer::LocalBundleAdjustment`.
- **LoopClosing** (`LoopClosing.cc`): DBoW2 place recognition, `Sim3Solver` + `Optimizer::OptimizeSim3` to compute the loop transform, `OptimizeEssentialGraph`, then a full BA in a *fourth* detached thread.

The threads are cross-wired after construction (`SetLocalMapper` / `SetLoopCloser` / `SetTracker`) and communicate through the shared `Map` plus request/ack flag pairs guarded by per-object mutexes (`RequestStop`/`isStopped`, `RequestReset`, `SetNotStop`). **Every shared object is mutex-protected internally** — `Map` (`mMutexMap`), `KeyFrame` (separate `mMutexPose`, `mMutexConnections`, `mMutexFeatures`), `MapPoint` (`mMutexPos`, `mMutexFeatures`). When touching these classes, take the same lock the surrounding methods take; deadlocks here are the main hazard in this codebase.

Supporting pieces: `ORBextractor` (FAST on a scale pyramid with an octree-based quadtree distribution), `ORBmatcher` (all matching variants: projection, BoW, epipolar, Sim3), `Optimizer` (all g2o graph construction — pose-only, local BA, global BA, essential graph, Sim3), `Converter` (cv::Mat ↔ Eigen/g2o types — used everywhere at the g2o boundary).

Localization Mode (`ActivateLocalizationMode`) stops LocalMapping and puts Tracking in `mbOnlyTracking` — the map is frozen and only the pose is estimated.

## Conventions

- Naming is Hungarian-ish and consistent: `mp` member pointer, `mv` member vector, `mb` member bool, `mn` member int, `mT` transform, `n`/`v`/`b`/`p` locals. Match it.
- Poses are `cv::Mat` 4x4 CV_32F world-to-camera (`Tcw`) unless the name says otherwise (`Twc` is the inverse).
- The public API in `include/` mirrors the paper; keep changes to it minimal — downstream users link against it. `System`'s constructor still takes `bUseViewer`; it is accepted and ignored, kept so the signature did not break.
- `PnPsolver`'s EPnP uses the OpenCV **C API** (`CvMat`, `cvSVD`, `cvSolve`, `cvInvert`) via `opencv2/core/core_c.h`. That API still ships in OpenCV 4, so it needs the header, not a rewrite. Leave the numerics alone.
- `Thirdparty/DBoW2` and `Thirdparty/g2o` are *modified* forks, not upstream. Do not swap them for packaged versions without treating it as an algorithm change.
- `include/`, `src/` and `Examples/` are clang-formatted (Google style, `.clang-format`); keep them that way. `Thirdparty/` is deliberately excluded — it is vendored.
- **Include order is load-bearing**, which is why `.clang-format` sets `SortIncludes: Never` and `IncludeBlocks: Preserve`. Several headers (`Map.h`, `Frame.h`, `Initializer.h`, ...) use unqualified `vector`/`pair`/`map` and only compile because `Thirdparty/DBoW2`'s `TemplatedVocabulary.h` and `Timestamp.h` leak a `using namespace std;` in through an earlier include. Reordering includes breaks the build with `'vector' does not name a type`. Qualifying those names properly is a separate change from a formatting pass.

## Known upstream bugs, not yet fixed

- `LoadImages` in every example uses `while(!f.eof())`. If the sequence path is wrong the stream never opens, `eof()` never becomes true, and the binary spins at 100% CPU forever with no output instead of reporting the bad path.
- `g2o`'s `EdgeSE3ProjectXYZ::read` / `EdgeSim3ProjectXYZ::read` have loops GCC flags as undefined behaviour (`-Waggressive-loop-optimizations`). ORB-SLAM2 never calls them.
