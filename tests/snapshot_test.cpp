#include "Snapshot.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <unordered_set>

#include "orbslam/core/KeyFrame.hpp"
#include "orbslam/core/Map.hpp"
#include "orbslam/core/MapPoint.hpp"
#include "synthetic.hpp"

using ORB_SLAM2::System;

namespace {

// BuildSnapshot is the boundary between the SLAM threads and the render thread:
// everything it emits must be a value copy, and nothing bad may survive the crossing.
// The GL and ImGui code above it has no unit test -- it is verified by looking at it.
struct SnapshotTest : ::testing::Test {
  static std::shared_ptr<const orbslam_viewer::Snapshot> RunRgbd(System& slam, int frames) {
    std::shared_ptr<const orbslam_viewer::Snapshot> snap;
    for(int i = 0; i < frames; ++i) {
      const cv::Mat Tcw = slam.TrackRGBD(synth::SynthImage(4 * i), synth::SynthDepth(), i / 30.0);
      snap = orbslam_viewer::BuildSnapshot(slam, Tcw);
    }
    return snap;
  }
};

TEST_F(SnapshotTest, MirrorsTheMapAndTheLiveFrame) {
  System slam(synth::TinyVocabFile(), synth::SettingsYaml(), System::RGBD, false);
  const auto snap = RunRgbd(slam, 5);
  ASSERT_NE(snap, nullptr);

  EXPECT_EQ(snap->state, ORB_SLAM2::Tracking::OK);
  EXPECT_TRUE(snap->pose_valid);
  EXPECT_EQ(snap->point_xyz.size(), 3 * snap->point_ids.size());
  EXPECT_EQ(snap->point_ref.size(), snap->point_ids.size());
  EXPECT_EQ(snap->n_map_points, snap->point_ids.size());
  EXPECT_GT(snap->n_map_points, 100u);
  EXPECT_GE(snap->n_key_frames, 1u);
  EXPECT_EQ(snap->key_frames.size(), snap->n_key_frames);

  // The live frame travels with its keypoints and a deep copy of the image.
  EXPECT_FALSE(snap->current_image.empty());
  EXPECT_EQ(snap->current_keys.size(), snap->current_mapped.size());
  EXPECT_GT(snap->current_keys.size(), 100u);
  EXPECT_FLOAT_EQ(snap->width, static_cast<float>(synth::Width()));
  // fx round-trips through the settings YAML, so it is close but not bit-equal.
  EXPECT_NEAR(snap->fx, synth::Calib().K.at<float>(0, 0), 1e-2);

  slam.Shutdown();
}

TEST_F(SnapshotTest, ExcludesBadPointsAndKeyFrames) {
  System slam(synth::TinyVocabFile(), synth::SettingsYaml(), System::RGBD, false);
  RunRgbd(slam, 3);

  ORB_SLAM2::Map* const map = slam.GetMap();
  std::vector<ORB_SLAM2::MapPoint*> points = map->GetAllMapPoints();
  ASSERT_GT(points.size(), 10u);
  const unsigned long killed = points.front()->mnId;
  points.front()->SetBadFlag();

  const cv::Mat Tcw = slam.TrackRGBD(synth::SynthImage(12), synth::SynthDepth(), 0.2);
  const auto snap = orbslam_viewer::BuildSnapshot(slam, Tcw);
  ASSERT_NE(snap, nullptr);

  // Bad MapPoints are never freed, so a snapshot that forgot to filter them would
  // still "work" -- and would draw a stale point forever.
  EXPECT_EQ(std::count(snap->point_ids.begin(), snap->point_ids.end(), killed), 0);
  for(const orbslam_viewer::KeyFrameView& kf : snap->key_frames)
    EXPECT_EQ(std::count(kf.point_ids.begin(), kf.point_ids.end(), killed), 0);

  slam.Shutdown();
}

TEST_F(SnapshotTest, KeyFrameViewMatchesTheKeyFrame) {
  System slam(synth::TinyVocabFile(), synth::SettingsYaml(), System::RGBD, false);
  const auto snap = RunRgbd(slam, 4);
  ASSERT_NE(snap, nullptr);
  ASSERT_FALSE(snap->key_frames.empty());

  std::vector<ORB_SLAM2::KeyFrame*> kfs = slam.GetMap()->GetAllKeyFrames();
  const orbslam_viewer::KeyFrameView& view = snap->key_frames.front();
  const auto it = std::find_if(kfs.begin(), kfs.end(), [&](ORB_SLAM2::KeyFrame* pKF) { return pKF->mnId == view.id; });
  ASSERT_NE(it, kfs.end());
  ORB_SLAM2::KeyFrame* const pKF = *it;

  EXPECT_EQ(view.n_features, pKF->N);
  EXPECT_EQ(view.keypoints.size(), pKF->mvKeysUn.size());
  EXPECT_EQ(view.point_ids.size(), view.keypoint_index.size());
  EXPECT_EQ(view.n_tracked, static_cast<int>(view.point_ids.size()));
  EXPECT_DOUBLE_EQ(view.timestamp, pKF->mTimeStamp);

  // The image overlay draws point_ids[k] at keypoints[keypoint_index[k]], so that
  // pairing has to be the keyframe's own index alignment, not a re-derived one.
  const std::vector<ORB_SLAM2::MapPoint*> matches = pKF->GetMapPointMatches();
  for(std::size_t k = 0; k < view.point_ids.size(); ++k) {
    const int idx = view.keypoint_index[k];
    ASSERT_GE(idx, 0);
    ASSERT_LT(static_cast<std::size_t>(idx), matches.size());
    ASSERT_NE(matches[static_cast<std::size_t>(idx)], nullptr);
    EXPECT_EQ(matches[static_cast<std::size_t>(idx)]->mnId, view.point_ids[k]);
    EXPECT_FLOAT_EQ(view.keypoints[static_cast<std::size_t>(idx)][0], pKF->mvKeysUn[idx].pt.x);
  }

  // Twc is column-major for OpenGL: the translation lands in elements 12..14.
  const cv::Mat Ow = pKF->GetCameraCenter();
  EXPECT_NEAR(view.Twc[12], Ow.at<float>(0), 1e-5);
  EXPECT_NEAR(view.Twc[13], Ow.at<float>(1), 1e-5);
  EXPECT_NEAR(view.Twc[14], Ow.at<float>(2), 1e-5);
  EXPECT_NEAR(view.center[0], Ow.at<float>(0), 1e-5);

  slam.Shutdown();
}

TEST_F(SnapshotTest, LostTrackingLeavesAnIdentityPoseButKeepsTheMap) {
  System slam(synth::TinyVocabFile(), synth::SettingsYaml(), System::RGBD, false);
  RunRgbd(slam, 3);
  const auto snap = orbslam_viewer::BuildSnapshot(slam, cv::Mat());
  ASSERT_NE(snap, nullptr);

  EXPECT_FALSE(snap->pose_valid);
  EXPECT_FLOAT_EQ(snap->Twc[0], 1.0f);
  EXPECT_FLOAT_EQ(snap->Twc[15], 1.0f);
  EXPECT_FLOAT_EQ(snap->Twc[12], 0.0f);
  EXPECT_GT(snap->n_map_points, 100u);

  slam.Shutdown();
}

}    // namespace
