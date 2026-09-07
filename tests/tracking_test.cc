#include "include/Tracking.h"

#include <gtest/gtest.h>

#include "include/System.h"
#include "synthetic.h"

using ORB_SLAM2::System;
using ORB_SLAM2::Tracking;

namespace {

// Tracking runs in the caller's thread, so it can be driven directly -- but it needs a live
// LocalMapping and LoopClosing to hand keyframes to. The System pointer is only dereferenced
// on the "lost right after initialisation" path, which these tests do not take, so it stays
// null here.
struct TrackingTest : ::testing::Test {
  synth::Pipeline p;
  Tracking tracker{nullptr, &synth::TinyVocab(), &p.map, &p.kfdb, synth::SettingsYaml(), System::RGBD};

  void SetUp() override {
    tracker.SetLocalMapper(&p.mapper);
    tracker.SetLoopClosing(&p.closer);
    p.Start();
  }

  void TearDown() override {
    EXPECT_TRUE(p.Shutdown()) << "a pipeline thread did not finish";
  }

  cv::Mat Grab(int shift, double t) {
    return tracker.GrabImageRGBD(synth::SynthImage(shift), synth::SynthDepth(), t);
  }
};

TEST_F(TrackingTest, StartsWithNoImages) {
  EXPECT_EQ(tracker.mState, Tracking::NO_IMAGES_YET);
  EXPECT_FALSE(tracker.mbOnlyTracking);
  EXPECT_EQ(tracker.mSensor, System::RGBD);
}

TEST_F(TrackingTest, FirstRgbdFrameInitialisesTheMap) {
  const cv::Mat Tcw = Grab(0, 0.0);

  EXPECT_EQ(tracker.mState, Tracking::OK) << "one RGB-D frame is enough to initialise";
  ASSERT_FALSE(Tcw.empty());
  EXPECT_LT(cv::norm(Tcw - cv::Mat::eye(4, 4, CV_32F)), 1e-5) << "the first pose is the identity";
  EXPECT_EQ(p.map.KeyFramesInMap(), 1u);
  EXPECT_GT(p.map.MapPointsInMap(), 100u);
}

TEST_F(TrackingTest, TracksThePanAcrossSeveralFrames) {
  Grab(0, 0.0);
  ASSERT_EQ(tracker.mState, Tracking::OK);

  cv::Mat Tcw;
  const int kShift = 4;
  for(int i = 1; i <= 6; ++i)
    Tcw = Grab(kShift * i, i / 30.0);

  ASSERT_EQ(tracker.mState, Tracking::OK) << "tracking was lost on a trivial pan";
  ASSERT_FALSE(Tcw.empty());

  // Panning the image window right is the camera moving right, so the world slides left in
  // camera coordinates: Tcw's tx grows negative by shift * depth / fx per frame.
  const float expected = -6 * kShift * synth::PlaneDepth() / synth::Calib().K.at<float>(0, 0);
  EXPECT_NEAR(Tcw.at<float>(0, 3), expected, 0.05f);
  EXPECT_NEAR(Tcw.at<float>(1, 3), 0.f, 0.05f);
}

TEST_F(TrackingTest, LocalizationModeFreezesTheMap) {
  Grab(0, 0.0);
  Grab(4, 1 / 30.0);
  ASSERT_EQ(tracker.mState, Tracking::OK);

  const unsigned long kfsBefore = p.map.KeyFramesInMap();
  tracker.InformOnlyTracking(true);
  EXPECT_TRUE(tracker.mbOnlyTracking);

  for(int i = 2; i <= 6; ++i)
    Grab(4 * i, i / 30.0);

  EXPECT_EQ(tracker.mState, Tracking::OK);
  EXPECT_EQ(p.map.KeyFramesInMap(), kfsBefore) << "localization mode must not grow the map";

  tracker.InformOnlyTracking(false);
  EXPECT_FALSE(tracker.mbOnlyTracking);
}

TEST_F(TrackingTest, ResetClearsTheMapAndTheState) {
  Grab(0, 0.0);
  Grab(4, 1 / 30.0);
  ASSERT_GT(p.map.MapPointsInMap(), 0u);

  tracker.Reset();

  EXPECT_EQ(tracker.mState, Tracking::NO_IMAGES_YET);
  EXPECT_EQ(p.map.KeyFramesInMap(), 0u);
  EXPECT_EQ(p.map.MapPointsInMap(), 0u);
  EXPECT_TRUE(tracker.mlRelativeFramePoses.empty());

  // ...and it can initialise again afterwards.
  Grab(0, 1.0);
  EXPECT_EQ(tracker.mState, Tracking::OK);
}

TEST_F(TrackingTest, ChangeCalibrationReloadsTheIntrinsics) {
  tracker.ChangeCalibration(synth::SettingsYaml());
  EXPECT_NEAR(ORB_SLAM2::Frame::fx, synth::Calib().K.at<float>(0, 0), 1e-3);
  EXPECT_NEAR(ORB_SLAM2::Frame::cy, synth::Calib().K.at<float>(1, 2), 1e-3);
}

}    // namespace
