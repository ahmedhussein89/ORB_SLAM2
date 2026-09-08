#include "orbslam/system/System.hpp"

#include <gtest/gtest.h>

#include <cstdio>
#include <fstream>
#include <sstream>

#include "synthetic.hpp"

using ORB_SLAM2::System;

namespace {

// System is the whole pipeline behind one object, so these are integration-shaped: build it on
// the tiny vocabulary, push a few synthetic frames through, and check the public contract
// (state, localization mode, reset, the three trajectory writers). The SLAM quality question
// belongs to tools/smoke_test.sh, which runs real sequence lengths through the example binaries.
struct SystemTest : ::testing::Test {
  static int CountColumns(const std::string& path, int* lines) {
    std::ifstream f(path);
    std::string line;
    int cols = -1;
    *lines = 0;
    while(std::getline(f, line)) {
      if(line.empty())
        continue;
      std::istringstream iss(line);
      std::string tok;
      int n = 0;
      while(iss >> tok)
        ++n;
      if(cols < 0)
        cols = n;
      if(cols != n)
        return -1;    // ragged file
      ++(*lines);
    }
    return cols;
  }
};

TEST_F(SystemTest, ConstructsForEverySensorAndShutsDown) {
  for(System::eSensor sensor : {System::MONOCULAR, System::STEREO, System::RGBD}) {
    System slam(synth::TinyVocabFile(), synth::SettingsYaml(), sensor, false);
    EXPECT_EQ(slam.GetTrackingState(), ORB_SLAM2::Tracking::SYSTEM_NOT_READY)
        << "GetTrackingState is only refreshed by a Track* call, so before the first frame it "
           "must report the constructor's value";
    EXPECT_TRUE(slam.GetTrackedMapPoints().empty());
    EXPECT_TRUE(slam.GetTrackedKeyPointsUn().empty());
    slam.Shutdown();
  }
}

TEST_F(SystemTest, TheViewerFlagIsAcceptedAndIgnored) {
  // System.h promises the signature did not break when the viewer was removed.
  System slam(synth::TinyVocabFile(), synth::SettingsYaml(), System::RGBD, true);
  EXPECT_EQ(slam.GetTrackingState(), ORB_SLAM2::Tracking::SYSTEM_NOT_READY);
  slam.Shutdown();
}

TEST_F(SystemTest, TrackRgbdInitialisesAndTracks) {
  System slam(synth::TinyVocabFile(), synth::SettingsYaml(), System::RGBD, false);

  const cv::Mat first = slam.TrackRGBD(synth::SynthImage(0), synth::SynthDepth(), 0.0);
  ASSERT_FALSE(first.empty());
  EXPECT_EQ(slam.GetTrackingState(), ORB_SLAM2::Tracking::OK);
  EXPECT_GT(slam.GetTrackedMapPoints().size(), 100u);
  EXPECT_EQ(slam.GetTrackedMapPoints().size(), slam.GetTrackedKeyPointsUn().size());

  cv::Mat last;
  for(int i = 1; i <= 4; ++i)
    last = slam.TrackRGBD(synth::SynthImage(4 * i), synth::SynthDepth(), i / 30.0);

  ASSERT_FALSE(last.empty());
  EXPECT_EQ(slam.GetTrackingState(), ORB_SLAM2::Tracking::OK);
  const float expected = -4 * 4 * synth::PlaneDepth() / synth::Calib().K.at<float>(0, 0);
  EXPECT_NEAR(last.at<float>(0, 3), expected, 0.05f);

  slam.Shutdown();
}

TEST_F(SystemTest, TrackStereoInitialises) {
  System slam(synth::TinyVocabFile(), synth::SettingsYaml(), System::STEREO, false);

  const cv::Mat Tcw = slam.TrackStereo(synth::SynthImage(0), synth::SynthImageRight(0), 0.0);

  ASSERT_FALSE(Tcw.empty());
  EXPECT_EQ(slam.GetTrackingState(), ORB_SLAM2::Tracking::OK);
  slam.Shutdown();
}

TEST_F(SystemTest, TrackMonocularNeedsTwoViewsToInitialise) {
  System slam(synth::TinyVocabFile(), synth::SettingsYaml(), System::MONOCULAR, false);

  EXPECT_TRUE(slam.TrackMonocular(synth::SynthImage(0), 0.0).empty()) << "one view cannot fix a monocular pose";
  EXPECT_EQ(slam.GetTrackingState(), ORB_SLAM2::Tracking::NOT_INITIALIZED);
  slam.Shutdown();
}

TEST_F(SystemTest, LocalizationModeTogglesAndResetClearsTheMap) {
  System slam(synth::TinyVocabFile(), synth::SettingsYaml(), System::RGBD, false);
  slam.TrackRGBD(synth::SynthImage(0), synth::SynthDepth(), 0.0);
  ASSERT_EQ(slam.GetTrackingState(), ORB_SLAM2::Tracking::OK);

  slam.ActivateLocalizationMode();
  slam.TrackRGBD(synth::SynthImage(4), synth::SynthDepth(), 1 / 30.0);
  EXPECT_EQ(slam.GetTrackingState(), ORB_SLAM2::Tracking::OK);

  slam.DeactivateLocalizationMode();
  slam.TrackRGBD(synth::SynthImage(8), synth::SynthDepth(), 2 / 30.0);
  EXPECT_EQ(slam.GetTrackingState(), ORB_SLAM2::Tracking::OK);

  slam.Reset();
  slam.TrackRGBD(synth::SynthImage(0), synth::SynthDepth(), 3 / 30.0);
  EXPECT_EQ(slam.GetTrackingState(), ORB_SLAM2::Tracking::OK) << "the system must re-initialise after a reset";

  slam.Shutdown();
}

TEST_F(SystemTest, TrajectoryWritersProduceParsableFiles) {
  System slam(synth::TinyVocabFile(), synth::SettingsYaml(), System::RGBD, false);
  for(int i = 0; i < 5; ++i)
    slam.TrackRGBD(synth::SynthImage(4 * i), synth::SynthDepth(), i / 30.0);
  ASSERT_EQ(slam.GetTrackingState(), ORB_SLAM2::Tracking::OK);

  slam.Shutdown();

  const std::string tum = synth::TempPath("traj_tum.txt");
  const std::string kf = synth::TempPath("traj_kf.txt");
  const std::string kitti = synth::TempPath("traj_kitti.txt");
  slam.SaveTrajectoryTUM(tum);
  slam.SaveKeyFrameTrajectoryTUM(kf);
  slam.SaveTrajectoryKITTI(kitti);

  int lines = 0;
  EXPECT_EQ(CountColumns(tum, &lines), 8) << "TUM is: timestamp tx ty tz qx qy qz qw";
  EXPECT_EQ(lines, 5) << "one row per tracked frame";

  EXPECT_EQ(CountColumns(kf, &lines), 8);
  EXPECT_GE(lines, 1) << "at least the initial keyframe";

  EXPECT_EQ(CountColumns(kitti, &lines), 12) << "KITTI is a row-major 3x4 matrix";
  EXPECT_EQ(lines, 5);

  std::remove(tum.c_str());
  std::remove(kf.c_str());
  std::remove(kitti.c_str());
}

}    // namespace
