#include "include/Initializer.h"

#include <gtest/gtest.h>

#include "include/ORBmatcher.h"
#include "synthetic.h"

using ORB_SLAM2::Frame;
using ORB_SLAM2::Initializer;

namespace {

// Monocular initialisation is RANSAC-driven. Seed it and assert directions and magnitudes,
// never exact numbers.
struct InitializerTest : ::testing::Test {
  void SetUp() override {
    srand(12345);
  }
};

TEST_F(InitializerTest, RecoversAPureSidewaysTranslation) {
  // 10 px of pan is not enough parallax for ReconstructH to accept a solution; 60 px at a
  // 2 m plane is ~23 cm of baseline, which is.
  const int kShift = 60;
  Frame F1 = synth::MakeFrame(synth::kMono, 0);
  Frame F2 = synth::MakeFrame(synth::kMono, kShift);

  std::vector<cv::Point2f> prevMatched(F1.mvKeysUn.size());
  for(size_t i = 0; i < F1.mvKeysUn.size(); ++i)
    prevMatched[i] = F1.mvKeysUn[i].pt;
  std::vector<int> matches12;
  ORB_SLAM2::ORBmatcher matcher(0.9f, true);
  ASSERT_GT(matcher.SearchForInitialization(F1, F2, prevMatched, matches12, 100), 100);

  Initializer init(F1, 1.0f, 200);
  cv::Mat R21, t21;
  std::vector<cv::Point3f> vP3D;
  std::vector<bool> vbTriangulated;

  ASSERT_TRUE(init.Initialize(F2, matches12, R21, t21, vP3D, vbTriangulated));

  ASSERT_EQ(R21.size(), cv::Size(3, 3));
  ASSERT_EQ(t21.size(), cv::Size(1, 3));
  EXPECT_EQ(vP3D.size(), matches12.size());
  EXPECT_EQ(vbTriangulated.size(), matches12.size());

  // Rotation is (near) identity: the camera only slid sideways.
  EXPECT_LT(cv::norm(R21 - cv::Mat::eye(3, 3, CV_32F)), 0.1);

  // Monocular has no scale, so only the direction is meaningful. Panning the image window
  // right means the camera moved +x in its own frame, i.e. the world slid to -x.
  const cv::Mat dir = t21 / cv::norm(t21);
  EXPECT_NEAR(std::abs(dir.at<float>(0)), 1.f, 0.15f) << "translation must be along x";
  EXPECT_NEAR(dir.at<float>(1), 0.f, 0.15f);
  EXPECT_NEAR(dir.at<float>(2), 0.f, 0.15f);

  int nTriangulated = 0;
  for(bool b : vbTriangulated)
    nTriangulated += b;
  EXPECT_GT(nTriangulated, 50);
}

// Initialize samples eight matches per RANSAC iteration, so it has to refuse anything
// smaller rather than index out of bounds.
TEST_F(InitializerTest, FailsWithTooFewMatches) {
  Frame F1 = synth::MakeFrame(synth::kMono, 0);
  Frame F2 = synth::MakeFrame(synth::kMono, 10);

  Initializer init(F1, 1.0f, 50);
  cv::Mat R21, t21;
  std::vector<cv::Point3f> vP3D;
  std::vector<bool> vbTriangulated;

  std::vector<int> matches12(F1.mvKeysUn.size(), -1);
  EXPECT_FALSE(init.Initialize(F2, matches12, R21, t21, vP3D, vbTriangulated)) << "no matches at all";

  for(int i = 0; i < 7; ++i)
    matches12[i] = i;
  EXPECT_FALSE(init.Initialize(F2, matches12, R21, t21, vP3D, vbTriangulated)) << "seven is one short of a sample";
}

TEST_F(InitializerTest, FailsOnDegenerateMatches) {
  Frame F1 = synth::MakeFrame(synth::kMono, 0);
  Frame F2 = synth::MakeFrame(synth::kMono, 10);

  // Enough matches to sample from, but every one of them points at the same keypoint, so
  // neither the homography nor the fundamental model can be scored.
  std::vector<int> matches12(F1.mvKeysUn.size(), -1);
  for(int i = 0; i < 60; ++i)
    matches12[i] = 0;

  Initializer init(F1, 1.0f, 50);
  cv::Mat R21, t21;
  std::vector<cv::Point3f> vP3D;
  std::vector<bool> vbTriangulated;

  EXPECT_FALSE(init.Initialize(F2, matches12, R21, t21, vP3D, vbTriangulated));
}

}    // namespace
