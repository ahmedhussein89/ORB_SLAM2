#include "include/Frame.h"

#include <gtest/gtest.h>

#include "include/Map.h"
#include "synthetic.h"

using ORB_SLAM2::Frame;
using ORB_SLAM2::Map;
using ORB_SLAM2::MapPoint;

namespace {

void ExpectSelfConsistent(const Frame& F) {
  EXPECT_GT(F.N, 100) << "the synthetic dot plane should give ORB plenty of corners";
  EXPECT_EQ(static_cast<int>(F.mvKeys.size()), F.N);
  EXPECT_EQ(static_cast<int>(F.mvKeysUn.size()), F.N);
  EXPECT_EQ(static_cast<int>(F.mvuRight.size()), F.N);
  EXPECT_EQ(static_cast<int>(F.mvDepth.size()), F.N);
  EXPECT_EQ(static_cast<int>(F.mvpMapPoints.size()), F.N);
  EXPECT_EQ(F.mDescriptors.rows, F.N);
  EXPECT_EQ(F.mDescriptors.cols, 32);
}

TEST(Frame, MonocularConstruction) {
  Frame F = synth::MakeFrame(synth::kMono);
  ExpectSelfConsistent(F);
  // Monocular has no stereo information at all.
  for(int i = 0; i < F.N; ++i) {
    EXPECT_LT(F.mvuRight[i], 0.f);
    EXPECT_LT(F.mvDepth[i], 0.f);
  }
}

TEST(Frame, RgbdDepthComesFromTheDepthMap) {
  Frame F = synth::MakeFrame(synth::kRgbd);
  ExpectSelfConsistent(F);

  int withDepth = 0;
  for(int i = 0; i < F.N; ++i) {
    if(F.mvDepth[i] <= 0.f)
      continue;
    ++withDepth;
    EXPECT_NEAR(F.mvDepth[i], synth::PlaneDepth(), 1e-4);
    // u_right = u_left - bf/z
    EXPECT_NEAR(F.mvuRight[i], F.mvKeys[i].pt.x - synth::Calib().bf / synth::PlaneDepth(), 1e-3);
  }
  EXPECT_GT(withDepth, F.N / 2) << "a full depth map should give most keypoints a depth";
}

TEST(Frame, StereoMatchingRecoversThePlaneDisparity) {
  Frame F = synth::MakeFrame(synth::kStereo);
  ExpectSelfConsistent(F);
  EXPECT_FALSE(F.mvKeysRight.empty());

  // ComputeStereoMatches refines each disparity with a parabola fit on a sliding-window SAD.
  // On a repetitive dot pattern individual matches wander by a pixel or two, so check the
  // distribution: the median must be the true depth and the bulk must be close to it. That
  // still fails loudly on a wrong baseline, a sign flip, or no matching at all.
  std::vector<float> depths;
  for(int i = 0; i < F.N; ++i)
    if(F.mvDepth[i] > 0.f)
      depths.push_back(F.mvDepth[i]);

  ASSERT_GT(static_cast<int>(depths.size()), F.N / 4) << "the right image is the left shifted by a constant disparity";
  std::sort(depths.begin(), depths.end());
  EXPECT_NEAR(depths[depths.size() / 2], synth::PlaneDepth(), 0.05f);

  int within5pct = 0;
  for(float d : depths)
    within5pct += std::abs(d - synth::PlaneDepth()) < 0.05f * synth::PlaneDepth();
  EXPECT_GT(within5pct, static_cast<int>(depths.size()) * 3 / 4);

  // mvuRight must be consistent with mvDepth: u_right = u_left - bf/z.
  for(int i = 0; i < F.N; ++i)
    if(F.mvDepth[i] > 0.f)
      EXPECT_NEAR(F.mvuRight[i], F.mvKeys[i].pt.x - synth::Calib().bf / F.mvDepth[i], 1e-2);
}

TEST(Frame, PoseMatricesAreConsistent) {
  Frame F = synth::MakeFrame(synth::kMono);
  F.SetPose(synth::PoseAt(1.f, 2.f, 3.f));

  const cv::Mat Ow = F.GetCameraCenter();
  EXPECT_NEAR(Ow.at<float>(0), 1.f, 1e-5);
  EXPECT_NEAR(Ow.at<float>(1), 2.f, 1e-5);
  EXPECT_NEAR(Ow.at<float>(2), 3.f, 1e-5);
  EXPECT_LT(cv::norm(F.GetRotationInverse() - F.mTcw.rowRange(0, 3).colRange(0, 3).t()), 1e-5);
}

TEST(Frame, PosInGridAgreesWithGetFeaturesInArea) {
  Frame F = synth::MakeFrame(synth::kMono);

  int posX = -1, posY = -1;
  ASSERT_TRUE(F.PosInGrid(F.mvKeysUn[0], posX, posY));
  EXPECT_GE(posX, 0);
  EXPECT_LT(posX, FRAME_GRID_COLS);
  EXPECT_GE(posY, 0);
  EXPECT_LT(posY, FRAME_GRID_ROWS);

  cv::KeyPoint outside = F.mvKeysUn[0];
  outside.pt = cv::Point2f(-100.f, -100.f);
  EXPECT_FALSE(F.PosInGrid(outside, posX, posY));

  const float x = 320.f, y = 240.f, r = 20.f;
  const std::vector<size_t> got = F.GetFeaturesInArea(x, y, r);
  std::set<size_t> gotSet(got.begin(), got.end());

  int expected = 0;
  for(int i = 0; i < F.N; ++i) {
    const cv::KeyPoint& kp = F.mvKeysUn[i];
    if(std::abs(kp.pt.x - x) >= r || std::abs(kp.pt.y - y) >= r)
      continue;
    ++expected;
    EXPECT_EQ(gotSet.count(i), 1u) << "missed keypoint " << i;
  }
  ASSERT_GT(expected, 0);
  for(size_t i : got) {
    EXPECT_LT(std::abs(F.mvKeysUn[i].pt.x - x), r + 1e-3f);
    EXPECT_LT(std::abs(F.mvKeysUn[i].pt.y - y), r + 1e-3f);
  }
}

TEST(Frame, GetFeaturesInAreaRespectsTheLevelRange) {
  Frame F = synth::MakeFrame(synth::kMono);
  const std::vector<size_t> level0 = F.GetFeaturesInArea(320.f, 240.f, 60.f, 0, 0);
  for(size_t i : level0)
    EXPECT_EQ(F.mvKeysUn[i].octave, 0);
  EXPECT_LE(level0.size(), F.GetFeaturesInArea(320.f, 240.f, 60.f).size());
}

TEST(Frame, IsInFrustum) {
  Map map;
  Frame F = synth::MakeFrame(synth::kMono);
  F.SetPose(cv::Mat::eye(4, 4, CV_32F));

  // MapPoint(Pos, Map, Frame, idx) fills the normal and the scale bounds from the frame.
  MapPoint infront(synth::Point3(0.f, 0.f, synth::PlaneDepth()), &map, &F, 0);
  EXPECT_TRUE(F.isInFrustum(&infront, 0.5f));
  EXPECT_GT(infront.mTrackProjX, 0.f);
  EXPECT_LT(infront.mTrackProjX, static_cast<float>(synth::Width()));

  MapPoint behind(synth::Point3(0.f, 0.f, -synth::PlaneDepth()), &map, &F, 0);
  EXPECT_FALSE(F.isInFrustum(&behind, 0.5f));

  // Far off to the side: projects outside the image.
  MapPoint offImage(synth::Point3(100.f, 0.f, synth::PlaneDepth()), &map, &F, 0);
  EXPECT_FALSE(F.isInFrustum(&offImage, 0.5f));
}

TEST(Frame, UnprojectStereoNeedsDepth) {
  Frame F = synth::MakeFrame(synth::kRgbd);
  F.SetPose(cv::Mat::eye(4, 4, CV_32F));

  int idx = -1;
  for(int i = 0; i < F.N && idx < 0; ++i)
    if(F.mvDepth[i] > 0.f)
      idx = i;
  ASSERT_GE(idx, 0);

  const cv::Mat x3D = F.UnprojectStereo(idx);
  ASSERT_FALSE(x3D.empty());
  EXPECT_NEAR(x3D.at<float>(2), synth::PlaneDepth(), 1e-4);
  // Re-projecting it must land back on the keypoint.
  const cv::Point2f uv = synth::Project(cv::Mat::eye(4, 4, CV_32F), x3D);
  EXPECT_NEAR(uv.x, F.mvKeysUn[idx].pt.x, 1e-2);
  EXPECT_NEAR(uv.y, F.mvKeysUn[idx].pt.y, 1e-2);

  Frame mono = synth::MakeFrame(synth::kMono);
  mono.SetPose(cv::Mat::eye(4, 4, CV_32F));
  EXPECT_TRUE(mono.UnprojectStereo(0).empty()) << "no depth means no 3D point";
}

TEST(Frame, ComputeBoWFillsTheVectors) {
  Frame F = synth::MakeFrame(synth::kMono);
  EXPECT_TRUE(F.mBowVec.empty());
  F.ComputeBoW();
  EXPECT_FALSE(F.mBowVec.empty());
  EXPECT_FALSE(F.mFeatVec.empty());
}

TEST(Frame, IdsIncreaseAndCopiesShareThem) {
  Frame a = synth::MakeFrame(synth::kMono);
  Frame b = synth::MakeFrame(synth::kMono);
  EXPECT_GT(b.mnId, a.mnId);

  Frame copy(a);
  EXPECT_EQ(copy.mnId, a.mnId);
  EXPECT_EQ(copy.N, a.N);
}

TEST(Frame, ScalePyramidInfoIsGeometric) {
  Frame F = synth::MakeFrame(synth::kMono);
  ASSERT_EQ(static_cast<int>(F.mvScaleFactors.size()), F.mnScaleLevels);
  EXPECT_NEAR(F.mvScaleFactors[0], 1.f, 1e-6);
  for(int i = 1; i < F.mnScaleLevels; ++i) {
    EXPECT_NEAR(F.mvScaleFactors[i], F.mvScaleFactors[i - 1] * F.mfScaleFactor, 1e-4);
    EXPECT_NEAR(F.mvInvScaleFactors[i] * F.mvScaleFactors[i], 1.f, 1e-4);
  }
}

}    // namespace
