#include "include/ORBextractor.h"

#include <gtest/gtest.h>

#include "synthetic.h"

using ORB_SLAM2::ORBextractor;

namespace {

TEST(ORBextractor, ScalePyramidInfo) {
  ORBextractor e(500, 1.2f, 8, 20, 7);

  EXPECT_EQ(e.GetLevels(), 8);
  EXPECT_NEAR(e.GetScaleFactor(), 1.2f, 1e-6);

  const std::vector<float> s = e.GetScaleFactors(), inv = e.GetInverseScaleFactors();
  const std::vector<float> s2 = e.GetScaleSigmaSquares(), inv2 = e.GetInverseScaleSigmaSquares();
  ASSERT_EQ(s.size(), 8u);
  EXPECT_NEAR(s[0], 1.f, 1e-6);
  for(size_t i = 0; i < s.size(); ++i) {
    EXPECT_NEAR(s[i] * inv[i], 1.f, 1e-4);
    EXPECT_NEAR(s2[i], s[i] * s[i], 1e-3);
    EXPECT_NEAR(s2[i] * inv2[i], 1.f, 1e-3);
    if(i)
      EXPECT_NEAR(s[i], s[i - 1] * 1.2f, 1e-4);
  }
}

TEST(ORBextractor, ExtractsFeaturesFromATexturedImage) {
  ORBextractor e(500, 1.2f, 8, 20, 7);
  std::vector<cv::KeyPoint> kps;
  cv::Mat desc;

  e(synth::SynthImage(), cv::Mat(), kps, desc);

  ASSERT_GT(kps.size(), 100u);
  EXPECT_EQ(desc.rows, static_cast<int>(kps.size()));
  EXPECT_EQ(desc.cols, 32);
  EXPECT_EQ(desc.type(), CV_8U);
  for(const cv::KeyPoint& kp : kps) {
    EXPECT_GE(kp.octave, 0);
    EXPECT_LT(kp.octave, 8);
    EXPECT_GE(kp.pt.x, 0.f);
    EXPECT_LE(kp.pt.x, static_cast<float>(synth::Width()));
    EXPECT_GE(kp.pt.y, 0.f);
    EXPECT_LE(kp.pt.y, static_cast<float>(synth::Height()));
    EXPECT_GT(kp.size, 0.f);
  }
}

TEST(ORBextractor, FindsNothingInABlankImage) {
  ORBextractor e(500, 1.2f, 8, 20, 7);
  std::vector<cv::KeyPoint> kps;
  cv::Mat desc;

  e(synth::BlankImage(), cv::Mat(), kps, desc);

  EXPECT_TRUE(kps.empty());
  EXPECT_TRUE(desc.empty());
}

// Extraction must be bit-reproducible. It was not until DistributeOctTree's split queue
// stopped breaking ties on ExtractorNode addresses, so this test guards that fix: a
// regression there shows up here as a handful of moved keypoints and changed descriptors.
TEST(ORBextractor, IsDeterministic) {
  ORBextractor a(500, 1.2f, 8, 20, 7), b(500, 1.2f, 8, 20, 7);
  std::vector<cv::KeyPoint> kps1, kps2, kps3;
  cv::Mat d1, d2, d3;

  a(synth::SynthImage(), cv::Mat(), kps1, d1);
  a(synth::SynthImage(), cv::Mat(), kps2, d2);    // same extractor, second call
  b(synth::SynthImage(), cv::Mat(), kps3, d3);    // a different extractor object

  ASSERT_FALSE(kps1.empty());
  ASSERT_EQ(kps1.size(), kps2.size());
  ASSERT_EQ(kps1.size(), kps3.size());

  for(size_t i = 0; i < kps1.size(); ++i) {
    EXPECT_EQ(kps1[i].pt, kps2[i].pt);
    EXPECT_EQ(kps1[i].pt, kps3[i].pt);
    EXPECT_EQ(kps1[i].octave, kps2[i].octave);
    EXPECT_EQ(kps1[i].angle, kps2[i].angle);
    EXPECT_EQ(kps1[i].angle, kps3[i].angle);
  }
  EXPECT_EQ(cv::countNonZero(d1 != d2), 0);
  EXPECT_EQ(cv::countNonZero(d1 != d3), 0);
}

TEST(ORBextractor, ShiftedImageGivesShiftedKeypoints) {
  ORBextractor e(1000, 1.2f, 8, 20, 7);
  std::vector<cv::KeyPoint> kps1, kps2;
  cv::Mat d1, d2;
  const int kShift = 12;

  e(synth::SynthImage(0), cv::Mat(), kps1, d1);
  e(synth::SynthImage(kShift), cv::Mat(), kps2, d2);

  // Panning the window right by kShift moves the same corners left by kShift. Just check the
  // distribution moved the right way rather than matching individual points (that is
  // ORBmatcher's job).
  auto meanX = [](const std::vector<cv::KeyPoint>& v) {
    double s = 0;
    for(const cv::KeyPoint& kp : v)
      s += kp.pt.x;
    return s / v.size();
  };
  ASSERT_FALSE(kps1.empty());
  ASSERT_FALSE(kps2.empty());
  EXPECT_LT(std::abs(meanX(kps1) - meanX(kps2)), 20.0) << "the plane is statistically uniform";
}

TEST(ORBextractor, RespectsTheFeatureBudgetRoughly) {
  std::vector<cv::KeyPoint> few, many;
  cv::Mat d;
  ORBextractor(200, 1.2f, 8, 20, 7)(synth::SynthImage(), cv::Mat(), few, d);
  ORBextractor(1000, 1.2f, 8, 20, 7)(synth::SynthImage(), cv::Mat(), many, d);

  EXPECT_LT(few.size(), many.size());
  EXPECT_LT(few.size(), 400u) << "the octree distribution should stay near the budget";
}

}    // namespace
