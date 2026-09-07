#include "include/PnPsolver.h"

#include <gtest/gtest.h>

#include "include/Map.h"
#include "synthetic.h"

using ORB_SLAM2::Frame;
using ORB_SLAM2::Map;
using ORB_SLAM2::MapPoint;
using ORB_SLAM2::PnPsolver;

namespace {

// EPnP inside a RANSAC loop: seed it, then assert the recovered pose, not exact numbers.
struct PnPsolverTest : ::testing::Test {
  Map map;
  Frame F = synth::MakeFrame(synth::kRgbd);
  cv::Mat Tcw_gt;

  void SetUp() override {
    srand(4242);

    // Ground truth: the camera sits at (0.3, -0.2, -1.0) looking down +z.
    Tcw_gt = synth::PoseAt(0.3f, -0.2f, -1.0f);
    F.SetPose(cv::Mat::eye(4, 4, CV_32F));
  }

  // MapPoints placed so that, viewed from Tcw_gt, they project onto F's keypoints.
  std::vector<MapPoint*> BuildMatches(int stride = 1) {
    const cv::Mat Twc_gt = Tcw_gt.inv();
    std::vector<MapPoint*> matches(F.N, static_cast<MapPoint*>(nullptr));
    int n = 0;
    for(int i = 0; i < F.N; ++i) {
      if(F.mvDepth[i] <= 0.f || i % stride)
        continue;
      const cv::Mat Xc = F.UnprojectStereo(i);    // F is at the identity, so this is camera-frame
      const cv::Mat Xw = Twc_gt.rowRange(0, 3).colRange(0, 3) * Xc + Twc_gt.rowRange(0, 3).col(3);
      matches[i] = new MapPoint(Xw, &map, &F, i);
      ++n;
    }
    EXPECT_GT(n, 0);
    return matches;
  }
};

TEST_F(PnPsolverTest, RecoversTheGroundTruthPose) {
  const std::vector<MapPoint*> matches = BuildMatches();

  PnPsolver solver(F, matches);
  solver.SetRansacParameters(0.99, 10, 300, 4, 0.5, 5.991);

  bool bNoMore = false;
  std::vector<bool> vbInliers;
  int nInliers = 0;
  const cv::Mat Tcw = solver.iterate(300, bNoMore, vbInliers, nInliers);

  ASSERT_FALSE(Tcw.empty()) << "EPnP failed on noise-free correspondences";
  EXPECT_GT(nInliers, 10);
  EXPECT_LT(cv::norm(Tcw - Tcw_gt), 0.05) << "recovered\n" << Tcw << "\nexpected\n" << Tcw_gt;
}

TEST_F(PnPsolverTest, FindWrapsIterate) {
  const std::vector<MapPoint*> matches = BuildMatches();

  PnPsolver solver(F, matches);
  solver.SetRansacParameters(0.99, 10, 300, 4, 0.5, 5.991);

  std::vector<bool> vbInliers;
  int nInliers = 0;
  const cv::Mat Tcw = solver.find(vbInliers, nInliers);

  ASSERT_FALSE(Tcw.empty());
  EXPECT_EQ(vbInliers.size(), matches.size());
  EXPECT_LT(cv::norm(Tcw.rowRange(0, 3).col(3) - Tcw_gt.rowRange(0, 3).col(3)), 0.05);
}

TEST_F(PnPsolverTest, GivesUpWhenThereAreTooFewCorrespondences) {
  // Three points is below any usable minimum inlier count.
  std::vector<MapPoint*> matches = BuildMatches();
  int kept = 0;
  for(size_t i = 0; i < matches.size(); ++i) {
    if(!matches[i])
      continue;
    if(kept < 3)
      ++kept;
    else
      matches[i] = nullptr;
  }

  PnPsolver solver(F, matches);
  solver.SetRansacParameters(0.99, 10, 300, 4, 0.5, 5.991);

  bool bNoMore = false;
  std::vector<bool> vbInliers;
  int nInliers = 0;
  const cv::Mat Tcw = solver.iterate(50, bNoMore, vbInliers, nInliers);

  EXPECT_TRUE(Tcw.empty());
  EXPECT_TRUE(bNoMore);
  EXPECT_EQ(nInliers, 0);
}

}    // namespace
