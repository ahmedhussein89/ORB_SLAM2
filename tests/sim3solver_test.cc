#include "include/Sim3Solver.h"

#include <gtest/gtest.h>

#include "include/KeyFrameDatabase.h"
#include "include/Map.h"
#include "synthetic.h"

using ORB_SLAM2::KeyFrame;
using ORB_SLAM2::Map;
using ORB_SLAM2::MapPoint;
using ORB_SLAM2::Sim3Solver;

namespace {

// Sim3Solver never looks at the keypoints themselves -- it compares the projections of the two
// MapPoint clouds -- so the correspondences can be synthesised outright. Both keyframes sit at
// the identity, so world coordinates are camera coordinates, and the solver's T12 must satisfy
// X1 = s12 * R12 * X2 + t12.
//
// The default ground truth carries a real rotation; PureTranslationIsNotNaN covers the
// degenerate case where the rotation is exactly the identity.
struct Sim3SolverTest : ::testing::Test {
  Map map;
  ORB_SLAM2::KeyFrameDatabase kfdb{synth::TinyVocab()};
  KeyFrame *pKF1 = nullptr, *pKF2 = nullptr;
  std::vector<MapPoint*> matched12;

  cv::Mat R_gt;    // R12
  cv::Mat t_gt;    // t12
  float s_gt = 1.f;

  void SetUp() override {
    srand(999);
    cv::Mat rvec = (cv::Mat_<float>(3, 1) << 0.05f, 0.20f, -0.03f);
    cv::Rodrigues(rvec, R_gt);
    t_gt = (cv::Mat_<float>(3, 1) << 0.30f, -0.10f, 0.15f);
  }

  void Build(float scale, int nPoints = 60) {
    s_gt = scale;
    ORB_SLAM2::Frame F1 = synth::MakeFrame(synth::kRgbd, 0);
    ORB_SLAM2::Frame F2 = synth::MakeFrame(synth::kRgbd, 10);
    pKF1 = synth::MakeKeyFrame(F1, &map, &kfdb);
    pKF2 = synth::MakeKeyFrame(F2, &map, &kfdb);
    ASSERT_GT(std::min(pKF1->N, pKF2->N), nPoints);

    matched12.assign(pKF1->N, static_cast<MapPoint*>(nullptr));
    cv::RNG rng(7);
    const cv::Mat Rt = R_gt.t();
    for(int i = 0; i < nPoints; ++i) {
      // A spread-out, non-degenerate cloud in front of camera 1.
      const cv::Mat X1 = synth::Point3(rng.uniform(-1.f, 1.f), rng.uniform(-1.f, 1.f), rng.uniform(2.f, 5.f));
      const cv::Mat X2 = Rt * (X1 - t_gt) / s_gt;

      MapPoint* pMP1 = new MapPoint(X1, pKF1, &map);
      pMP1->AddObservation(pKF1, i);
      pKF1->AddMapPoint(pMP1, i);

      MapPoint* pMP2 = new MapPoint(X2, pKF2, &map);
      pMP2->AddObservation(pKF2, i);
      pKF2->AddMapPoint(pMP2, i);

      matched12[i] = pMP2;
    }
  }
};

TEST_F(Sim3SolverTest, FixedScaleRecoversRotationAndTranslation) {
  Build(1.0f);

  Sim3Solver solver(pKF1, pKF2, matched12, true);
  solver.SetRansacParameters(0.99, 20, 300);

  std::vector<bool> vbInliers;
  int nInliers = 0;
  const cv::Mat T12 = solver.find(vbInliers, nInliers);

  ASSERT_FALSE(T12.empty()) << "RANSAC found no consistent Sim3 on noise-free correspondences";
  EXPECT_GT(nInliers, 20);
  EXPECT_EQ(vbInliers.size(), matched12.size());

  EXPECT_NEAR(solver.GetEstimatedScale(), 1.f, 1e-4) << "bFixScale must pin the scale to 1";
  EXPECT_LT(cv::norm(solver.GetEstimatedRotation() - R_gt), 1e-3);
  EXPECT_LT(cv::norm(solver.GetEstimatedTranslation().reshape(1, 3) - t_gt), 1e-3);

  // T12 = [s*R | t].
  EXPECT_LT(cv::norm(T12.rowRange(0, 3).colRange(0, 3) - R_gt), 1e-3);
  EXPECT_LT(cv::norm(T12.rowRange(0, 3).col(3) - t_gt), 1e-3);
}

TEST_F(Sim3SolverTest, FreeScaleRecoversTheScale) {
  Build(2.0f);    // camera-2 points shrunk by 2, so T12 must scale them back up

  Sim3Solver solver(pKF1, pKF2, matched12, false);
  solver.SetRansacParameters(0.99, 20, 300);

  std::vector<bool> vbInliers;
  int nInliers = 0;
  const cv::Mat T12 = solver.find(vbInliers, nInliers);

  ASSERT_FALSE(T12.empty());
  EXPECT_GT(nInliers, 20);
  EXPECT_NEAR(solver.GetEstimatedScale(), 2.f, 1e-2);
  EXPECT_LT(cv::norm(solver.GetEstimatedRotation() - R_gt), 1e-3);
  EXPECT_LT(cv::norm(T12.rowRange(0, 3).colRange(0, 3) - 2.f * R_gt), 1e-2);
}

TEST_F(Sim3SolverTest, RejectsOutliers) {
  Build(1.0f, 60);

  // Move a quarter of the camera-2 points somewhere else entirely.
  int spoiled = 0;
  for(size_t i = 0; i < matched12.size() && spoiled < 15; ++i) {
    if(!matched12[i])
      continue;
    matched12[i]->SetWorldPos(synth::Point3(5.f + spoiled, -4.f, 9.f));
    ++spoiled;
  }

  Sim3Solver solver(pKF1, pKF2, matched12, true);
  solver.SetRansacParameters(0.99, 20, 500);

  std::vector<bool> vbInliers;
  int nInliers = 0;
  const cv::Mat T12 = solver.find(vbInliers, nInliers);

  ASSERT_FALSE(T12.empty());
  EXPECT_GE(nInliers, 40);
  EXPECT_LE(nInliers, 45) << "the 15 displaced points must not be counted as inliers";
  EXPECT_LT(cv::norm(solver.GetEstimatedRotation() - R_gt), 1e-2);
}

// ComputeSim3 recovers the angle-axis vector as `2*ang*vec/norm(vec)`, which is 0/0 when the
// true rotation is exactly the identity -- that used to hand back an all-NaN transform.
TEST_F(Sim3SolverTest, PureTranslationIsNotNaN) {
  R_gt = cv::Mat::eye(3, 3, CV_32F);
  Build(1.0f);

  Sim3Solver solver(pKF1, pKF2, matched12, true);
  solver.SetRansacParameters(0.99, 20, 300);

  std::vector<bool> vbInliers;
  int nInliers = 0;
  const cv::Mat T12 = solver.find(vbInliers, nInliers);

  ASSERT_FALSE(T12.empty());
  ASSERT_TRUE(cv::checkRange(T12, true, nullptr, -1e9, 1e9)) << "NaN in\n" << T12;
  EXPECT_GT(nInliers, 20);
  EXPECT_LT(cv::norm(solver.GetEstimatedRotation() - cv::Mat::eye(3, 3, CV_32F)), 1e-3);
  EXPECT_LT(cv::norm(solver.GetEstimatedTranslation().reshape(1, 3) - t_gt), 1e-3);
}

TEST_F(Sim3SolverTest, GivesUpWithTooFewCorrespondences) {
  Build(1.0f, 5);

  Sim3Solver solver(pKF1, pKF2, matched12, true);
  solver.SetRansacParameters(0.99, 20, 100);

  bool bNoMore = false;
  std::vector<bool> vbInliers;
  int nInliers = 0;
  EXPECT_TRUE(solver.iterate(20, bNoMore, vbInliers, nInliers).empty());
  EXPECT_TRUE(bNoMore);
  EXPECT_EQ(nInliers, 0);
}

}    // namespace
