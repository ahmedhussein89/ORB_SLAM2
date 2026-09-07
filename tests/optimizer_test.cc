#include "include/Optimizer.h"

#include <gtest/gtest.h>

#include "include/KeyFrameDatabase.h"
#include "include/LoopClosing.h"
#include "include/Map.h"
#include "synthetic.h"

using ORB_SLAM2::Frame;
using ORB_SLAM2::KeyFrame;
using ORB_SLAM2::Map;
using ORB_SLAM2::MapPoint;
using ORB_SLAM2::Optimizer;

namespace {

bool IsFinite(const cv::Mat& m) {
  return cv::checkRange(m, true, nullptr, -1e9, 1e9);
}

// Every keypoint of an RGB-D frame gets a MapPoint at the position implied by `poseGt`, so the
// reprojection residual is zero at the ground truth and any optimiser that moves away from it
// is wrong.
int SeedFrame(Frame& F, Map& map, const cv::Mat& poseGt) {
  Frame ref = F;
  ref.SetPose(cv::Mat::eye(4, 4, CV_32F));
  F.SetPose(poseGt.clone());    // MapPoint's frame constructor needs a camera centre
  const cv::Mat Twc = poseGt.inv();

  int n = 0;
  for(int i = 0; i < F.N; ++i) {
    if(F.mvDepth[i] <= 0.f)
      continue;
    const cv::Mat Xc = ref.UnprojectStereo(i);
    const cv::Mat Xw = Twc.rowRange(0, 3).colRange(0, 3) * Xc + Twc.rowRange(0, 3).col(3);
    MapPoint* pMP = new MapPoint(Xw, &map, &F, i);
    map.AddMapPoint(pMP);
    F.mvpMapPoints[i] = pMP;
    ++n;
  }
  return n;
}

TEST(Optimizer, PoseOptimizationConvergesToTheGroundTruth) {
  Map map;
  Frame F = synth::MakeFrame(synth::kRgbd);
  const cv::Mat poseGt = synth::PoseAt(0.2f, -0.1f, -0.5f);
  ASSERT_GT(SeedFrame(F, map, poseGt), 100);

  // Start 5 cm and a couple of degrees off.
  cv::Mat perturbed = poseGt.clone();
  perturbed.at<float>(0, 3) += 0.05f;
  perturbed.at<float>(1, 3) -= 0.03f;
  cv::Mat dR, rvec = (cv::Mat_<float>(3, 1) << 0.01f, 0.02f, 0.f);
  cv::Rodrigues(rvec, dR);
  cv::Mat Rperturbed = dR * poseGt.rowRange(0, 3).colRange(0, 3);
  Rperturbed.copyTo(perturbed.rowRange(0, 3).colRange(0, 3));
  F.SetPose(perturbed);
  ASSERT_GT(cv::norm(F.mTcw - poseGt), 0.05);

  const int nInliers = Optimizer::PoseOptimization(&F);

  EXPECT_GT(nInliers, 100);
  EXPECT_LT(cv::norm(F.mTcw - poseGt), 0.01) << "recovered\n" << F.mTcw << "\nexpected\n" << poseGt;
}

TEST(Optimizer, PoseOptimizationFlagsGrossOutliers) {
  Map map;
  Frame F = synth::MakeFrame(synth::kRgbd);
  const cv::Mat poseGt = cv::Mat::eye(4, 4, CV_32F);
  ASSERT_GT(SeedFrame(F, map, poseGt), 100);

  // Displace a handful of points far from where they were observed.
  std::vector<int> spoiled;
  for(int i = 0; i < F.N && static_cast<int>(spoiled.size()) < 10; ++i) {
    if(!F.mvpMapPoints[i])
      continue;
    F.mvpMapPoints[i]->SetWorldPos(synth::Point3(3.f + spoiled.size(), 3.f, 4.f));
    spoiled.push_back(i);
  }
  ASSERT_EQ(spoiled.size(), 10u);

  F.SetPose(poseGt.clone());
  Optimizer::PoseOptimization(&F);

  int flagged = 0;
  for(int i : spoiled)
    flagged += F.mvbOutlier[i];
  EXPECT_GE(flagged, 8) << "displaced points must be rejected as outliers";
  EXPECT_LT(cv::norm(F.mTcw - poseGt), 0.01) << "outliers must not drag the pose";
}

// A two-keyframe map of the same plane, wired up enough for the graph optimisers to run.
struct GraphFixture {
  Map map;
  ORB_SLAM2::KeyFrameDatabase kfdb{synth::TinyVocab()};
  std::vector<KeyFrame*> kfs;
  std::vector<MapPoint*> mps;

  // All keyframes are built from the SAME frame at the SAME pose, so every observation is
  // exactly consistent and a correct optimiser has nothing to move. A map assembled from
  // different views would need ground-truth poses to compare against, which is what
  // tools/smoke_test.sh does end to end.
  void Build(int nKeyFrames) {
    Frame F = synth::MakeFrame(synth::kRgbd, 0);
    for(int k = 0; k < nKeyFrames; ++k) {
      KeyFrame* pKF = synth::MakeKeyFrame(F, &map, &kfdb);
      pKF->SetPose(cv::Mat::eye(4, 4, CV_32F));
      map.AddKeyFrame(pKF);
      if(k)
        pKF->ChangeParent(kfs.front());
      kfs.push_back(pKF);
    }
    map.mvpKeyFrameOrigins.push_back(kfs.front());

    // Points triangulated from the first keyframe, observed by all of them at the same index.
    const int nCommon = kfs.front()->N;
    for(int i = 0; i < nCommon; ++i) {
      const cv::Mat X = kfs.front()->UnprojectStereo(i);
      if(X.empty())
        continue;
      MapPoint* pMP = new MapPoint(X, kfs.front(), &map);
      for(KeyFrame* pKF : kfs) {
        if(i >= pKF->N || pKF->mvuRight[i] < 0)
          continue;
        pMP->AddObservation(pKF, i);
        pKF->AddMapPoint(pMP, i);
      }
      if(pMP->Observations() < 2)
        continue;
      pMP->ComputeDistinctiveDescriptors();
      pMP->UpdateNormalAndDepth();
      map.AddMapPoint(pMP);
      mps.push_back(pMP);
    }
    for(KeyFrame* pKF : kfs)
      pKF->UpdateConnections();
  }
};

TEST(Optimizer, BundleAdjustmentLeavesAConsistentMapAlone) {
  GraphFixture g;
  g.Build(3);
  ASSERT_GT(g.mps.size(), 100u);

  std::vector<cv::Mat> before;
  for(KeyFrame* pKF : g.kfs)
    before.push_back(pKF->GetPose().clone());

  Optimizer::BundleAdjustment(g.kfs, g.mps, 10, nullptr, 0, true);

  for(size_t i = 0; i < g.kfs.size(); ++i) {
    ASSERT_TRUE(IsFinite(g.kfs[i]->GetPose()));
    EXPECT_LT(cv::norm(g.kfs[i]->GetPose() - before[i]), 0.05) << "keyframe " << i << " moved";
  }
  for(MapPoint* pMP : g.mps)
    ASSERT_TRUE(IsFinite(pMP->GetWorldPos()));
}

TEST(Optimizer, GlobalBundleAdjustmentWritesTheGlobalBAFields) {
  GraphFixture g;
  g.Build(2);

  Optimizer::GlobalBundleAdjustemnt(&g.map, 5, nullptr, 42, true);

  for(KeyFrame* pKF : g.kfs) {
    EXPECT_EQ(pKF->mnBAGlobalForKF, 42u);
    ASSERT_FALSE(pKF->mTcwGBA.empty());
    EXPECT_TRUE(IsFinite(pKF->mTcwGBA));
  }
  for(MapPoint* pMP : g.mps) {
    EXPECT_EQ(pMP->mnBAGlobalForKF, 42u);
    EXPECT_TRUE(IsFinite(pMP->mPosGBA));
  }
}

TEST(Optimizer, LocalBundleAdjustmentRuns) {
  GraphFixture g;
  g.Build(3);

  const cv::Mat before = g.kfs.back()->GetPose().clone();
  bool stop = false;
  Optimizer::LocalBundleAdjustment(g.kfs.back(), &stop, &g.map);

  ASSERT_TRUE(IsFinite(g.kfs.back()->GetPose()));
  EXPECT_LT(cv::norm(g.kfs.back()->GetPose() - before), 0.05);
}

TEST(Optimizer, LocalBundleAdjustmentHonoursTheStopFlag) {
  GraphFixture g;
  g.Build(3);

  bool stop = true;    // already asked to stop before it starts
  Optimizer::LocalBundleAdjustment(g.kfs.back(), &stop, &g.map);

  EXPECT_TRUE(IsFinite(g.kfs.back()->GetPose()));
}

TEST(Optimizer, OptimizeEssentialGraphRuns) {
  GraphFixture g;
  g.Build(3);

  ORB_SLAM2::LoopClosing::KeyFrameAndPose nonCorrected, corrected;
  std::map<KeyFrame*, std::set<KeyFrame*>> loopConnections;
  loopConnections[g.kfs.back()].insert(g.kfs.front());

  Optimizer::OptimizeEssentialGraph(&g.map, g.kfs.front(), g.kfs.back(), nonCorrected, corrected, loopConnections, true);

  for(KeyFrame* pKF : g.kfs)
    EXPECT_TRUE(IsFinite(pKF->GetPose()));
  for(MapPoint* pMP : g.mps)
    EXPECT_TRUE(IsFinite(pMP->GetWorldPos()));
}

TEST(Optimizer, OptimizeSim3OnAnIdenticalPair) {
  GraphFixture g;
  g.Build(2);

  // Both keyframes see the same points at the same indices, so the Sim3 between them is the
  // identity and OptimizeSim3 should keep it there.
  std::vector<MapPoint*> matches1(g.kfs[0]->N, static_cast<MapPoint*>(nullptr));
  int n = 0;
  for(int i = 0; i < g.kfs[0]->N; ++i) {
    MapPoint* pMP = g.kfs[1]->GetMapPoint(i);
    if(!pMP || pMP->isBad() || !g.kfs[0]->GetMapPoint(i))
      continue;
    matches1[i] = pMP;
    ++n;
  }
  ASSERT_GT(n, 50);

  g2o::Sim3 S12(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero(), 1.0);
  const int nInliers = Optimizer::OptimizeSim3(g.kfs[0], g.kfs[1], matches1, S12, 10.f, true);

  EXPECT_GT(nInliers, 20);
  EXPECT_NEAR(S12.scale(), 1.0, 1e-3);
  EXPECT_LT(S12.translation().norm(), 0.05);
}

}    // namespace
