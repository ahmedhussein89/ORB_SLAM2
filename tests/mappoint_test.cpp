#include "orbslam/core/MapPoint.hpp"

#include <gtest/gtest.h>

#include "synthetic.hpp"

using ORB_SLAM2::KeyFrame;
using ORB_SLAM2::Map;
using ORB_SLAM2::MapPoint;

namespace {

// RGB-D keyframes, so mvuRight >= 0 and observations count double -- that is what makes
// EraseObservation's "nObs <= 2 means discard" threshold reachable with few keyframes.
struct MapPointTest : ::testing::Test {
  Map map;
  std::vector<KeyFrame*> kfs;

  KeyFrame* NewKeyFrame(int shift = 0) {
    ORB_SLAM2::Frame F = synth::MakeFrame(synth::kRgbd, shift);
    KeyFrame* pKF = synth::MakeKeyFrame(F, &map);
    map.AddKeyFrame(pKF);
    kfs.push_back(pKF);
    return pKF;
  }

  MapPoint* NewMapPoint(KeyFrame* pRef) {
    MapPoint* pMP = new MapPoint(synth::Point3(0.f, 0.f, synth::PlaneDepth()), pRef, &map);
    map.AddMapPoint(pMP);
    return pMP;
  }
};

TEST_F(MapPointTest, WorldPosRoundTrip) {
  MapPoint* pMP = NewMapPoint(NewKeyFrame());
  const cv::Mat pos = synth::Point3(1.f, -2.f, 3.f);
  pMP->SetWorldPos(pos);
  EXPECT_LT(cv::norm(pMP->GetWorldPos() - pos), 1e-6);
  EXPECT_FALSE(pMP->isBad());
}

TEST_F(MapPointTest, ObservationsAreIndexedByKeyFrame) {
  KeyFrame* pKF = NewKeyFrame();
  MapPoint* pMP = NewMapPoint(pKF);

  EXPECT_FALSE(pMP->IsInKeyFrame(pKF));
  EXPECT_EQ(pMP->GetIndexInKeyFrame(pKF), -1);

  pMP->AddObservation(pKF, 3);
  EXPECT_TRUE(pMP->IsInKeyFrame(pKF));
  EXPECT_EQ(pMP->GetIndexInKeyFrame(pKF), 3);
  EXPECT_EQ(pMP->GetObservations().size(), 1u);
  EXPECT_EQ(pMP->GetReferenceKeyFrame(), pKF);

  // A repeated AddObservation is a no-op, not a double count.
  const int obs = pMP->Observations();
  pMP->AddObservation(pKF, 3);
  EXPECT_EQ(pMP->Observations(), obs);
}

TEST_F(MapPointTest, StereoObservationsCountDouble) {
  KeyFrame* pKF = NewKeyFrame();
  MapPoint* pMP = NewMapPoint(pKF);

  // Find a keypoint with a valid right coordinate (RGB-D gives most of them one).
  int idx = -1;
  for(int i = 0; i < pKF->N && idx < 0; ++i)
    if(pKF->mvuRight[i] >= 0)
      idx = i;
  ASSERT_GE(idx, 0) << "no stereo keypoint in the synthetic RGB-D keyframe";

  pMP->AddObservation(pKF, idx);
  EXPECT_EQ(pMP->Observations(), 2);
}

TEST_F(MapPointTest, SetBadFlagDropsObserversAndLeavesTheMap) {
  KeyFrame* pKF = NewKeyFrame();
  MapPoint* pMP = NewMapPoint(pKF);
  pMP->AddObservation(pKF, 0);
  pKF->AddMapPoint(pMP, 0);
  ASSERT_EQ(map.MapPointsInMap(), 1u);

  pMP->SetBadFlag();

  EXPECT_TRUE(pMP->isBad());
  EXPECT_TRUE(pMP->GetObservations().empty());
  EXPECT_EQ(pKF->GetMapPoint(0), nullptr);
  EXPECT_EQ(map.MapPointsInMap(), 0u);
}

TEST_F(MapPointTest, EraseObservationDiscardsAPointWithTooFewViews) {
  KeyFrame* pKF1 = NewKeyFrame(0);
  KeyFrame* pKF2 = NewKeyFrame(5);
  MapPoint* pMP = NewMapPoint(pKF1);

  // Mono-style observations (right coordinate ignored by using indices we know are mono is
  // fragile, so drive the count directly): 3 observations, erase one -> nObs 2 -> discarded.
  pMP->AddObservation(pKF1, 0);
  pMP->AddObservation(pKF2, 0);
  ASSERT_LE(pMP->Observations(), 4);

  pMP->EraseObservation(pKF1);
  EXPECT_FALSE(pMP->IsInKeyFrame(pKF1));
  EXPECT_TRUE(pMP->isBad()) << "dropping below three observations must discard the point";
}

TEST_F(MapPointTest, ReplaceRedirectsObservationsAndSetsReplaced) {
  KeyFrame* pKF = NewKeyFrame();
  MapPoint* pOld = NewMapPoint(pKF);
  MapPoint* pNew = NewMapPoint(pKF);

  pOld->AddObservation(pKF, 0);
  pKF->AddMapPoint(pOld, 0);
  pOld->IncreaseFound(4);
  pOld->IncreaseVisible(4);

  pOld->Replace(pNew);

  EXPECT_TRUE(pOld->isBad());
  EXPECT_EQ(pOld->GetReplaced(), pNew);
  EXPECT_EQ(pKF->GetMapPoint(0), pNew);
  EXPECT_TRUE(pNew->IsInKeyFrame(pKF));
  EXPECT_GE(pNew->GetFound(), 5);
  EXPECT_EQ(map.MapPointsInMap(), 1u);
}

TEST_F(MapPointTest, ReplaceWithItselfIsANoOp) {
  MapPoint* pMP = NewMapPoint(NewKeyFrame());
  pMP->Replace(pMP);
  EXPECT_FALSE(pMP->isBad());
  EXPECT_EQ(pMP->GetReplaced(), nullptr);
}

TEST_F(MapPointTest, FoundRatioIsFoundOverVisible) {
  MapPoint* pMP = NewMapPoint(NewKeyFrame());    // starts at 1 found / 1 visible
  pMP->IncreaseVisible(3);                       // 1 / 4
  EXPECT_NEAR(pMP->GetFoundRatio(), 0.25f, 1e-6);
  pMP->IncreaseFound(1);    // 2 / 4
  EXPECT_NEAR(pMP->GetFoundRatio(), 0.5f, 1e-6);
}

TEST_F(MapPointTest, DistinctiveDescriptorIsOneOfTheObserved) {
  // A point nobody observes keeps no descriptor at all.
  MapPoint* pUnobserved = NewMapPoint(NewKeyFrame());
  pUnobserved->ComputeDistinctiveDescriptors();
  EXPECT_TRUE(pUnobserved->GetDescriptor().empty());

  KeyFrame* pKF1 = NewKeyFrame(0);
  KeyFrame* pKF2 = NewKeyFrame(5);
  KeyFrame* pKF3 = NewKeyFrame(10);
  MapPoint* pMP = NewMapPoint(pKF1);

  pMP->AddObservation(pKF1, 0);
  pMP->AddObservation(pKF2, 1);
  pMP->AddObservation(pKF3, 2);
  pMP->ComputeDistinctiveDescriptors();

  const cv::Mat d = pMP->GetDescriptor();
  ASSERT_FALSE(d.empty());
  EXPECT_EQ(d.cols, 32);
  EXPECT_EQ(d.type(), CV_8U);

  // It must be one of the observed descriptors -- ComputeDistinctiveDescriptors picks the
  // one with the smallest median distance to the rest, it never synthesises a new one.
  const cv::Mat candidates[] = {pKF1->mDescriptors.row(0), pKF2->mDescriptors.row(1), pKF3->mDescriptors.row(2)};
  int matches = 0;
  for(const cv::Mat& c : candidates)
    matches += (cv::countNonZero(d != c) == 0);
  EXPECT_EQ(matches, 1);
}

TEST_F(MapPointTest, ScaleInvarianceBoundsBracketTheObservedDistance) {
  KeyFrame* pKF = NewKeyFrame();
  MapPoint* pMP = NewMapPoint(pKF);
  pMP->AddObservation(pKF, 0);
  pMP->UpdateNormalAndDepth();

  // Camera at the origin, point at PlaneDepth() on the optical axis.
  const float dist = synth::PlaneDepth();
  EXPECT_LT(pMP->GetMinDistanceInvariance(), dist);
  EXPECT_GT(pMP->GetMaxDistanceInvariance(), dist);
  EXPECT_NEAR(cv::norm(pMP->GetNormal()), 1.0, 1e-5);
}

TEST_F(MapPointTest, PredictScaleGrowsAsThePointGetsCloser) {
  KeyFrame* pKF = NewKeyFrame();
  MapPoint* pMP = NewMapPoint(pKF);
  pMP->AddObservation(pKF, 0);
  pMP->UpdateNormalAndDepth();

  const float far = pMP->GetMaxDistanceInvariance() * 0.99f;
  const float near = pMP->GetMinDistanceInvariance() * 1.01f;
  EXPECT_LE(pMP->PredictScale(far, pKF), pMP->PredictScale(near, pKF));
  EXPECT_GE(pMP->PredictScale(far, pKF), 0);
  EXPECT_LT(pMP->PredictScale(near, pKF), pKF->mnScaleLevels);
}

}    // namespace
