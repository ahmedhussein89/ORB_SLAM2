#include "include/ORBmatcher.h"

#include <gtest/gtest.h>

#include "include/Map.h"
#include "synthetic.h"

using ORB_SLAM2::Frame;
using ORB_SLAM2::KeyFrame;
using ORB_SLAM2::Map;
using ORB_SLAM2::MapPoint;
using ORB_SLAM2::ORBmatcher;

namespace {

const int kShift = 8;    // pixels the synthetic camera pans between the two test frames

TEST(ORBmatcher, DescriptorDistanceCountsDifferingBits) {
  cv::Mat a = cv::Mat::zeros(1, 32, CV_8U);
  cv::Mat b = a.clone();

  EXPECT_EQ(ORBmatcher::DescriptorDistance(a, b), 0);

  b.at<uchar>(0, 0) = 0x01;
  EXPECT_EQ(ORBmatcher::DescriptorDistance(a, b), 1);
  EXPECT_EQ(ORBmatcher::DescriptorDistance(b, a), 1);

  b.at<uchar>(0, 0) = 0xFF;
  EXPECT_EQ(ORBmatcher::DescriptorDistance(a, b), 8);

  b.setTo(0xFF);
  EXPECT_EQ(ORBmatcher::DescriptorDistance(a, b), 256);

  EXPECT_LT(ORBmatcher::TH_LOW, ORBmatcher::TH_HIGH);
  EXPECT_GT(ORBmatcher::HISTO_LENGTH, 0);
}

TEST(ORBmatcher, SearchForInitializationFindsThePan) {
  Frame F1 = synth::MakeFrame(synth::kMono, 0);
  Frame F2 = synth::MakeFrame(synth::kMono, kShift);

  std::vector<cv::Point2f> prevMatched(F1.mvKeysUn.size());
  for(size_t i = 0; i < F1.mvKeysUn.size(); ++i)
    prevMatched[i] = F1.mvKeysUn[i].pt;
  std::vector<int> matches12;

  ORBmatcher matcher(0.9f, true);
  const int n = matcher.SearchForInitialization(F1, F2, prevMatched, matches12, 20);

  ASSERT_GT(n, 50) << "two views of the same plane 8 px apart must match";
  ASSERT_EQ(matches12.size(), F1.mvKeysUn.size());

  // The window is panned right, so the same feature sits kShift px further left in F2.
  std::vector<float> dx;
  for(size_t i = 0; i < matches12.size(); ++i)
    if(matches12[i] >= 0)
      dx.push_back(F2.mvKeysUn[matches12[i]].pt.x - F1.mvKeysUn[i].pt.x);
  ASSERT_EQ(static_cast<int>(dx.size()), n);
  std::sort(dx.begin(), dx.end());
  EXPECT_NEAR(dx[dx.size() / 2], -static_cast<float>(kShift), 2.0f);
}

TEST(ORBmatcher, SearchForInitializationFindsNothingBetweenUnrelatedViews) {
  Frame F1 = synth::MakeFrame(synth::kMono, 0);
  Frame F2 = synth::MakeFrame(synth::kMono, 100);    // no overlap within the search window

  std::vector<cv::Point2f> prevMatched(F1.mvKeysUn.size());
  for(size_t i = 0; i < F1.mvKeysUn.size(); ++i)
    prevMatched[i] = F1.mvKeysUn[i].pt;
  std::vector<int> matches12;

  ORBmatcher matcher(0.6f, true);
  const int n = matcher.SearchForInitialization(F1, F2, prevMatched, matches12, 10);

  EXPECT_LT(n, 30) << "a 100 px pan is outside a 10 px search window";
}

TEST(ORBmatcher, SearchByBoWMatchesAKeyFrameAgainstTheSameView) {
  Map map;
  Frame KFrame = synth::MakeFrame(synth::kRgbd, 0);
  KeyFrame* pKF = synth::MakeKeyFrame(KFrame, &map);

  // Give every keypoint of the keyframe a MapPoint, so SearchByBoW has something to return.
  pKF->SetPose(cv::Mat::eye(4, 4, CV_32F));
  int nPoints = 0;
  for(int i = 0; i < pKF->N; ++i) {
    const cv::Mat x3D = pKF->UnprojectStereo(i);
    if(x3D.empty())
      continue;
    MapPoint* pMP = new MapPoint(x3D, pKF, &map);
    pMP->AddObservation(pKF, i);
    pMP->ComputeDistinctiveDescriptors();
    pKF->AddMapPoint(pMP, i);
    ++nPoints;
  }
  ASSERT_GT(nPoints, 100);

  Frame F = synth::MakeFrame(synth::kRgbd, 0);
  F.ComputeBoW();
  std::vector<MapPoint*> matches;

  ORBmatcher matcher(0.7f, true);
  const int n = matcher.SearchByBoW(pKF, F, matches);

  EXPECT_GT(n, nPoints / 4) << "the frame is the very image the keyframe was built from";
  EXPECT_EQ(matches.size(), static_cast<size_t>(F.N));
}

TEST(ORBmatcher, SearchByProjectionAgainstTheLastFrame) {
  Map map;
  Frame last = synth::MakeFrame(synth::kRgbd, 0);
  last.SetPose(cv::Mat::eye(4, 4, CV_32F));

  int nPoints = 0;
  for(int i = 0; i < last.N; ++i) {
    if(last.mvDepth[i] <= 0.f)
      continue;
    MapPoint* pMP = new MapPoint(last.UnprojectStereo(i), &map, &last, i);
    last.mvpMapPoints[i] = pMP;
    ++nPoints;
  }
  ASSERT_GT(nPoints, 100);

  Frame current = synth::MakeFrame(synth::kRgbd, 4);
  current.SetPose(cv::Mat::eye(4, 4, CV_32F));    // near enough for a projection search

  ORBmatcher matcher(0.9f, true);
  const int n = matcher.SearchByProjection(current, last, 15.f, false);

  EXPECT_GT(n, nPoints / 10) << "the two views overlap almost completely";
}

TEST(ORBmatcher, FuseReplacesADuplicatedMapPoint) {
  Map map;
  Frame F1 = synth::MakeFrame(synth::kRgbd, 0);
  Frame F2 = synth::MakeFrame(synth::kRgbd, 0);    // the same view, a second time
  KeyFrame* pKF1 = synth::MakeKeyFrame(F1, &map);
  KeyFrame* pKF2 = synth::MakeKeyFrame(F2, &map);
  pKF1->SetPose(cv::Mat::eye(4, 4, CV_32F));
  pKF2->SetPose(cv::Mat::eye(4, 4, CV_32F));

  // Fuse skips points the keyframe already observes, so the duplicate has to live in a
  // different keyframe. Pair up a keypoint that both keyframes found, by exact descriptor.
  int i1 = -1, i2 = -1;
  for(int i = 0; i < pKF1->N && i1 < 0; ++i) {
    if(pKF1->mvuRight[i] < 0 || pKF1->mvKeysUn[i].octave != 0)
      continue;
    for(int j = 0; j < pKF2->N; ++j) {
      if(ORBmatcher::DescriptorDistance(pKF1->mDescriptors.row(i), pKF2->mDescriptors.row(j)) != 0)
        continue;
      if(cv::norm(pKF1->mvKeysUn[i].pt - pKF2->mvKeysUn[j].pt) > 1.0)
        continue;
      i1 = i, i2 = j;
      break;
    }
  }
  ASSERT_GE(i1, 0) << "the two extractions of the same image share no keypoint";

  auto NewPoint = [&](KeyFrame* pKF, int idx) {
    MapPoint* pMP = new MapPoint(pKF->UnprojectStereo(idx), pKF, &map);
    pMP->AddObservation(pKF, idx);
    pKF->AddMapPoint(pMP, idx);
    pMP->ComputeDistinctiveDescriptors();
    pMP->UpdateNormalAndDepth();
    map.AddMapPoint(pMP);
    return pMP;
  };
  MapPoint* pKept = NewPoint(pKF1, i1);
  MapPoint* pDuplicate = NewPoint(pKF2, i2);
  ASSERT_EQ(map.MapPointsInMap(), 2u);

  ORBmatcher matcher;
  const int nFused = matcher.Fuse(pKF1, std::vector<MapPoint*>{pDuplicate}, 3.0f);

  EXPECT_EQ(nFused, 1);
  // Fuse keeps whichever point has more observations and discards the other.
  ASSERT_TRUE(pDuplicate->isBad() != pKept->isBad()) << "exactly one of the two must survive";
  MapPoint* pGone = pDuplicate->isBad() ? pDuplicate : pKept;
  MapPoint* pSurvivor = pDuplicate->isBad() ? pKept : pDuplicate;
  EXPECT_EQ(pGone->GetReplaced(), pSurvivor);
  EXPECT_EQ(pKF1->GetMapPoint(i1), pSurvivor);
  EXPECT_EQ(map.MapPointsInMap(), 1u);
}

}    // namespace
