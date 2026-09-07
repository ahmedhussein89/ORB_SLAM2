#include "include/KeyFrame.h"

#include <gtest/gtest.h>

#include "include/KeyFrameDatabase.h"
#include "include/Map.h"
#include "synthetic.h"

using ORB_SLAM2::KeyFrame;
using ORB_SLAM2::KeyFrameDatabase;
using ORB_SLAM2::Map;
using ORB_SLAM2::MapPoint;

namespace {

struct KeyFrameTest : ::testing::Test {
  Map map;
  KeyFrameDatabase kfdb{synth::TinyVocab()};

  // KeyFrame::SetBadFlag refuses to erase id 0 and dereferences the database, so every
  // fixture starts with a throwaway keyframe and always passes a real database.
  void SetUp() override {
    NewKeyFrame();
  }

  KeyFrame* NewKeyFrame(int shift = 0) {
    ORB_SLAM2::Frame F = synth::MakeFrame(synth::kRgbd, shift);
    KeyFrame* pKF = synth::MakeKeyFrame(F, &map, &kfdb);
    map.AddKeyFrame(pKF);
    return pKF;
  }
};

TEST_F(KeyFrameTest, PoseAccessorsAreConsistent) {
  KeyFrame* pKF = NewKeyFrame();
  cv::Mat Tcw = synth::PoseAt(1.f, 2.f, 3.f);
  pKF->SetPose(Tcw);

  EXPECT_LT(cv::norm(pKF->GetPose() - Tcw), 1e-6);
  EXPECT_LT(cv::norm(pKF->GetPose() * pKF->GetPoseInverse() - cv::Mat::eye(4, 4, CV_32F)), 1e-5);

  // Camera centre is -R' * t, i.e. the world position we asked for.
  const cv::Mat Ow = pKF->GetCameraCenter();
  EXPECT_NEAR(Ow.at<float>(0), 1.f, 1e-5);
  EXPECT_NEAR(Ow.at<float>(1), 2.f, 1e-5);
  EXPECT_NEAR(Ow.at<float>(2), 3.f, 1e-5);
  EXPECT_LT(cv::norm(-pKF->GetRotation().t() * pKF->GetTranslation() - Ow), 1e-5);
}

TEST_F(KeyFrameTest, CovisiblesAreOrderedByDescendingWeight) {
  KeyFrame* pKF = NewKeyFrame();
  KeyFrame* pWeak = NewKeyFrame(4);
  KeyFrame* pMid = NewKeyFrame(8);
  KeyFrame* pStrong = NewKeyFrame(12);

  pKF->AddConnection(pWeak, 10);
  pKF->AddConnection(pStrong, 100);
  pKF->AddConnection(pMid, 50);

  EXPECT_EQ(pKF->GetWeight(pStrong), 100);
  EXPECT_EQ(pKF->GetWeight(pWeak), 10);
  EXPECT_EQ(pKF->GetConnectedKeyFrames().size(), 3u);
  EXPECT_EQ(pKF->GetVectorCovisibleKeyFrames(), (std::vector<KeyFrame*>{pStrong, pMid, pWeak}));
  EXPECT_EQ(pKF->GetBestCovisibilityKeyFrames(2), (std::vector<KeyFrame*>{pStrong, pMid}));
  EXPECT_EQ(pKF->GetBestCovisibilityKeyFrames(99).size(), 3u);
  EXPECT_EQ(pKF->GetCovisiblesByWeight(40), (std::vector<KeyFrame*>{pStrong, pMid}));

  pKF->EraseConnection(pMid);
  EXPECT_EQ(pKF->GetWeight(pMid), 0);
  EXPECT_EQ(pKF->GetVectorCovisibleKeyFrames().size(), 2u);
}

TEST_F(KeyFrameTest, SpanningTreeAndLoopEdges) {
  KeyFrame* pParent = NewKeyFrame();
  KeyFrame* pChild = NewKeyFrame(4);
  KeyFrame* pLoop = NewKeyFrame(8);

  pChild->ChangeParent(pParent);
  EXPECT_EQ(pChild->GetParent(), pParent);
  EXPECT_TRUE(pParent->hasChild(pChild));
  EXPECT_EQ(pParent->GetChilds().count(pChild), 1u);

  pParent->EraseChild(pChild);
  EXPECT_FALSE(pParent->hasChild(pChild));

  pChild->AddLoopEdge(pLoop);
  EXPECT_EQ(pChild->GetLoopEdges().count(pLoop), 1u);
  // A loop edge pins the keyframe: SetBadFlag must not erase it.
  pChild->SetBadFlag();
  EXPECT_FALSE(pChild->isBad());
}

TEST_F(KeyFrameTest, SetBadFlagRewiresChildrenToTheGrandparent) {
  KeyFrame* pGrand = NewKeyFrame();
  KeyFrame* pParent = NewKeyFrame(4);
  KeyFrame* pChild = NewKeyFrame(8);
  ASSERT_NE(pParent->mnId, 0u);

  pParent->ChangeParent(pGrand);
  pChild->ChangeParent(pParent);
  const unsigned long before = map.KeyFramesInMap();

  pParent->SetBadFlag();

  EXPECT_TRUE(pParent->isBad());
  EXPECT_EQ(pChild->GetParent(), pGrand);
  EXPECT_FALSE(pGrand->hasChild(pParent));
  EXPECT_EQ(map.KeyFramesInMap(), before - 1);
}

TEST_F(KeyFrameTest, SetNotEraseDefersTheErase) {
  KeyFrame* pParent = NewKeyFrame();
  KeyFrame* pKF = NewKeyFrame(4);
  pKF->ChangeParent(pParent);

  pKF->SetNotErase();
  pKF->SetBadFlag();
  EXPECT_FALSE(pKF->isBad()) << "SetNotErase must defer the erase";

  pKF->SetErase();
  EXPECT_TRUE(pKF->isBad()) << "SetErase must apply the deferred erase";
}

TEST_F(KeyFrameTest, MapPointMatchesByIndexAndByPointer) {
  KeyFrame* pKF = NewKeyFrame();
  MapPoint* pMP = new MapPoint(synth::Point3(0.f, 0.f, synth::PlaneDepth()), pKF, &map);

  pKF->AddMapPoint(pMP, 5);
  EXPECT_EQ(pKF->GetMapPoint(5), pMP);
  EXPECT_EQ(pKF->GetMapPoints().count(pMP), 1u);
  pMP->AddObservation(pKF, 5);
  EXPECT_EQ(pKF->GetMapPointMatches().size(), static_cast<size_t>(pKF->N));

  pKF->EraseMapPointMatch(pMP);
  EXPECT_EQ(pKF->GetMapPoint(5), nullptr);
  EXPECT_EQ(pKF->GetMapPoints().count(pMP), 0u);

  // GetMapPoints filters bad points, GetMapPointMatches does not.
  pKF->AddMapPoint(pMP, 5);
  pMP->SetBadFlag();
  EXPECT_EQ(pKF->GetMapPoints().count(pMP), 0u);
}

TEST_F(KeyFrameTest, GetFeaturesInAreaMatchesABruteForceScan) {
  KeyFrame* pKF = NewKeyFrame();
  const float x = 320.f, y = 240.f, r = 25.f;

  const std::vector<size_t> got = pKF->GetFeaturesInArea(x, y, r);

  // The grid is a coarse accelerator: it may return nothing outside the box, and everything
  // strictly inside the box must be found.
  std::set<size_t> expected;
  for(int i = 0; i < pKF->N; ++i) {
    const cv::KeyPoint& kp = pKF->mvKeysUn[i];
    if(std::abs(kp.pt.x - x) < r && std::abs(kp.pt.y - y) < r)
      expected.insert(i);
  }
  ASSERT_FALSE(expected.empty()) << "synthetic image has no features near the centre";

  const std::set<size_t> gotSet(got.begin(), got.end());
  for(size_t i : expected)
    EXPECT_EQ(gotSet.count(i), 1u) << "missed keypoint " << i;
  for(size_t i : got) {
    EXPECT_LT(std::abs(pKF->mvKeysUn[i].pt.x - x), r + 1e-3f);
    EXPECT_LT(std::abs(pKF->mvKeysUn[i].pt.y - y), r + 1e-3f);
  }
}

TEST_F(KeyFrameTest, IsInImage) {
  KeyFrame* pKF = NewKeyFrame();
  EXPECT_TRUE(pKF->IsInImage(320.f, 240.f));
  EXPECT_FALSE(pKF->IsInImage(-1.f, 240.f));
  EXPECT_FALSE(pKF->IsInImage(320.f, static_cast<float>(synth::Height()) + 1.f));
}

TEST_F(KeyFrameTest, UnprojectStereoRecoversThePlaneDepth) {
  KeyFrame* pKF = NewKeyFrame();
  pKF->SetPose(cv::Mat::eye(4, 4, CV_32F));

  int idx = -1;
  for(int i = 0; i < pKF->N && idx < 0; ++i)
    if(pKF->mvuRight[i] >= 0)
      idx = i;
  ASSERT_GE(idx, 0);

  const cv::Mat x3D = pKF->UnprojectStereo(idx);
  ASSERT_FALSE(x3D.empty());
  EXPECT_NEAR(x3D.at<float>(2), synth::PlaneDepth(), 0.05f);
}

TEST_F(KeyFrameTest, SceneMedianDepthOfAPlaneIsThePlaneDepth) {
  KeyFrame* pKF = NewKeyFrame();
  pKF->SetPose(cv::Mat::eye(4, 4, CV_32F));

  int added = 0;
  for(int i = 0; i < pKF->N && added < 20; ++i) {
    const cv::Mat x3D = pKF->UnprojectStereo(i);
    if(x3D.empty())
      continue;
    MapPoint* pMP = new MapPoint(x3D, pKF, &map);
    pKF->AddMapPoint(pMP, i);
    ++added;
  }
  ASSERT_GT(added, 0);

  EXPECT_NEAR(pKF->ComputeSceneMedianDepth(2), synth::PlaneDepth(), 0.05f);
}

TEST_F(KeyFrameTest, BowVectorIsPopulated) {
  KeyFrame* pKF = NewKeyFrame();
  EXPECT_FALSE(pKF->mBowVec.empty());
  EXPECT_FALSE(pKF->mFeatVec.empty());
}

}    // namespace
