#include "orbslam/core/Map.hpp"

#include <gtest/gtest.h>

#include "synthetic.hpp"

using ORB_SLAM2::KeyFrame;
using ORB_SLAM2::Map;
using ORB_SLAM2::MapPoint;

namespace {

// One Frame is enough to spawn as many KeyFrames as a Map test needs.
KeyFrame* NewKeyFrame(Map* pMap) {
  ORB_SLAM2::Frame F = synth::MakeFrame(synth::kRgbd);
  return synth::MakeKeyFrame(F, pMap);
}

TEST(Map, StartsEmpty) {
  Map map;
  EXPECT_EQ(map.KeyFramesInMap(), 0u);
  EXPECT_EQ(map.MapPointsInMap(), 0u);
  EXPECT_TRUE(map.GetAllKeyFrames().empty());
  EXPECT_TRUE(map.GetAllMapPoints().empty());
}

TEST(Map, AddAndEraseKeyFrames) {
  Map map;
  KeyFrame* pKF = NewKeyFrame(&map);
  map.AddKeyFrame(pKF);
  EXPECT_EQ(map.KeyFramesInMap(), 1u);
  EXPECT_EQ(map.GetAllKeyFrames().front(), pKF);

  map.AddKeyFrame(pKF);    // the set de-duplicates
  EXPECT_EQ(map.KeyFramesInMap(), 1u);

  map.EraseKeyFrame(pKF);
  EXPECT_EQ(map.KeyFramesInMap(), 0u);
}

TEST(Map, MaxKFidTracksTheLargestIdAdded) {
  Map map;
  KeyFrame* pKF1 = NewKeyFrame(&map);
  KeyFrame* pKF2 = NewKeyFrame(&map);
  ASSERT_GT(pKF2->mnId, pKF1->mnId);

  map.AddKeyFrame(pKF2);
  map.AddKeyFrame(pKF1);
  EXPECT_EQ(map.GetMaxKFid(), pKF2->mnId);

  // Erasing does not lower it -- ids are never reused.
  map.EraseKeyFrame(pKF2);
  EXPECT_EQ(map.GetMaxKFid(), pKF2->mnId);
}

TEST(Map, AddAndEraseMapPoints) {
  Map map;
  KeyFrame* pKF = NewKeyFrame(&map);
  MapPoint* pMP = new MapPoint(synth::Point3(0, 0, 2), pKF, &map);

  map.AddMapPoint(pMP);
  EXPECT_EQ(map.MapPointsInMap(), 1u);
  map.EraseMapPoint(pMP);
  EXPECT_EQ(map.MapPointsInMap(), 0u);
}

TEST(Map, ReferenceMapPointsRoundTrip) {
  Map map;
  KeyFrame* pKF = NewKeyFrame(&map);
  std::vector<MapPoint*> refs{new MapPoint(synth::Point3(0, 0, 2), pKF, &map), new MapPoint(synth::Point3(1, 0, 2), pKF, &map)};

  map.SetReferenceMapPoints(refs);
  EXPECT_EQ(map.GetReferenceMapPoints(), refs);
}

TEST(Map, BigChangeIndexIncrements) {
  Map map;
  const int before = map.GetLastBigChangeIdx();
  map.InformNewBigChange();
  EXPECT_EQ(map.GetLastBigChangeIdx(), before + 1);
  map.InformNewBigChange();
  EXPECT_EQ(map.GetLastBigChangeIdx(), before + 2);
}

TEST(Map, ClearEmptiesEverything) {
  Map map;
  KeyFrame* pKF = NewKeyFrame(&map);
  map.AddKeyFrame(pKF);
  map.AddMapPoint(new MapPoint(synth::Point3(0, 0, 2), pKF, &map));
  map.mvpKeyFrameOrigins.push_back(pKF);

  map.clear();

  EXPECT_EQ(map.KeyFramesInMap(), 0u);
  EXPECT_EQ(map.MapPointsInMap(), 0u);
  EXPECT_EQ(map.GetMaxKFid(), 0u);
  EXPECT_TRUE(map.mvpKeyFrameOrigins.empty());
}

}    // namespace
