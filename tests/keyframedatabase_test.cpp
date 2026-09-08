#include "orbslam/features/KeyFrameDatabase.hpp"

#include <gtest/gtest.h>

#include "orbslam/core/Map.hpp"
#include "synthetic.hpp"

using ORB_SLAM2::Frame;
using ORB_SLAM2::KeyFrame;
using ORB_SLAM2::KeyFrameDatabase;
using ORB_SLAM2::Map;

namespace {

struct KeyFrameDatabaseTest : ::testing::Test {
  Map map;
  KeyFrameDatabase kfdb{synth::TinyVocab()};

  KeyFrame* NewKeyFrame(int shift) {
    Frame F = synth::MakeFrame(synth::kRgbd, shift);
    KeyFrame* pKF = synth::MakeKeyFrame(F, &map, &kfdb);
    map.AddKeyFrame(pKF);
    return pKF;
  }

  KeyFrame* AddKeyFrame(int shift) {
    KeyFrame* pKF = NewKeyFrame(shift);
    kfdb.add(pKF);
    return pKF;
  }

  // DetectRelocalizationCandidates memoises per query frame id (mnRelocQuery), so every query
  // needs a fresh Frame or the second call silently returns nothing.
  Frame Query(int shift) {
    Frame F = synth::MakeFrame(synth::kRgbd, shift);
    F.ComputeBoW();
    return F;
  }
};

TEST_F(KeyFrameDatabaseTest, RelocalizationFindsTheMatchingView) {
  KeyFrame* pSame = AddKeyFrame(0);
  AddKeyFrame(60);
  AddKeyFrame(110);

  Frame query = Query(0);    // the very view pSame was built from
  const std::vector<KeyFrame*> candidates = kfdb.DetectRelocalizationCandidates(&query);

  ASSERT_FALSE(candidates.empty()) << "the query is an exact re-observation";
  EXPECT_NE(std::find(candidates.begin(), candidates.end(), pSame), candidates.end());
}

TEST_F(KeyFrameDatabaseTest, EraseAndClearRemoveCandidates) {
  KeyFrame* pKF = AddKeyFrame(0);

  Frame q1 = Query(0);
  ASSERT_FALSE(kfdb.DetectRelocalizationCandidates(&q1).empty());

  kfdb.erase(pKF);
  Frame q2 = Query(0);
  EXPECT_TRUE(kfdb.DetectRelocalizationCandidates(&q2).empty());

  kfdb.add(pKF);
  Frame q3 = Query(0);
  ASSERT_FALSE(kfdb.DetectRelocalizationCandidates(&q3).empty());

  kfdb.clear();
  Frame q4 = Query(0);
  EXPECT_TRUE(kfdb.DetectRelocalizationCandidates(&q4).empty());
}

TEST_F(KeyFrameDatabaseTest, LoopCandidatesExcludeTheQuerysNeighbours) {
  // LoopClosing adds the current keyframe to the database only *after* querying it, so the
  // query is not in the inverted file -- there is no self-exclusion inside the database.
  KeyFrame* pQuery = NewKeyFrame(0);
  KeyFrame* pNeighbour = AddKeyFrame(0);    // identical view, so a perfect BoW match...
  KeyFrame* pLoop = AddKeyFrame(0);         // ...and so is this one

  // ...but the first is a covisible neighbour of the query, so it must never be a loop.
  pQuery->AddConnection(pNeighbour, 100);
  pNeighbour->AddConnection(pQuery, 100);

  const std::vector<KeyFrame*> candidates = kfdb.DetectLoopCandidates(pQuery, 0.f);

  EXPECT_EQ(std::find(candidates.begin(), candidates.end(), pNeighbour), candidates.end())
      << "a covisible keyframe is not a loop closure";
  EXPECT_EQ(std::find(candidates.begin(), candidates.end(), pQuery), candidates.end());
  EXPECT_NE(std::find(candidates.begin(), candidates.end(), pLoop), candidates.end());
}

TEST_F(KeyFrameDatabaseTest, LoopCandidatesRespectMinScore) {
  KeyFrame* pQuery = NewKeyFrame(0);
  AddKeyFrame(0);

  EXPECT_FALSE(kfdb.DetectLoopCandidates(pQuery, 0.f).empty());
  // Nothing can score above 1, so an impossible threshold must return nothing.
  EXPECT_TRUE(kfdb.DetectLoopCandidates(pQuery, 2.f).empty());
}

}    // namespace
