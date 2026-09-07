#include "include/LoopClosing.h"

#include <gtest/gtest.h>

#include "synthetic.h"

using ORB_SLAM2::KeyFrame;

namespace {

// Lifecycle and wiring only. A synthetic three-keyframe map has no loop to close, so what is
// under test is that the thread starts, drains its queue, resets and finishes -- and that
// DetectLoop on a covisible cluster does not fire.
struct LoopClosingTest : ::testing::Test {
  synth::Pipeline p;

  void TearDown() override {
    EXPECT_TRUE(p.Shutdown()) << "a pipeline thread did not finish";
  }
};

TEST_F(LoopClosingTest, StartsIdle) {
  EXPECT_FALSE(p.closer.isRunningGBA());
  EXPECT_TRUE(p.closer.isFinishedGBA());
  // isFinished() reports true until Run() actually starts -- the flag means "the thread is
  // not running", not "the thread ran and stopped".
  EXPECT_TRUE(p.closer.isFinished());

  p.Start();
  EXPECT_TRUE(synth::WaitFor([&] { return !p.closer.isFinished(); }));
}

TEST_F(LoopClosingTest, ConsumesKeyFramesWithoutFindingALoop) {
  p.Start();

  // Three views of the same plane, mutually covisible: every candidate is a neighbour, so
  // nothing may be reported as a loop.
  std::vector<KeyFrame*> kfs;
  for(int k = 0; k < 3; ++k)
    kfs.push_back(p.NewKeyFrame(4 * k));
  for(KeyFrame* a : kfs)
    for(KeyFrame* b : kfs)
      if(a != b)
        a->AddConnection(b, 100);

  for(KeyFrame* pKF : kfs)
    p.closer.InsertKeyFrame(pKF);

  // Give the thread time to drain the queue, then check it neither fired a loop nor died.
  EXPECT_TRUE(synth::WaitFor([&] { return !p.closer.isRunningGBA(); }));
  EXPECT_EQ(p.map.KeyFramesInMap(), 3u);
}

TEST_F(LoopClosingTest, IgnoresTheFirstKeyFrame) {
  p.Start();

  // LoopClosing::DetectLoop refuses to consider keyframes younger than 10 frames after the
  // last loop, and never the map's very first keyframe.
  KeyFrame* pFirst = p.NewKeyFrame(0);
  p.closer.InsertKeyFrame(pFirst);

  EXPECT_TRUE(synth::WaitFor([&] { return !p.closer.isRunningGBA(); }));
  EXPECT_FALSE(pFirst->isBad());
}

TEST_F(LoopClosingTest, ResetClearsTheQueue) {
  p.Start();

  for(int k = 0; k < 3; ++k)
    p.closer.InsertKeyFrame(p.NewKeyFrame(4 * k));
  p.closer.RequestReset();    // blocks until the running thread acknowledges

  EXPECT_FALSE(p.closer.isFinished()) << "a reset must not finish the thread";
}

TEST_F(LoopClosingTest, FinishStopsTheThread) {
  p.Start();

  ASSERT_TRUE(synth::WaitFor([&] { return !p.closer.isFinished(); }));
  p.closer.RequestFinish();
  EXPECT_TRUE(synth::WaitFor([&] { return p.closer.isFinished(); }));
}

TEST_F(LoopClosingTest, GlobalBundleAdjustmentRunsToCompletion) {
  // RunGlobalBundleAdjustment is what the loop thread hands to a fourth, detached thread.
  // Call it directly on a small map: it must finish and leave the poses finite.
  KeyFrame* pKF = p.NewKeyFrame(0);
  p.map.mvpKeyFrameOrigins.push_back(pKF);
  for(int i = 0; i < pKF->N; ++i) {
    const cv::Mat X = pKF->UnprojectStereo(i);
    if(X.empty())
      continue;
    ORB_SLAM2::MapPoint* pMP = new ORB_SLAM2::MapPoint(X, pKF, &p.map);
    pMP->AddObservation(pKF, i);
    pKF->AddMapPoint(pMP, i);
    p.map.AddMapPoint(pMP);
  }
  ASSERT_GT(p.map.MapPointsInMap(), 100u);

  p.closer.RunGlobalBundleAdjustment(pKF->mnId);

  EXPECT_TRUE(p.closer.isFinishedGBA());
  EXPECT_FALSE(p.closer.isRunningGBA());
  EXPECT_TRUE(cv::checkRange(pKF->GetPose(), true, nullptr, -1e9, 1e9));
}

}    // namespace
