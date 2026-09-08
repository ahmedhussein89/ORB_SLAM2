#include "orbslam/mapping/LocalMapping.hpp"

#include <gtest/gtest.h>

#include "synthetic.hpp"

using ORB_SLAM2::KeyFrame;

namespace {

// These are lifecycle tests, not SLAM tests: they check the request/ack flag pairs the three
// threads talk to each other with. Whether local mapping produces a good map is what
// tools/smoke_test.sh answers.
struct LocalMappingTest : ::testing::Test {
  synth::Pipeline p;

  void TearDown() override {
    EXPECT_TRUE(p.Shutdown()) << "a pipeline thread did not finish";
  }
};

TEST_F(LocalMappingTest, AcceptsKeyFramesByDefault) {
  EXPECT_TRUE(p.mapper.AcceptKeyFrames());
  p.mapper.SetAcceptKeyFrames(false);
  EXPECT_FALSE(p.mapper.AcceptKeyFrames());
  p.mapper.SetAcceptKeyFrames(true);
  EXPECT_TRUE(p.mapper.AcceptKeyFrames());
}

TEST_F(LocalMappingTest, InsertKeyFrameQueuesIt) {
  EXPECT_EQ(p.mapper.KeyframesInQueue(), 0);
  p.mapper.InsertKeyFrame(p.NewKeyFrame());
  EXPECT_EQ(p.mapper.KeyframesInQueue(), 1);
  p.mapper.InsertKeyFrame(p.NewKeyFrame(4));
  EXPECT_EQ(p.mapper.KeyframesInQueue(), 2);
}

TEST_F(LocalMappingTest, StopIsARequestAckPair) {
  EXPECT_FALSE(p.mapper.stopRequested());
  EXPECT_FALSE(p.mapper.isStopped());
  EXPECT_FALSE(p.mapper.Stop()) << "Stop must not stop anything that was not asked to stop";

  p.mapper.RequestStop();
  EXPECT_TRUE(p.mapper.stopRequested());
  EXPECT_TRUE(p.mapper.Stop());
  EXPECT_TRUE(p.mapper.isStopped());
  // Release is a no-op until Run() has started -- the mapper reports itself finished until
  // then -- so the release half of the pair is checked in RunningThreadStopsAndReleases.
}

TEST_F(LocalMappingTest, SetNotStopBlocksAStop) {
  EXPECT_TRUE(p.mapper.SetNotStop(true));
  p.mapper.RequestStop();
  EXPECT_FALSE(p.mapper.Stop()) << "SetNotStop must veto the stop";
  EXPECT_FALSE(p.mapper.isStopped());

  p.mapper.SetNotStop(false);
  EXPECT_TRUE(p.mapper.Stop());
  EXPECT_TRUE(p.mapper.isStopped());

  // SetNotStop is refused once the mapper is actually stopped.
  EXPECT_FALSE(p.mapper.SetNotStop(true));
}

TEST_F(LocalMappingTest, RunningThreadStopsAndReleases) {
  p.Start();

  p.mapper.RequestStop();
  EXPECT_TRUE(synth::WaitFor([&] { return p.mapper.isStopped(); })) << "the running thread never acked the stop";
  EXPECT_FALSE(p.mapper.AcceptKeyFrames()) << "a stopped mapper must stop accepting keyframes";

  p.mapper.Release();
  EXPECT_TRUE(synth::WaitFor([&] { return p.mapper.AcceptKeyFrames(); }));
  EXPECT_FALSE(p.mapper.isStopped());
}

TEST_F(LocalMappingTest, RunningThreadConsumesAKeyFrame) {
  p.Start();

  p.mapper.InsertKeyFrame(p.NewKeyFrame());
  EXPECT_TRUE(synth::WaitFor([&] { return p.mapper.KeyframesInQueue() == 0; })) << "the keyframe was never processed";
}

TEST_F(LocalMappingTest, ResetDrainsTheQueue) {
  p.Start();

  p.mapper.InsertKeyFrame(p.NewKeyFrame());
  p.mapper.InsertKeyFrame(p.NewKeyFrame(4));
  p.mapper.RequestReset();    // blocks until the running thread acknowledges

  EXPECT_EQ(p.mapper.KeyframesInQueue(), 0);
}

TEST_F(LocalMappingTest, FinishStopsTheThread) {
  p.Start();

  // isFinished() reports true until Run() actually enters its loop, so wait for the thread to
  // come up before asking it to shut down.
  ASSERT_TRUE(synth::WaitFor([&] { return !p.mapper.isFinished(); }));
  p.mapper.RequestFinish();
  EXPECT_TRUE(synth::WaitFor([&] { return p.mapper.isFinished(); }));
}

}    // namespace
