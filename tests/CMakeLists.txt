# One binary for all tests: 17 executables would mean 17 link steps against
# libORB_SLAM2.so for no gain.
add_executable(orbslam2_tests
  converter_test.cc
  map_test.cc
  mappoint_test.cc
  keyframe_test.cc
  frame_test.cc
  orbextractor_test.cc
  orbmatcher_test.cc
  initializer_test.cc
  pnpsolver_test.cc
  sim3solver_test.cc
  optimizer_test.cc
  keyframedatabase_test.cc
  localmapping_test.cc
  loopclosing_test.cc
  tracking_test.cc
  system_test.cc
)
target_link_libraries(orbslam2_tests PRIVATE ORB_SLAM2::ORB_SLAM2 GTest::gmock_main)
add_test(NAME orbslam2_tests COMMAND orbslam2_tests)

# A deadlock in the thread lifecycle tests should fail, not hang CI.
set_tests_properties(orbslam2_tests PROPERTIES TIMEOUT 300)
