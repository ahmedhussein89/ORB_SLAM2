add_library(${PROJECT_NAME} SHARED
  System.cc
  Tracking.cc
  LocalMapping.cc
  LoopClosing.cc
  ORBextractor.cc
  ORBmatcher.cc
  Converter.cc
  MapPoint.cc
  KeyFrame.cc
  Map.cc
  Optimizer.cc
  PnPsolver.cc
  Frame.cc
  KeyFrameDatabase.cc
  Sim3Solver.cc
  Initializer.cc
)
add_library(ORB_SLAM2::ORB_SLAM2 ALIAS ${PROJECT_NAME})

# PROJECT_SOURCE_DIR is public: public headers include "Thirdparty/g2o/g2o/...".
target_include_directories(${PROJECT_NAME} PUBLIC
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall)
target_link_libraries(${PROJECT_NAME} PUBLIC ${OpenCV_LIBS} DBoW2 g2o)
