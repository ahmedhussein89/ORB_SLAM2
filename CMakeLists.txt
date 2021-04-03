# ${CMAKE_SOURCE_DIR}/CMakeLists.txt
cmake_minimum_required(VERSION 3.16)
project(ORB_SLAM2 CXX)

option(ENABLE_UNIT_TESTING "Build unit-testing." OFF)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
include(options)
#include(coverage)

find_package(fmt      REQUIRED)
find_package(spdlog   REQUIRED)
find_package(Eigen3   REQUIRED)
find_package(OpenCV   REQUIRED features2d highgui imgcodecs imgproc calib3d)
find_package(Pangolin REQUIRED)
find_package(range-v3 REQUIRED)
find_package(g2o   REQUIRED)
find_package(DBoW2 REQUIRED)

add_library(
  ${PROJECT_NAME}
  STATIC
)

target_sources(
  ${PROJECT_NAME}
  PRIVATE
  src/Map.cpp
  src/Frame.cpp
  src/System.cpp
  src/Viewer.cpp
  src/Tracking.cpp
  src/MapPoint.cpp
  src/KeyFrame.cpp
  src/Converter.cpp
  src/MapDrawer.cpp
  src/Converter.cpp
  src/Optimizer.cpp
  src/PnPsolver.cpp
  src/ORBmatcher.cpp
  src/Sim3Solver.cpp
  src/FrameDrawer.cpp
  src/LoopClosing.cpp
  src/Initializer.cpp
  src/LocalMapping.cpp
  src/ORBextractor.cpp
  src/KeyFrameDatabase.cpp
)

target_include_directories(
  ${PROJECT_NAME}
  PUBLIC
  ${PROJECT_SOURCE_DIR}/include
)

target_include_directories(
  ${PROJECT_NAME}
  SYSTEM PRIVATE
  ${PROJECT_SOURCE_DIR}
  ${Pangolin_INCLUDE_DIRS}
)

target_include_directories(
  ${PROJECT_NAME}
  SYSTEM PUBLIC
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(
  ${PROJECT_NAME}
  PUBLIC
  Eigen3::Eigen
  ${OpenCV_LIBS}
)

target_link_libraries(
  ${PROJECT_NAME}
  PRIVATE
  ${Pangolin_LIBRARIES}
  fmt::fmt
  spdlog::spdlog
  options::options
  range-v3::range-v3
  g2o
  DBoW2
)

target_compile_options(
  ${PROJECT_NAME}
  PRIVATE
  $<$<PLATFORM_ID:Linux>:-march=native>
)

target_compile_definitions(
  ${PROJECT_NAME}
  PRIVATE
  COMPILEDWITHC11
)

target_precompile_headers(
  ${PROJECT_NAME}
  PRIVATE
  include/precompile.hpp
)

set_target_properties(
  ${PROJECT_NAME}
  PROPERTIES
  CXX_STANDARD          17
  CXX_EXTENSIONS        OFF
  CXX_STANDARD_REQUIRED ON
)

if(ENABLE_UNIT_TESTING)
  enable_testing()
  add_subdirectory(tests/unit-tests/)
endif()

if(ENABLE_EXAMPLES)
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/RGB-D)

  add_executable(rgbd_tum
  Examples/RGB-D/rgbd_tum.cc)
  target_link_libraries(rgbd_tum ${PROJECT_NAME})

  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Stereo)

  add_executable(stereo_kitti
  Examples/Stereo/stereo_kitti.cc)
  target_link_libraries(stereo_kitti ${PROJECT_NAME})

  add_executable(stereo_euroc
  Examples/Stereo/stereo_euroc.cc)
  target_link_libraries(stereo_euroc ${PROJECT_NAME})


  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Monocular)

  add_executable(mono_tum
  Examples/Monocular/mono_tum.cc)
  target_link_libraries(mono_tum ${PROJECT_NAME})

  add_executable(mono_kitti
  Examples/Monocular/mono_kitti.cc)
  target_link_libraries(mono_kitti ${PROJECT_NAME})

  add_executable(mono_euroc
  Examples/Monocular/mono_euroc.cc)
  target_link_libraries(mono_euroc ${PROJECT_NAME})
endif()
