# ${CMAKE_SOURCE_DIR}/CMakeLists.txt
cmake_minimum_required(VERSION 3.16)
project(ORB_SLAM2 CXX)

option(ENABLE_UNIT_TESTING "" OFF)
#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -march=native")

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(OpenCV REQUIRED features2d highgui imgcodecs imgproc calib3d)
find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)
#find_package(g2o   REQUIRED)
#find_package(DBoW2 REQUIRED)

add_library(
  ${PROJECT_NAME}
  SHARED
  #src/System.cc
  #src/Tracking.cc
  #src/LocalMapping.cc
  #src/LoopClosing.cc
  #src/ORBextractor.cc
  #src/ORBmatcher.cc
  #src/FrameDrawer.cc
  #src/Converter.cc
  #src/MapPoint.cc
  #src/KeyFrame.cc
  #src/Map.cc
  #src/MapDrawer.cc
  #src/Optimizer.cc
  #src/PnPsolver.cc
  #src/Frame.cc
  #src/KeyFrameDatabase.cc
  #src/Sim3Solver.cc
  #src/Initializer.cc
  #src/Viewer.cc
)

target_sources(
  ${PROJECT_NAME}
  PRIVATE
  src/Converter.cc
)

target_include_directories(
  ${PROJECT_NAME}
  SYSTEM PRIVATE
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
  ${Pangolin_INCLUDE_DIRS}
)

target_link_libraries(
  ${PROJECT_NAME}
  PRIVATE
  ${OpenCV_LIBS}
  ${EIGEN3_LIBS}
  ${Pangolin_LIBRARIES}
  #${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
  #${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
  #g2o
  #DBoW2
)

target_compile_definitions(
  ${PROJECT_NAME}
  PRIVATE
  COMPILEDWITHC11
)

if(ENABLE_UNIT_TESTING)
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

