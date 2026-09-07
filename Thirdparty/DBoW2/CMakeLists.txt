cmake_minimum_required(VERSION 3.16)
project(DBoW2)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# -march=native MUST match across ORB_SLAM2, DBoW2 and g2o. It raises
# EIGEN_MAX_ALIGN_BYTES (16 -> 64 on AVX-512), which changes the alignment and size of
# Eigen-containing types such as g2o::SE3Quat. Mixing settings across these libraries
# segfaults inside g2o's optimizer. Toggle it for all three or none.
option(ORBSLAM2_NATIVE_ARCH "Build with -march=native" OFF)
if(ORBSLAM2_NATIVE_ARCH)
  add_compile_options(-march=native)
endif()


set(HDRS_DBOW2
  DBoW2/BowVector.h
  DBoW2/FORB.h 
  DBoW2/FClass.h       
  DBoW2/FeatureVector.h
  DBoW2/ScoringObject.h   
  DBoW2/TemplatedVocabulary.h)
set(SRCS_DBOW2
  DBoW2/BowVector.cpp
  DBoW2/FORB.cpp      
  DBoW2/FeatureVector.cpp
  DBoW2/ScoringObject.cpp)

set(HDRS_DUTILS
  DUtils/Random.h
  DUtils/Timestamp.h)
set(SRCS_DUTILS
  DUtils/Random.cpp
  DUtils/Timestamp.cpp)

find_package(OpenCV 4.0 REQUIRED)

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories(${OpenCV_INCLUDE_DIRS})
add_library(DBoW2 SHARED ${SRCS_DBOW2} ${SRCS_DUTILS})
target_link_libraries(DBoW2 ${OpenCV_LIBS})

