cmake_minimum_required(VERSION 3.16)
project(ORB_SLAM2 LANGUAGES C CXX)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

# Off by default: -march=native makes binaries non-portable across machines.
# It raises EIGEN_MAX_ALIGN_BYTES (16 -> 64 on AVX-512), changing the alignment and
# size of Eigen-containing types such as g2o::SE3Quat, so it must be identical for
# ORB_SLAM2, DBoW2 and g2o -- mixing settings segfaults inside g2o. Setting it here,
# before add_subdirectory(), is what keeps all three in sync.
option(ORBSLAM2_NATIVE_ARCH "Build with -march=native" OFF)
if(ORBSLAM2_NATIVE_ARCH)
  add_compile_options(-march=native)
endif()

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
find_package(OpenCV 4.0 REQUIRED)
find_package(Eigen3 3.1.0 REQUIRED)

add_subdirectory(Thirdparty)
add_subdirectory(src)

add_subdirectory(Examples)
