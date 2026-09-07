cmake_minimum_required(VERSION 3.16)
project(DBoW2 LANGUAGES CXX)

find_package(OpenCV 4.0 REQUIRED)

add_library(DBoW2 SHARED
  DBoW2/BowVector.cpp
  DBoW2/FORB.cpp
  DBoW2/FeatureVector.cpp
  DBoW2/ScoringObject.cpp
  DUtils/Random.cpp
  DUtils/Timestamp.cpp
)
target_include_directories(DBoW2 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_features(DBoW2 PUBLIC cxx_std_17)
target_link_libraries(DBoW2 PUBLIC ${OpenCV_LIBS})
