# Modified forks, not upstream. -march=native is inherited from the top-level
# CMakeLists.txt so all three libraries agree on EIGEN_MAX_ALIGN_BYTES.
add_subdirectory(DBoW2)
add_subdirectory(g2o)
