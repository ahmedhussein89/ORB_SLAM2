# ${CMAKE_SOURCE_DIR}/cmake_modules/coverage.cmake
add_library(
  coverage
  INTERFACE
)

target_compile_options(
  coverage
  INTERFACE
  -fprofile-arcs
  -ftest-coverage
)

target_link_options(
  coverage
  INTERFACE
  -fprofile-arcs
  -ftest-coverage
)

