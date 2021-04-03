# ${CMAKE_SOURCE_DIR}/tests/unit-tests/CMakeLists.txt
set(
  testCases
  testConverter
)

find_package(GTest REQUIRED)

foreach(testCase IN LISTS testCases)
  add_executable(
    ${testCase}
    ${testCase}.cpp
  )

  target_link_libraries(
    ${testCase}
    PRIVATE
    ORB_SLAM2
    GTest::GTest
    GTest::Main
    #coverage
  )

  set_target_properties(
    ${testCase}
    PROPERTIES
    CXX_STANDARD          17
    CXX_EXTENSIONS        OFF
    CXX_STANDARD_REQUIRED ON
  )

  add_test(
    ${testCase}
    ${testCase}
  )
endforeach()

