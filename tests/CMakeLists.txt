# ${CMAKE_SOURCE_DIR}/tests/CMakeLists.txt
if(ENABLE_UNIT_TESTING)
  add_subdirectory(unit-tests/)
endif()

if(ENABLE_ACCEPTANCES)
  add_subdirectory(acceptances)
endif()

