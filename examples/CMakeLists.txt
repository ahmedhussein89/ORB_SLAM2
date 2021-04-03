# ${CMAKE_SOURCE_DIR}/examples/CMakeLists.txt
if(ENABLE_RGBD)
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/RGB-D)

  add_executable(
    rgbd_tum
    RGB-D/rgbd_tum.cc
  )

  target_link_libraries(
    rgbd_tum
    ${PROJECT_NAME}
  )
endif()

if(ENABLE_STEREO)
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Stereo)

  add_executable(
    stereo_kitti
    Stereo/stereo_kitti.cc
  )

  target_link_libraries(
    stereo_kitti
    ${PROJECT_NAME}
  )

  add_executable(
    stereo_euroc
    Stereo/stereo_euroc.cc
  )

  target_link_libraries(
    stereo_euroc
    ${PROJECT_NAME}
  )
endif()

if(ENABLE_MONOCULAR)
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Monocular)

  if(OFF)
  add_executable(
    mono_tum
    Monocular/mono_tum.cc
  )

  target_link_libraries(
    mono_tum
    ${PROJECT_NAME}
  )

  add_executable(
    mono_kitti
    Monocular/mono_kitti.cc
  )

  target_link_libraries(
    mono_kitti
    ${PROJECT_NAME}
  )
  endif()

  add_executable(
    mono_euroc
    Monocular/mono_euroc.cc
  )

  target_link_libraries(
    mono_euroc
    PRIVATE
    ${PROJECT_NAME}
  )
endif()

