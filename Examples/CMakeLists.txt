foreach(example
    RGB-D/rgbd_tum
    Stereo/stereo_kitti
    Stereo/stereo_euroc
    Monocular/mono_tum
    Monocular/mono_kitti
    Monocular/mono_euroc)
  get_filename_component(name ${example} NAME)
  get_filename_component(dir ${example} DIRECTORY)
  add_executable(${name} ${example}.cc)
  target_link_libraries(${name} PRIVATE ORB_SLAM2::ORB_SLAM2)
  # Gates the std::chrono timing in the examples; drop when apps/ replaces Examples/.
  target_compile_definitions(${name} PRIVATE COMPILEDWITHC11)
  set_target_properties(${name} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/${dir})
endforeach()
