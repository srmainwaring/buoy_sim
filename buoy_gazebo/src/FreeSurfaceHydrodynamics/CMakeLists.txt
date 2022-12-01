add_subdirectory(lib)

gz_add_plugin(WaveBodyInteractions
  SOURCES
    WaveBodyInteractions.cpp
  INCLUDE_DIRS
    ..
    lib
  EXTRA_ROS_PKGS
    simple_interp
    gnuplot-iostream
)

target_link_libraries(WaveBodyInteractions PUBLIC FreeSurfaceHydrodynamics)
