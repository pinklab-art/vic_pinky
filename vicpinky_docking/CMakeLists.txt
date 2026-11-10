cmake_minimum_required(VERSION 3.8)
project(vicpinky_docking)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config models worlds urdf rviz behavior_trees
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  scripts/make_marker.py
  scripts/make_checkerboard.py
  scripts/bt_viewer.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
