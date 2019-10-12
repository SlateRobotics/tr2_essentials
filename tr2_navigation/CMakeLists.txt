cmake_minimum_required(VERSION 2.8.3)
project(tr2_navigation)

find_package(catkin REQUIRED COMPONENTS
  move_base
  tr2py
  rospy
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS src/tr2_navigation/__init__.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
