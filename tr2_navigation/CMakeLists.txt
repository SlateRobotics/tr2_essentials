cmake_minimum_required(VERSION 2.8.3)
project(tr2_navigation)

find_package(catkin REQUIRED COMPONENTS
  move_base
  tr2py
  rospy
)

catkin_package(
)
