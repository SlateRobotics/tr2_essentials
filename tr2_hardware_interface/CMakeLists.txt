cmake_minimum_required(VERSION 2.8.3)
project(tr2_hardware_interface)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  hardware_interface
  controller_manager
  roscpp
  control_msgs
  trajectory_msgs
  actionlib
  pluginlib
  transmission_interface
  urdf
  control_toolbox
  joint_limits_interface
	tr2cpp
)

catkin_package(
  INCLUDE_DIRS 
    include
  CATKIN_DEPENDS 
    hardware_interface
    controller_manager
    roscpp
    control_msgs
    trajectory_msgs
    pluginlib
    transmission_interface
    urdf
    control_toolbox
    joint_limits_interface
		tr2cpp
  LIBRARIES 
    tr2_hardware_interface
		tr2cpp
)

include_directories(
  include/
  ${catkin_INCLUDE_DIRS})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)

add_library(tr2_hardware_interface src/tr2_hardware_interface.cpp)
add_dependencies(tr2_hardware_interface tr2cpp)
target_link_libraries(tr2_hardware_interface
	${tr2cpp_LIBRARIES}
	${catkin_LIBRARIES})

add_executable(tr2_hardware_interface_node src/tr2_hardware_interface_node.cpp)
add_dependencies(tr2_hardware_interface_node tr2_hardware_interface)

target_link_libraries(tr2_hardware_interface_node
	tr2_hardware_interface
	${catkin_LIBRARIES})
