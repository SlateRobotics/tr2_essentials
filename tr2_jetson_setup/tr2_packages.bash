#!/usr/bin/env bash
ROS_WS="~/ros_ws"
echo "Installing TR2 Packages at " $ROS_WS
mkdir $ROS_WS
mkdir $ROS_WS/src
cd $ROS_WS/src
echo "source ~/ros_ws/devel/setup.bash" >> ~/.bashrc
git clone https://github.com/slaterobotics/tr2_essentials
git clone https://github.com/slaterobotics/tr2_xbox_teleop
cd $ROS_WS
catkin_make
source ~/.bashrc
