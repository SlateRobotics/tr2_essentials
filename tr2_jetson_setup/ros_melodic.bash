#!/usr/bin/env bash
echo "Installing ROS melodic packages..."
sudo apt-get update
sudo apt-get install ros-melodic-desktop-full -y
sudo rosdep init
rosdep update
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt-get install python-rosinstall python-rosinstall-generator python-wstool build-essential xboxdrv -y
sudo apt-get install ros-melodic-moveit ros-melodic-joy ros-melodic-ros-control ros-melodic-ros-controllers ros-melodic-gazebo-ros-control ros-melodic-navigation -y
echo "ROS melodic installed"
