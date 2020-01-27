#!/usr/bin/env bash
echo "Installing ROS kinetic packages..."
sudo apt-get update
sudo apt-get install ros-kinetic-desktop-full -y
sudo rosdep init
rosdep update
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt-get install python-rosinstall python-rosinstall-generator python-wstool build-essential xboxdrv -y
sudo apt-get install ros-kinetic-moveit ros-kinetic-joy ros-kinetic-ros-control ros-kinetic-ros-controllers ros-kinetic-gazebo-control ros-kinetic-astra-camera ros-kinetic-astra-launch -y
echo "ROS kinetic installed"
