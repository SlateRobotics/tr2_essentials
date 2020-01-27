#!/usr/bin/env bash
echo "Installing Orbbec Packages"
cd ~/ros_ws/src
sudo apt-get install ros-*-rgbd-launch ros-*-libuvc ros-*-libuvc-camera ros-*-libuvc-ros
git clone https://github.com/orbbec/ros_astra_camera
git clone https://github.com/orbbec/ros_astra_launch
cd ~/ros_ws
catkin_make
source ~/.bashrc
roscd astra_camera
./scripts/create_udev_rules
echo "Orbbec packages installed"
