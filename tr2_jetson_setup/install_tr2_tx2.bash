#!/usr/bin/env bash

while true; do
	read -p "You must enable main, universe, restricted, and multiverse repositories in the Software & Updates program to continue. If you try to run this without doing so, you will have a bad time. Have you completed this? [y/n] `echo $'\n> '`" yn
	case $yn in
		[Yy]* ) break;;
		[Nn]* ) exit;;
		* ) echo "Please answer yes or no.";;
	esac
done

sudo apt-get update

echo "Installing ROS Kinetic"
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo -E apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

sudo apt-get update
sudo apt-get install ros-kinetic-desktop-full -y
sudo rosdep init
rosdep update
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt-get install python-rosinstall python-rosinstall-generator python-wstool build-essential xboxdrv -y
sudo apt-get install ros-kinetic-moveit ros-kinetic-joy ros-kinetic-ros-control ros-kinetic-ros-controllers ros-kinetic-gazebo-control ros-kinetic-astra-camera ros-kinetic-astra-launch -y

echo "Installing TR2 Packages"
mkdir ~/ros_ws
mkdir ~/ros_ws/src
cd ~/ros_ws/src
echo "source ~/ros_ws/devel/setup.bash" >> ~/.bashrc
git clone https://github.com/slaterobotics/tr2_essentials
git clone https://github.com/slaterobotics/tr2_xbox_teleop
cd ~/ros_ws
catkin_make
source ~/.bashrc

echo "Configuring environment"
sudo cp local.rules /etc/udev/rules.d/
sudo cp orbbec-usb.rules /etc/udev/rules.d/

echo "Installing Kernel USB-UART Drivers"
mkdir ~/Documents/Github
cd ~/Documents/Github
git clone https://github.com/jetsonhacks/installACMModule
cd installACMModule
./installCH341.sh
./installCP210x.sh
./installCDCACM.sh

echo "Updating Packages"
sudo apt-get upgrade -y
sudo apt-get dist-upgrade -y

echo "Install complete. Please reboot the device"
