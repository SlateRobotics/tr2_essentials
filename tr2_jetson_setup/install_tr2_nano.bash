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

./ros_base.bash
./ros_melodic.bash
./tr2_packages.bash
./orbecc_packages.bash

echo "Configuring environment"
sudo cp local.rules /etc/udev/rules.d/
sudo cp orbbec-usb.rules /etc/udev/rules.d/

./kernel_drivers.bash

echo "Updating Packages"
sudo apt-get upgrade -y
sudo apt-get dist-upgrade -y

echo "Install complete. Please reboot the device"
