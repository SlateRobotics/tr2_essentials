#!/usr/bin/env bash
echo "Installing Kernel USB-UART Drivers"
mkdir ~/Documents/Github
cd ~/Documents/Github
git clone https://github.com/jetsonhacks/installACMModule
cd installACMModule
./installCH341.sh
./installCP210x.sh
./installCDCACM.sh
