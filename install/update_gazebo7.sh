#!/bin/sh

sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
cd /tmp/
wget http://packages.osrfoundation.org/gazebo.key
sudo apt-key add gazebo.key
sudo apt-get update
sudo apt-get install gazebo7

sudo apt-get update
sudo apt-get upgrade
sudo reboot
