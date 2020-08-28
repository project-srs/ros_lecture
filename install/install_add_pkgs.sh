#!/bin/sh

# basic
sudo apt install -y ros-${ROS_DISTRO}-joy
sudo apt install -y ros-${ROS_DISTRO}-joystick-drivers

# vis
sudo apt install -y ros-${ROS_DISTRO}-urdf-tutorial 

# sim
sudo apt install -y ros-${ROS_DISTRO}-gazebo-ros-pkgs
sudo apt install -y ros-${ROS_DISTRO}-gazebo-ros-control
sudo apt install -y ros-${ROS_DISTRO}-ros-control
sudo apt install -y ros-${ROS_DISTRO}-ros-controllers

# adv
sudo apt install -y ros-${ROS_DISTRO}-robot-pose-publisher 

# info
sudo apt install -y ros-${ROS_DISTRO}-jsk-visualization
sudo apt install -y ros-${ROS_DISTRO}-robot-pose-publisher

# camera
sudo apt install -y ros-${ROS_DISTRO}-uvc-camera
sudo apt install -y ros-${ROS_DISTRO}-image-transport
sudo apt install -y ros-${ROS_DISTRO}-image-transport-plugins
sudo apt install -y ros-${ROS_DISTRO}-camera-calibration
sudo apt install -y ros-${ROS_DISTRO}-image-proc
sudo apt install -y ros-${ROS_DISTRO}-opencv-apps

# web
sudo apt install -y ros-${ROS_DISTRO}-roswww
sudo apt install -y ros-${ROS_DISTRO}-rosbridge-suite 
sudo apt install -y ros-${ROS_DISTRO}-web-video-server

# nav
sudo apt install -y ros-${ROS_DISTRO}-robot-localization
sudo apt install -y ros-${ROS_DISTRO}-gmapping
sudo apt install -y ros-${ROS_DISTRO}-amcl
sudo apt install -y ros-${ROS_DISTRO}-map-server
sudo apt install -y ros-${ROS_DISTRO}-move-base

# arm
sudo apt install -y ros-${ROS_DISTRO}-moveit
sudo apt install -y ros-${ROS_DISTRO}-moveit-ros-visualization

# hardware
sudo apt install -y ros-${ROS_DISTRO}-rosserial
sudo apt install -y ros-${ROS_DISTRO}-rosserial-arduino

# other (not ROS)
sudo apt install -y chrony lpc21isp 
sudo apt install -y libarmadillo-dev libarmadillo6

# pip
sudo apt install -y python-pip
pip install jinja2