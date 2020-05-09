#!/bin/sh

# basic
sudo apt install -y ros-kinetic-joy
sudo apt install -y ros-kinetic-joystick-drivers

# vis
sudo apt install -y ros-kinetic-urdf-tutorial 

# sim
sudo apt install -y ros-kinetic-gazebo-ros-pkgs
sudo apt install -y ros-kinetic-gazebo-ros-control
sudo apt install -y ros-kinetic-ros-control
sudo apt install -y ros-kinetic-ros-controllers

# adv
sudo apt install -y ros-kinetic-robot-pose-publisher 

# info
sudo apt install -y ros-kinetic-jsk-visualization
sudo apt install -y ros-kinetic-robot-pose-publisher

# camera
sudo apt install -y ros-kinetic-uvc-camera
sudo apt install -y ros-kinetic-image-transport
sudo apt install -y ros-kinetic-image-transport-plugins
sudo apt install -y ros-kinetic-camera-calibration
sudo apt install -y ros-kinetic-image-proc
sudo apt install -y ros-kinetic-opencv-apps

# web
sudo apt install -y ros-kinetic-roswww
sudo apt install -y ros-kinetic-rosbridge-suite 
sudo apt install -y ros-kinetic-web-video-server

# nav
sudo apt install -y ros-kinetic-robot-localization
sudo apt install -y ros-kinetic-gmapping
sudo apt install -y ros-kinetic-amcl
sudo apt install -y ros-kinetic-map-server
sudo apt install -y ros-kinetic-move-base

# arm
sudo apt install -y ros-kinetic-moveit
sudo apt install -y ros-kinetic-moveit-ros-visualization

# hardware
sudo apt install -y ros-kinetic-rosserial
sudo apt install -y ros-kinetic-rosserial-arduino

# other (not ROS)
sudo apt install -y chrony
sudo apt install -y libarmadillo-dev libarmadillo6
sudo apt install -y lpc21isp

