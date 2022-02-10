[![Build Status](https://travis-ci.org/project-srs/ros_lecture.svg?branch=master)](https://travis-ci.org/project-srs/ros_lecture)

# how to build
prepare Ubuntu20.04

## install ROS Noetic

see [install_ros_noetic.sh](install/install_ros_noetic.sh)

## clone

```shell
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin init
cd ~/catkin_ws/src
git clone https://github.com/project-srs/ros_lecture.git
```

## pre build

```shell
cd ~/catkin_ws/src/ros_lecture
git submodule update --init --recursive
rosdep install -i -y --from-paths ./
```

## build

```shell
cd ~/catkin_ws
catkin build
```

# how to use

see [Qiita Page](https://qiita.com/srs/items/5f44440afea0eb616b4a)
