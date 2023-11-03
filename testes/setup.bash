#!/bin/bash

cd $HOME/src/PX4-Autopilot
#DONT_RUN=1 make px4_sitl_default gazebo
source ~/catkin_ws/devel/setup.bash
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo
roslaunch px4 mavros_posix_sitl.launch &
roslaunch aruco_description spawn_aruco.launch aruco_dictionary:="DICT_4X4_50" aruco_id:="1" aruco_description_name:="teste"
