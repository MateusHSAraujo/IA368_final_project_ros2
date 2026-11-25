#!/bin/bash

source ros2_ws/install/setup.bash && ros2 launch vr_robot_pkg vr_robot_bringup.launch.py map:=scene_house
