#!/usr/bin/env bash
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash
ros2 pkg prefix line_follower_bot

export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$(ros2 pkg prefix line_follower_bot)/share/line_follower_bot

# gz sim -r src/line_follower_bot/worlds/line_world.sdf 
gz sim -r src/line_follower_bot/worlds/final_grading.sdf 