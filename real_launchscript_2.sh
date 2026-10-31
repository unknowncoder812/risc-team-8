#!/usr/bin/env bash
colcon build --packages-select line_follower_bot
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash
ros2 pkg prefix line_follower_bot

export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$(ros2 pkg prefix line_follower_bot)/share/line_follower_bot


ros2 launch line_follower_bot spawn_robot.launch.py