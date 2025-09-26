#!/bin/bash
set -e
wget -q -nv https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O /home/robot/ros-archive-keyring.gpg
sudo cp /home/robot/ros-archive-keyring.gpg /etc/apt/keyrings/ros-archive-keyring.gpg

# Update container
sudo apt-get update
sudo apt-get install -y pkg-config

sudo apt-get install -y git

# To visualize the camera in rviz
sudo apt-get install -y ros-${ROS_DISTRO}-rviz2 ros-${ROS_DISTRO}-image-transport-plugins

# Download dependencies
local_deps.sh

# Retry at least 5 times to install dependencies
counter=0
until rosdep update --rosdistro=${ROS_DISTRO} || [ $counter -eq 5 ]; do
  ((counter++))
  echo "rosdep update failed, retrying"
  sleep 1
done
if [ $counter -eq 5 ]; then
  echo "rosdep update failed, exiting"
  exit 1
fi

git clone https://github.com/RobotnikAutomation/robotnik_common.git /home/robot/robot_ws/src/robotnik_common
git clone https://github.com/RobotnikAutomation/robotnik_interfaces.git /home/robot/robot_ws/src/robotnik_interfaces

rosdep install --from-paths ./src --ignore-src -y -r

compile_workspace.sh
