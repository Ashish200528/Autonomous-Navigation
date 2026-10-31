#!/usr/bin/env bash
# Source this from a project shell; it deliberately does not modify ~/.bashrc.
if [[ ! -r /opt/ros/jazzy/setup.bash ]]; then
  echo "ROS 2 Jazzy is not installed at /opt/ros/jazzy." >&2
  return 1
fi
source /opt/ros/jazzy/setup.bash
