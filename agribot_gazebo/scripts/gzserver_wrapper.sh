#!/bin/bash
# Wrapper script to set HOME for Gazebo in sandbox environment
export HOME=/tmp/home
export GAZEBO_HOME=/tmp/home/.gazebo
mkdir -p /tmp/home/.gazebo /tmp/home/.ros/log
exec /opt/ros/humble/lib/gazebo_ros/gzserver "$@"
