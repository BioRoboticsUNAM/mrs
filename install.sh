#!/bin/bash
# Script of basic instalation
sudo apt-get install ros-${ROS_DISTRO}-tf -y
sudo apt-get install ros-${ROS_DISTRO}-image-transport -y
sudo apt-get install ros-${ROS_DISTRO}-cv-bridge -y

# Create a symbolic link to find python3 
if [ "$ROS_DISTRO" == "noetic" ]; then
    echo ROS noetic version detected
    sudo ln -s /usr/bin/python3 /usr/bin/python
elif [ "$ROS_DISTRO" == "kinetic" ]; then
    echo ROS kinetic version detected
else
    echo No ROS version supported!
fi


