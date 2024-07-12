#!/bin/bash
# Script of basic instalation
sudo apt-get install ros-${ROS_DISTRO}-tf -y
sudo apt-get install ros-${ROS_DISTRO}-image-transport -y
sudo apt-get install ros-${ROS_DISTRO}-cv-bridge -y

# If you are using python3 these lines create a symbolic
# link to find python3 into /usr/bin/python
if command -v python3 &>/dev/null; then
    echo Python 3 detected
    sudo ln -s /usr/bin/python3 /usr/bin/python
elif command -v python &>/dev/null; then
    echo Python 2 detected
else 
    echo "No Python detected"
    exit 1
fi


