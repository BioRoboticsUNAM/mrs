# Mobile Robot Simulator

ToDo: Description

**IMPORTANT:** This repo contains several modules to run a simulator.
If you are only interested in the CLIPS node check [rosCLIPS60](https://github.com/kyordhel/rosCLIPS60).

## Prerrequisites

1. ROS kinetic kame or later
2. Dependencies
    1. `libcgal-dev`
    2. `libncurses5-dev`
    3. `xterm`
    4. `python3-tk` or `python-tk` (depends on ROS distro)
    5. `libopencv-dev`
    6. `libcgal-qt5-dev` (if you are using ROS Kinetic)
## Build

1. Clone inside your `catkin_ws` directory
2. Run the following commands
```
./install.sh
source /opt/ros/«ROS DISTRO»/setup.bash
catkin_make
source devel/setup.bash

## Run with this two options
./start.sh 
## or
roslaunch simulator simulator.launch
```

