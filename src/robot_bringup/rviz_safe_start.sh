#!/bin/bash
killall -9 rviz > /dev/null 2>&1

export LIBGL_ALWAYS_SOFTWARE=1

roslaunch robot_bringup loam_livox.rviz
