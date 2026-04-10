#!/bin/bash
export PATH="$HOME/.local/bin:$PATH"
export PYTHONPATH="$HOME/.local/lib/python3.8/site-packages:$PYTHONPATH"


source /opt/ros/noetic/setup.bash
source /home/hyq-/simple_lashingrobot_ws/devel/setup.bash
roslaunch chassis_ctrl run.launch 
exit 0
