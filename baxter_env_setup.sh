#!/bin/bash
source /opt/ros/hydro/setup.bash 
source ~/ros/baxter_ws/devel/setup.bash
source ~/ros/correll_lab_ws/devel/setup.bash
export ROS_HOSTNAME=`/sbin/ifconfig eth0 | grep 'inet addr:' | cut -d: -f2 | awk '{ print $1}'`
export ROS_MASTER_URI=http://011305P0009.local:11311
export ROSLAUNCH_SSH_UNKNOWN=1