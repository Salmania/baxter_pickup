#!/bin/bash
export ROS_MASTER_URI=http://011305P0009.local:11311
export ROS_HOSTNAME=`/sbin/ifconfig eth0 | grep 'inet addr:' | cut -d: -f2 | awk '{ print $1}'`