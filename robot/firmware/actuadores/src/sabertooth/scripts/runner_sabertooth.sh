#!/bin/bash

export ROBOT=`hostname | sed -e "s/-/_/g"`
export ROS_HOSTNAME=`hostname`.local
export ROS_MASTER_URI=http://$ROS_HOSTNAME:11311

source /opt/ros/melodic/setup.bash
#echo 1
source /opt/tfbot/setup.bash
#echo 2
#source /home/ubuntu/tfbot/robot/firmware/actuadores/devel/setup.bash

#echo `env`
echo "Corriendo"
rosrun sabertooth sabertooth_nodo __ns:=$ROBOT
