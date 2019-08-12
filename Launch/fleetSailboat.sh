#!/bin/sh

source /home/sailboats/workspaceRos/devel/setup.bash
source /opt/ros/melodic/setup.bash

roscore &
rosrun plymouth_internship_2019 fleetSailboat &
