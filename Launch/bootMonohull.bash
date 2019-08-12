#!/bin/bash

echo "`date +%s`" > /home/monohull/monohull.txt
sleep 30
source /opt/ros/melodic/setup.bash
source /home/monohull/workspaceRos/devel/setup.bash
echo "`date +%s`" >> /home/monohull/monohull.txt
roslaunch plymouth_internship_2019 sailboatBoot.launch
