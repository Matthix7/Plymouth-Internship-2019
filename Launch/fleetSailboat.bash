#!/bin/bash

echo "`date +%s`" > /home/trimaran/trimaran.txt
sleep 30
source /opt/ros/melodic/setup.bash
source /home/trimaran/Documents/workspace/devel/setup.bash
echo "`date +%s`" >> /home/trimaran/trimaran.txt
roslaunch plymouth_internship_2019 sailboatBoot.launch
