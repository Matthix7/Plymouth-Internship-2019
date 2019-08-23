# Plymouth-Internship-2019
Monohull, Catamaran &amp; Trimaran  
The goal of this project is to make three sailboats (size ≈ 1m for each) able to sail autonomously in one of various sailing modes that an operator can choose amongst waypoints path following, platooning, ...  


## Quick overview
![alt text](https://github.com/Matthix7/plymouth_internship_2019/blob/master/Visuels/Vue%20d'ensemble%201.png "Overview")

## To install and use all the software linked with this work (might not be complete)
1) Create a ROS workspace (`mkdir -p workspaceRos/src/`, `cd workspaceRos`, `catkin_make`). Steps to be followed on sailboats and remote computer.

2) In a terminal under workspaceRos/src, 
 `git clone https://github.com/Matthix7/plymouth_internship_2019`  
 `git clone https://github.com/corentin-j/wrsc_plymouth_2019` (needed only on sailboat)  
 `git clone https://github.com/AlexandreCourjaud/Stage2APlymouth` (needed only on sailboat)  

3) `echo "source ~/workspaceRos/devel/setup.bash" >> ~/.bashrc`

4) `sudo apt update`

5) `sudo apt install python-pip`

6) `sudo apt install python-pyudev`  
   `sudo apt install python-rospkg`  
   `sudo apt install python-picamera` (needed only on sailboat)  
   `sudo apt install ros-melodic-gps-umd`  

7) `sudo pip install pynput` (not needed on sailboat)  
   `sudo pip install pyautogui` (not needed on sailboat)

8) In a terminal under workspaceRos/, type `catkin_make`. You may need several trials.  

9) Reboot, just to be sure.

## Practical use of the package (alone)
1) Create a ROS workspace (`mkdir -p workspaceRos/src/`, `cd workspaceRos`, `catkin_make`).
2) In a terminal under `workspaceRos/src`, type `git clone https://github.com/Matthix7/plymouth_internship_2019`.
3) Under `workspaceRos/`, type `catkin_make` then `. devel/setup.bash`.
4) You now have the package installed in `workspaceRos/src/plymouth_internship_2019`. You will find the scripts relative to the different parts of the project in the folders of `plymouth_internship_2019/src`.

## Projects linked with this work
* https://github.com/corentin-j/wrsc_plymouth_2019/
* https://github.com/AlexandreCourjaud/Stage2APlymouth
* https://github.com/AlexandreArgento/Plymouth-AutonomousSailboat-Catamaran
* https://github.com/Plymouth-Sailboat/SailBoatROS

## Infos Utiles
Ne pas faire de bêtises sous git: https://openclassrooms.com/fr/courses/1233741-gerez-vos-codes-source-avec-git.    
Travail des années précédentes: https://github.com/Plymouth-Sailboat.  
Coder en python avec ROS: http://wiki.ros.org/rospy_tutorials/Tutorials/Makefile.  
