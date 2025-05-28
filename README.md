# Turtlebot3 + arenaxr

I dont have a better name for this

## This document should guide you through setting up Robotis’s turtlebot3 with arenaxr

### System Requirements:
Host Computer
- Ubuntu 20.04 LTS with GUI (Focal)
- ROS Noetic
- Turtlebot Packages

Turtlebot
- Ubuntu 20.04 LTS (GUI not required)
- ROS Noetic
- Turtlebot Packages 

Note: Most of this should be installed with the turtlebot download, however, you should ensure these are downloaded by attempting to install this

### Setting up the turtlebot3:
Refer to the turtlebot docs [here](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/)

NOTE: Make sure to select NOETIC at the top of your screen, otherwise, instructions may not be correct. Also, when downloading the turtlebot image, download using the FIRST LINK (8GB version), otherwise, turtlebot may not work properly.

### Running the Code:

1. On the thinkcenter, run roscore
2. On the turtlebot run bringup (roslaunch turtlebot3_bringup turtlebot3_robot.launch)
3. On the thinkcenter, run navigation (roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/<NAME OF MAP FILE>.yaml)
4. Navigate to the folder “Turtlebot3_ARENA” (path: ~/Desktop/Turtlebot3_ARENA)
5. Run python3 groundClick_Final_Final.py
6. Enter the arena scene at arenaxr.org/matthewkibarian/groundClick
7. Click around!

### Things to note:
Turtlebot password: turtlebot 
Thinkcenter password: turtlebot
Need to have a .pgm and .yaml file in the home directory of the host machine
