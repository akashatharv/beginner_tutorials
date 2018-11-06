# beginner_tutorials
[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)
---

This is an advanced implementation of ROS beginner tutorials [ROS tutorial](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29)
Here, a ROS service /customString and a launch file have been created to change the custom string and launch both nodes together respectively

## Pre-requisites
The project requires ROS kinetic and catkin, and it is developed on UBUNTU 16.04 LTS.

To install ROS kinetic, please follow the tutorial on: 
http://wiki.ros.org/kinetic/Installation/Ubuntu

Catkin is normally installed with ROS, If not follow :
http://wiki.ros.org/catkin

To make a catkin workspace: 
http://wiki.ros.org/catkin/Tutorials/create_a_workspace

## How to build
After installing both ROS and initiating your catkin workspace, build the repository using following commands

If catkin workspace was't created previously
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
```
Now clone the repository's branch Week_10HW into your catkin workspace
```
cd ~/catkin_ws/src/
git clone -b Week10_HW https://github.com/akashatharv/beginner_tutorials
cd ..
catkin_make
```
## Running the publisher and subscriber without using comprehensive launch file
Open a new terminal and type the following command to start ROS
```
roscore
```

Open a new terminal and type the following commands to start ROS to run the talker node
```
cd ~/catkin_ws/
source devel/setup.bash
rosrun beginner_tutorials talker <integer frequency value>
```

Open a new terminal and type the following commands to start ROS to run the listener node
```
cd ~/catkin_ws/
source devel/setup.bash
rosrun beginner_tutorials listener
```

To stop the program press ctrl+C in each of the three terminals.

## Running the publisher and subscriber node using launch file
Open a new termincal and type the following commands
```
cd ~/catkin_ws/
source devel/setup.bash
roslaunch beginner_tutorials string.launch frequencyInput:=<interger frequency value>
```
Press ctrl+C in the main window to terminate the execution

## Using ROS Service to change the custom string to be printed
A ROS service has been implemented to change the custom string using command line 
For using this follow the instructions listed above to make sure that both talker and listener nodes are running
Then follow the instructions below
```
source devel/setup.bash
rosservice call /customString "Your custom string"
```
And then observe output

## Visualizing logger messages using rqt_console
If you want to visualize the logger messages using rqt_console open a new terminal and use 
```
rosrun rqt_console rqt_console
```
Make sure that all the nodes are running before you use the above command for proper visualization


