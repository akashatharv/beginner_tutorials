# beginner_tutorials
[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)
---

This is an implementation of ROS beginner tutorials [ROS tutorial](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29)

## Pre-requisite
The project requires ROS kinetic and catkin, and it is developed on UBUNTU 16.04 LTS.

To install ROS kinetic, please follow the tutorial on: 
http://wiki.ros.org/kinetic/Installation/Ubuntu

To install catkin, please follow the tutorial on: 
http://wiki.ros.org/catkin?distro=indigo#Installing_catkin

## How to build
After installing both ROS and initiating your catkin workspace, build the repository using following commands
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
cd src/
git clone https://github.com/akashatharv/beginner_tutorials
cd ..
catkin_make
```
## Running the publisher and subscriber
Open a new terminal and type the following command to start ROS
```
roscore
```

Open a new terminal and type the following commands to start ROS to run the talker node
```
cd catkin_ws
source devel/setup.bash
rosrun beginner_tutorials talker
```

Open a new terminal and type the following commands to start ROS to run the listener node
```
cd catkin_ws
source devel/setup.bash
rosrun beginner_tutorials listener
```

To stop the program press ctrl+C in each of the three terminals.

