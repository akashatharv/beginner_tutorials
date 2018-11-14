# beginner_tutorials
[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)
---

This is an advanced implementation of ROS beginner tutorials [ROS tutorial](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29)
Here, a ROS service /customString and a launch file have been created to change the custom string and launch both nodes together respectively. Additionally a [tf frame](http://wiki.ros.org/tf/Tutorials/Introduction%20to%20tf) 
is also broadcasted. Level2 integration test and rosbag recording for all topics is also implemented

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
git clone -b Week11_HW https://github.com/akashatharv/beginner_tutorials
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
roslaunch beginner_tutorials string.launch frequencyInput:=<integer frequency value>
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

## ROSTF

In addition to previous Week's implementation of the talker node a new ROS tf implementation is added to repository. Here, the talker node broadcasts tf transforms to "\talk" with respect to the "\world" frame. The tf frame can be visualized while the talker node and roscore are running using "tf_echo" in a separate terminal

```
rosrun tf tf_echo /world /talk
```
and also using "view frames" in a separate terminal

```
rosrun tf view_frames
evince frames.pdf
```

which will provide a similar output to the picture below:
<p align="center">
  <img width="800" height="400" src="https://github.com/akashatharv/beginner_tutorials/blob/Week11_HW/Week11_HW_results/rqt_tf_tree_and_tf_echo_output.jpg">
</p>
Additionaly we can also visualize the frames using "rviz" by using

```
rosrun rviz rviz
```

in a separate terminal and selecting the world frame.

## Running rostest

level 2 integration tests are created to test the Talker node using gtest framework.To run the tests run the following commands
```
cd ~/catkin_ws/
catkin_make run_tests
```
The output should be similar to the picture below
<p align="center">
  <img width="800" height="400" src="https://github.com/akashatharv/beginner_tutorials/blob/Week11_HW/Week11_HW_results/test_successful.jpg">
</p>

## Using rosbag 

Rosbag is used to record messages published over any ROS topic. You can launch the nodes as described previously and simply use
```
rosbag record -a
```
in another terminal to record data from all the topics or modify the command to record a specific topic.
Additionally to use the launch file in the repository open a new terminal and type,
```
cd ~/catkin_ws/
source devel/setup.bash
roslaunch beginner_tutorials string.launch status:=true
```
Status is a flag which is false by default which means that rosbag recording is disabled. It can be set to true as shown in commands above to record messages over all topics in a bag file.

Press ctrl+C in the active window to terminate the execution
The resultant bag file can be found in the .ros directory.
To get details regarding the newly created bag file type
```
cd ~/.ros
rosbag info result.bag
```
To visualize the data recorded on the screen, first run roscore and listener node, then navigate to the above directory in a different terminal
```
cd ~/.ros
```
and use rosbag play 
```
rosbag play result.bag
```
