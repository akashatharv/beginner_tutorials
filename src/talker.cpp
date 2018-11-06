/**============================================================================
 * @file       : talker.cpp
 * @author     : Akash Atharv
 * @version    : 1.0
 * @Copyright  : 3-Clause BSD
 * Copyright 2018 Akash Atharv
 * @brief      : ROS publisher created using beginner_tutorials from ROS wiki
 *============================================================================
 */

#include <sstream>
#include <string>
#include "ros/ros.h"
#include "ros/console.h"
#include "std_msgs/String.h"
#include "beginner_tutorials/customString.h"

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */

std::string initialString = "Akash says Go terps!";

bool change(beginner_tutorials::customString::Request &req,
            beginner_tutorials::customString::Response &res) {
      initialString = req.input;
      res.output = initialString;
      ROS_DEBUG_STREAM("Custom String has been changed");
      return true;
}

int main(int argc, char **argv) {
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "talker");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */

  auto service = n.advertiseService("customString",change);

  auto chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  int frequencyInput;
  std::string frequency;
  
  //n.getParam(frequency,frequencyInput);
  frequencyInput = atoi(argv[1]);
  if(frequencyInput == 0) {
        ROS_ERROR_STREAM("Frequency cannot be set to zero, Please enter valid value");
        ROS_DEBUG_STREAM("Default value of frequency is set i.e 10");
        frequencyInput = 10;
  }
  else if(frequencyInput < 3) {
        ROS_WARN_STREAM("Frequency value is less than expected, Please verify");
  }
  
  ros::Rate loop_rate(frequencyInput);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  if (ros::ok())
        ROS_FATAL_STREAM("ROS isn't running properly, Exiting code");

  while (ros::ok()) {
   /**
    * This is a message object. You stuff it with data, and then publish it.
    */
    std_msgs::String msg;
    std::stringstream ss;
    //ss << "Akash says Go terps! " << count;
    ss << initialString<<count;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}
