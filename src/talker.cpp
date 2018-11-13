/**============================================================================
 * @file       : talker.cpp
 * @author     : Akash Atharv
 * @version    : 1.0
 * @Copyright  : 3-Clause BSD
Copyright (c) 2018, Akash Atharv
 
Redistribution and use in source and binary forms, with or without  
modification, are permitted provided that the following conditions are 
met:
 
1. Redistributions of source code must retain the above copyright notice, 
this list of conditions and the following disclaimer.
 
2. Redistributions in binary form must reproduce the above copyright 
notice, this list of conditions and the following disclaimer in the   
documentation and/or other materials provided with the distribution.
 
3. Neither the name of the copyright holder nor the names of its 
contributors may be used to endorse or promote products derived from this 
software without specific prior written permission.
 
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS 
IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, 
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR 
PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR 
CONTRIBUTORS BE 
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF 
THE POSSIBILITY OF SUCH DAMAGE.
 * @brief      : ROS publisher created using beginner_tutorials from ROS wiki
 *============================================================================
 */

#include <sstream>
#include <string>
#include "ros/ros.h"
#include "ros/console.h"
#include "std_msgs/String.h"
#include "beginner_tutorials/customString.h"
#include <tf/transform_broadcaster.h>

// Default message which will be sent over the topic "chatter"
extern std::string initialString = "Akash says Go terps!";

/**
 * @brief change function where the custom string is changed using rosservice
 * @param req New string input by the user using rosservice
 * @param res Message which was previously sent over the topic "chatter"
 * @return boolean (typically true if the function works properly)
 */

bool change(beginner_tutorials::customString::Request &req,
            beginner_tutorials::customString::Response &res) {
      initialString = req.input;  // input string using rosservice
      res.output = initialString;  // output string of the rosservice call
      ROS_DEBUG_STREAM("Custom String has been changed");
      return true;
}

/**
 * @brief Main function where the node "talker" is created and operates
 * @param argc number of input arguments
 * @param argv argument for calling main function
 * @return int (typically 0 if main function works properly)
 */

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

  auto chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  // service call back function "change" is called for the string change
  auto service = n.advertiseService("customString", change);

  int frequencyInput;
  // frequency input by user is read and then set

  frequencyInput = atoi(argv[1]);

  if (frequencyInput == 0) {
  // Error logging message stream
        ROS_ERROR_STREAM("Frequency cannot be zero, Please enter valid value");
  // Debug logging message stream
        ROS_DEBUG_STREAM("Default value of frequency is set i.e 10");
  // Frequency modified as we can't set frequency to be 0
        frequencyInput = 10;
  } else if (frequencyInput < 3) {
  // Warning logging message stream for lower frequency values
        ROS_WARN_STREAM("Frequency value is less than expected, Please verify");
  }

  ros::Rate loop_rate(frequencyInput);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  if (ros::ok())
  // Fatal logging message stream if ROS isn't working properly
        ROS_FATAL_STREAM("ROS isn't running properly, Exiting code");

  static tf::TransformBroadcaster br; // Declaring the tf broadcaster
  tf::Transform transform;            // Declaring the frame
  

  while (ros::ok()) {
   /**
    * This is a message object. You stuff it with data, and then publish it.
    */
    std_msgs::String msg;
    std::stringstream ss;
    ss << initialString<< count;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    chatter_pub.publish(msg);

    transform.setOrigin( tf::Vector3(3.0, 2.0, 1.0) ); // Setting origin of the transform
    transform.setRotation( tf::Quaternion(0, 0, 0, 1) ); // Setting rotation of the transform
    br.sendTransform(tf::StampedTransform(transform,ros::Time::now(),"world","talk")); // Transform Broadcasted

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}
