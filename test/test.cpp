/**=============================================================================
 * @file       : test.cpp
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
 * @brief      : Gtest implementation for custom ROS beginner-tutorials package 
 *=============================================================================
 */
#include <gtest/gtest.h>
#include <ros/ros.h>
#include "beginner_tutorials/customString.h"

/**
 * @brief Test to find if the implemented ROS service for changing string works correctly
 * @param TESTSuite Gtest framework
 * @param ROS_SERVICE_CUSTOM_STRING_PASSES Test name
 */

TEST(TestSuite, ROS_SERVICE_CUSTOM_STRING_PASSES) {
  ros::NodeHandle n; //Node Handle Created
  ros::ServiceClient client = n.serviceClient<beginner_tutorials::customString>(
     "customString"); //Client created for accesing the service
  beginner_tutorials::customString srv;
//
  srv.request.input = "Akash";  //Input test string
  client.call(srv);
  EXPECT_EQ(srv.response.output,"Akash"); //Test check
}

/**
 * @brief  main function for calling the test
 * @param  argc number of input arguments
 * @param  argv character array
 * @return int (typically 0 if main function works properly)
 */

int main(int argc, char **argv) {
  ros::init(argc, argv, "tester");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

