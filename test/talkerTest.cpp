/************************************************************************
MIT License
Copyright © 2021 Charu Sharma
Permission is hereby granted, free of charge, to any person
obtaining a copy of this software and associated documentation
files (the “Software”), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge,
publish, distribute, sublicense, and/or sell copies of the Software,
and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:
The above copyright notice and this permission notice shall be included 
in all copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS
OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
OTHER DEALINGS IN THE SOFTWARE.
 *************************************************************************/

/**
 * @file talkerTest.cpp
 * @author Charu Sharma (charu107@umd.edu)
 * @brief to test talker.cpp
 * @version 0.2
 * @date 2021-11-15
 */

#include "beginner_tutorials/change_string.h"
#include "std_msgs/String.h"
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <ros/service_client.h>

/**
 * @brief test existance of service
 */
TEST(testTalkerNode, testTalkerDefaultMessageUpdate) {
  ros::NodeHandle n;
  ros::ServiceClient client =
                      n.serviceClient<beginner_tutorials::change_string>(
                                              "change");
  // Check if the client exists
  bool exists(client.waitForExistence(ros::Duration(5.0)));
  EXPECT_TRUE(exists);
}

/**
 * @brief  tests if changeBaseString service replaces default text with user
 * input
 */
TEST(testTalkerNode, test_change_string) {
  ros::NodeHandle n;
  ros::ServiceClient client =
                      n.serviceClient<beginner_tutorials::change_string>(
                        "change");
  beginner_tutorials::change_string srv;
  // change input string
  srv.request.input_string = "testString";
  client.call(srv.request, srv.response);
  EXPECT_STREQ(srv.response.output_string.c_str(), "testString");

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "talkerTest");
    testing::InitGoogleTest(&argc, argv);
    ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}

