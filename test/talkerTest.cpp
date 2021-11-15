/**
 * @file main.cpp
 * @brief This files runs all the tests
 * @author Charu Sharma 
 * @license MIT 
 * @version 0.1
 * @date 2021-11-15
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include "beginner_tutorials/change_string.h"
#include "std_msgs/String.h"
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <ros/service_client.h>

/**
 * @brief test existance of service
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

  // EXPECT_TRUE(client.waitForExistence(ros::Duration(2)));
}
/**
 * @brief  tests if changeBaseString service replaces default text with user
 * input
 */
// TEST(testTalkerNode, testTalkerDefaultMessageUpdate) {
//   ros::NodeHandle n;
//   auto client = n.serviceClient<beginner_tutorials::change_string>(
//       "change");
//   beginner_tutorials::change_string srv;
//   // change input string
//   srv.request.input_string = "testString";
//   client.call(srv.request, srv.response);
//   EXPECT_STREQ("testString", srv.response.output_string.c_str());
// }


int main(int argc, char** argv) {
    ros::init(argc, argv, "talkerTest");
    testing::InitGoogleTest(&argc, argv);
    ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}

