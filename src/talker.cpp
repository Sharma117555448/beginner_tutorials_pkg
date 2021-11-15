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
 * @file talker.cpp
 * @author Charu Sharma (charu107@umd.edu)
 * @brief ROS Publisher to publish messages to a topic
 * @version 0.2
 * @date 2021-11-15
 */

// TF Transform Library
#include <tf/transform_broadcaster.h>

// ROS Console
#include <ros/console.h>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include "beginner_tutorials/change_string.h"


// Initializing the string
extern std::string str = "ENPM808X";

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */

/**
 * @brief ROS Service to change the string
 * @param req Service Request 
 * @param res Service Response
 * @return true
 */
bool change(beginner_tutorials::change_string::Request &req,
            beginner_tutorials::change_string::Response &res) {
  // Changing the string
  res.output_string = req.input_string;
  str = res.output_string;
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

  // Create transform broadcast object
  static tf::TransformBroadcaster br;
  tf::Transform transform;

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
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

    /**
 * @brief ROS Service
 * @param change_string service call 
 */
  ros::ServiceServer service = n.advertiseService("change_string", change);
  // int freq = atoi(argv[1]);
  // ros::Rate loop_rate(freq);
  ros::Rate loop_rate(10);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  while (ros::ok()) {
    // ROS_DEBUG_STREAM("Publishing frequency provided- " << freq);

    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    std_msgs::String msg;

    std::stringstream ss;
    ss << str;
    msg.data = ss.str();

    // Adding all the Logging Levels
    // for (int i = 1; ros::ok(); i ++) {
    //   if ((i % 10) == 0) {
    //     ROS_WARN_STREAM(i << " Stats enclosing for each deste");
    //   }
    //   if ((i % 12) == 0) {
    //     ROS_ERROR_STREAM(i << " Stats enclosing for each dozen");
    //   }
    //   if ((i % 30) == 0) {
    //     ROS_FATAL_STREAM(i << " Stats enclosing for each dozen");
    //   }

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */

    ROS_INFO_STREAM(msg.data.c_str() << " " << count);

    chatter_pub.publish(msg);

    // Set translation and rotation for the transform broadcast
    transform.setOrigin(tf::Vector3(sin(ros::Time::now().toSec()),
                                    cos(ros::Time::now().toSec()), 0.0));
    tf::Quaternion q;
    q.setRPY(0, 0, 1.0);
    transform.setRotation(q);

    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),
                                          "parent_frame", "talk_frame"));

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }
  return 0;
}
