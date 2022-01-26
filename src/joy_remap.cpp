// BSD 2-Clause License
// Copyright (c) 2022, Christopher E. Mower
// All rights reserved.
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 1. Redistributions of source code must retain the above copyright notice, this
//    list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"

/**

Recieves joy messages from a joy_node and republishes at a given
sampling frequency. Sometimes I have noticed the sampling frequency of
a joy_node does not always match the observed frequency (the
difference is typically fairly small but significant).

 **/

class RemapNode
{

  sensor_msgs::Joy joy_msg;
  ros::Publisher pub;
  ros::Subscriber sub;
  ros::Timer timer;

public:

  RemapNode(int argc, char **argv);
  ~RemapNode();

  void joyReader(const sensor_msgs::Joy::ConstPtr& msg);
  void updateJoy(const ros::TimerEvent& event);
  void spin();

};

RemapNode::RemapNode(int argc, char **argv)
{

  // Setup
  ros::init(argc, argv, "joy_remap_node");
  ros::NodeHandle nh_param("~");
  ros::NodeHandle nh;
  int hz;

  // Get parameters
  nh_param.param<int>("sampling_rate", hz, 100);

  // Setup publisher
  pub = nh.advertise<sensor_msgs::Joy>("joy_out", 1000);

  // Wait for first message
  sensor_msgs::Joy::ConstPtr first_msg = ros::topic::waitForMessage<sensor_msgs::Joy>("joy_in");
  joyReader(first_msg);

  // Start subscribing and start the timer
  sub = nh.subscribe("joy_in", 1000, &RemapNode::joyReader, this);
  timer = nh.createTimer(ros::Duration(1.0/static_cast<double>(hz)), &RemapNode::updateJoy, this);
}

RemapNode::~RemapNode()
{
  timer.stop();
  sub.shutdown();
}

void RemapNode::joyReader(const sensor_msgs::Joy::ConstPtr& msg)
{
  joy_msg = *msg;
}

void RemapNode::updateJoy(const ros::TimerEvent& event)
{

  // Grab current message
  sensor_msgs::Joy msg_out = joy_msg;

  // Update time stamp and publish
  msg_out.header.stamp = ros::Time::now();
  pub.publish(msg_out);

}

void RemapNode::spin()
{
  ros::spin();
}

int main(int argc, char **argv)
{
  RemapNode remap_node = RemapNode(argc, argv);
  remap_node.spin();
  return 0;
}
