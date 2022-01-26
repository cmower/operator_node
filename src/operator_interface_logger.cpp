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
#include "std_msgs/Float64MultiArray.h"

/**

operator_interface_logger

This node collects a log of the previous N operator signals and
publishes this including timestamps as a flattened Float64MultiArray
message. The window duration is given in time (seconds).

You can think of the log as being a (1+Nd)-by-N array that has been
flattened by columns. N is the number of signals in the current
buffer.  Nd is the number of dimensions of the operator signal, and
the 1 is for the time stamp.

 **/

class LoggerNode
{

private:

  double window_duration;
  ros::Subscriber sub;
  ros::Publisher pub;
  std_msgs::Float64MultiArray msgout;

public:

  LoggerNode(int argc, char **argv);
  ~LoggerNode();
  void updateLog(const std_msgs::Float64MultiArray::ConstPtr& msg);

};

LoggerNode::LoggerNode(int argc, char **argv)
{
  ros::init(argc, argv, "interface_logger_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_param("~");
  nh_param.param("window_duration", window_duration);
  pub = nh.advertise<std_msgs::Float64MultiArray>("operator_node/window", 1000);
  sub = nh.subscribe("operator_node/signal", 1000, &LoggerNode::updateLog, this);
}

LoggerNode::~LoggerNode()
{
  sub.shutdown();
}

void LoggerNode::updateLog(const std_msgs::Float64MultiArray::ConstPtr& msg)
{

  // Get current time
  const int num_axes = msg->data.size(); // assumes this is the same for all operator signals on this topic
  const double tc = ros::Time::now().toSec();

  // Append data
  msgout.data.push_back(tc);
  for (int i=0; i<num_axes; ++i)
    msgout.data.push_back(msg->data[i]);

  // Trim data
  while ((tc - msgout.data[0]) > window_duration) {
    msgout.data.erase(msgout.data.begin(), msgout.data.begin()+num_axes+1);
  }

  // Publish window
  pub.publish(msgout);

}

int main(int argc, char **argv)
{
  LoggerNode loggernode = LoggerNode(argc, argv);
  ros::spin();
  return 0;
}
