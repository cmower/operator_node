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
#include "std_msgs/Float64MultiArray.h"
#include <math.h>
#include <vector>
#include <ros/console.h>

/**

Reads Joy messages, and scales the axes of interest such that result
is isometric.

 **/



class Node
{

private:

  double scale;
  std::vector<int> axis;
  std::vector<double> directions;
  ros::Publisher pub;
  ros::Subscriber sub;

public:

  Node(int argc, char **argv);
  ~Node();
  void callback(const sensor_msgs::Joy::ConstPtr& msgin);

};


Node::Node(int argc, char **argv)
{

  // Setup
  ros::init(argc, argv, "operator_node_isometric_node");

  // Get parameter
  ros::NodeHandle nh_param("~");
  nh_param.param("axis", axis, axis);
  nh_param.param("directions", directions, directions);
  nh_param.param<double>("scale", scale, 1.0);

  // Setup publisher and subscriber
  ros::NodeHandle nh;
  pub = nh.advertise<std_msgs::Float64MultiArray>("operator_node/signal", 1000);
  sub = nh.subscribe("joy", 1000, &Node::callback, this);

  ROS_INFO("operator node: isometric node initialized.");

}


Node::~Node()
{
  sub.shutdown();
}


void Node::callback(const sensor_msgs::Joy::ConstPtr& msgin)
{

  // Specify variables
  std::vector<double> h(axis.size());
  double hnorm=0.0;
  const double output_scale;
  std_msgs::Float64MultiArray msgout;

  // Get data
  for (int i=0; i<axis.size(); ++i) {
    h[i] = directions[i]*msgin->axes[axis[i]];
  }

  // Compute ||h|| and scale=max_vel*min(||h||, 1)/||h||
  for (int i=0; i<h.size(); ++i)
    hnorm += h[i]*h[i];
  hnorm = std::sqrt(hnorm);
  output_scale = scale*std::min(hnorm, 1.0)/hnorm;

  // Compute hnorm
  if (hnorm > 0) {
    for (int i=0; i<h.size(); ++i)
      hnorm[i] = scale*h[i];
  }
  else {
    std::fill(hnorm.begin(), hnorm.end(), 0);
  }

  // Pack and publish message
  msgout.data.clear();
  msgout.data.insert(msgout.data.end(), hnorm.begin(), hnorm.end());
  pub.publish(msgout);

}


int main(int argc, char **argv)
{
  Node node = Node(argc, argv);
  ros::spin();
  return 0;
}
