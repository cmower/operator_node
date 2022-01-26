// BSD 2-Clause License
// Copyright (c) 2021, Christopher E. Mower
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

class OperatorNode
{

private:

  double max_velocity;
  std::vector<int> axis;
  std::vector<double> directions;  
  ros::Publisher pub;
  ros::Subscriber sub;

public:

  OperatorNode(int argc, char **argv);
  ~OperatorNode();

  std::vector<double> normalize(std::vector<double> &h);
  void callback(const sensor_msgs::Joy::ConstPtr& msgin);

};

OperatorNode::OperatorNode(int argc, char **argv)
{

  // Setup
  ros::init(argc, argv, "operator_node");

  // Get parameter
  ros::NodeHandle nh_param("~");  
  nh_param.param("axis", axis, axis);
  nh_param.param("directions", directions, directions);  
  nh_param.param<double>("max_velocity", max_velocity, 1.0);
  
  // Setup publisher and subscriber
  ros::NodeHandle nh;  
  pub = nh.advertise<std_msgs::Float64MultiArray>("operator_node/signal", 1000);
  sub = nh.subscribe("joy", 1000, &OperatorNode::callback, this);

}

OperatorNode::~OperatorNode()
{
  sub.shutdown();
}

void OperatorNode::callback(const sensor_msgs::Joy::ConstPtr& msgin)
{

  // Get data
  std::vector<double> h(axis.size());
  for (int i=0; i<axis.size(); ++i) {
    h[i] = directions[i]*msgin->axes[axis[i]];
  }
    
  // Pack and publish message
  std_msgs::Float64MultiArray msgout;
  std::vector<double> hnorm = normalize(h);
  msgout.data.clear();
  msgout.data.insert(msgout.data.end(), hnorm.begin(), hnorm.end());
  pub.publish(msgout);

}

std::vector<double> OperatorNode::normalize(std::vector<double> &h)
{

  std::vector<double> hout(h.size());

  // Compute ||h|| and scale=max_vel*min(||h||, 1)/||h||
  double hnorm=0.0;
  for (int i=0; i<h.size(); ++i)
    hnorm += h[i]*h[i];
  hnorm = std::sqrt(hnorm);
  const double scale = max_velocity*std::min(hnorm, 1.0)/hnorm;

  // Compute hout
  if (hnorm > 0) {
    for (int i=0; i<h.size(); ++i)
      hout[i] = scale*h[i];
  }
  else {
    std::fill(hout.begin(), hout.end(), 0);
  }
  
  return hout;
}

int main(int argc, char **argv)
{
  OperatorNode operator_node = OperatorNode(argc, argv);
  ros::spin();
  return 0;
}
