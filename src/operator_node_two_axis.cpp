#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/Float64MultiArray.h"

#include<math.h>
#include<algorithm>

class OperatorNode
{
  double direction_x0;
  double direction_x1;
  int axis_x0;
  int axis_x1;
  double max_velocity;
  bool flip_axes;
  ros::Publisher pub_signal;
  ros::Publisher pub_norm;
  ros::Subscriber sub;

public:

  OperatorNode(int argc, char **argv);
  ~OperatorNode();

  void remapJoyToOperatorSignal(const sensor_msgs::Joy::ConstPtr& msg_in);
  void spin();

};

OperatorNode::OperatorNode(int argc, char **argv)
{

  // Setup
  ros::init(argc, argv, "operator_node");
  ros::NodeHandle nh_param("~");
  ros::NodeHandle nh;

  // Get parameters
  nh_param.param<double>("dr_x0", direction_x0, 1);
  nh_param.param<double>("dr_x1", direction_x1, 1);
  nh_param.param<int>("ax_x0", axis_x0, 0);
  nh_param.param<int>("ax_x1", axis_x1, 1);
  nh_param.param<bool>("flip_axes", flip_axes, false);
  nh_param.param<double>("max_velocity", max_velocity, 1.0);

  // Setup publishers
  pub_norm = nh.advertise<std_msgs::Float64MultiArray>("normalized", 1000);
  pub_signal = nh.advertise<std_msgs::Float64MultiArray>("signal", 1000);

  // Setup subscriber
  sub = nh.subscribe("raw", 1000, &OperatorNode::remapJoyToOperatorSignal, this);

}

OperatorNode::~OperatorNode()
{
  sub.shutdown();
}

void OperatorNode::remapJoyToOperatorSignal(const sensor_msgs::Joy::ConstPtr& msg_in)
{

  // Declare variables
  std_msgs::Float64MultiArray msg_out_signal, msg_out_norm;

  // Get raw signal
  const double hr0 = msg_in->axes[axis_x0];
  const double hr1 = msg_in->axes[axis_x1];

  // Normalize
  const double htheta = std::atan2(hr1, hr0);
  const double hscale = std::min(std::sqrt(hr0*hr0 + hr1*hr1), 1.0);
  const double hn0 = hscale * std::cos(htheta);
  const double hn1 = hscale * std::sin(htheta);
  if (flip_axes==true){
    msg_out_norm.data.push_back(direction_x0*hn1);
    msg_out_norm.data.push_back(direction_x1*hn0);
  }
  else {
    msg_out_norm.data.push_back(direction_x0*hn0);
    msg_out_norm.data.push_back(direction_x1*hn1);
  }

  // Compute signal
  msg_out_signal.data.push_back(max_velocity*msg_out_norm.data[0]);
  msg_out_signal.data.push_back(max_velocity*msg_out_norm.data[1]);

  // Publish messages
  pub_norm.publish(msg_out_norm);
  pub_signal.publish(msg_out_signal);
}

void OperatorNode::spin()
{
  ros::spin();
}

int main(int argc, char **argv)
{
  OperatorNode operator_node = OperatorNode(argc, argv);
  operator_node.spin();
  return 0;
}
