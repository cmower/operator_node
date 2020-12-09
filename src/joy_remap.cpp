#include "ros/ros.h"
#include "sensor_msgs/Joy.h"

class RemapNode
{

  sensor_msgs::Joy _joy_msg;
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
  ros::init(argc, argv, "joy_remap_node");
  ros::NodeHandle nh;
  int hz;
  if (~ros::param::get("hz", hz))
    hz = 100;
  double dt=1.0/static_cast<double>(hz);
  pub = nh.advertise<sensor_msgs::Joy>("joy_out", 1000);
  sub = nh.subscribe("joy_in", 1000, &RemapNode::joyReader, this);
  timer = nh.createTimer(ros::Duration(dt), &RemapNode::updateJoy, this);
}

RemapNode::~RemapNode()
{
  sub.shutdown();
}

void RemapNode::joyReader(const sensor_msgs::Joy::ConstPtr& msg)
{
  _joy_msg = *msg;
}

void RemapNode::updateJoy(const ros::TimerEvent& event)
{
  sensor_msgs::Joy msg_out = _joy_msg;
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
