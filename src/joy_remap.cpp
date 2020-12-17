#include "ros/ros.h"
#include "sensor_msgs/Joy.h"

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
  nh_param.param<int>("hz", hz, 100);

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
