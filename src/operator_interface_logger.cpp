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
  nh.getParam("window_duration", window_duration);
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
