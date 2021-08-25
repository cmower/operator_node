#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"

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
  ros::NodeHandle nh_param("~");  
  nh_param.getParam("window_duration", window_duration);
  ros::NodeHandle nh;  
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
  const int num_axes = msg->data.size();
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
