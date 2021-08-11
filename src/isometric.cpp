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
