#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "keyboard/Key.h"

#include "single_key.h"

class KeyboardToJoyNode
{

  SingleKey key_up;
  SingleKey key_down;
  SingleKey key_left;
  SingleKey key_right;
  ros::Publisher pub;
  ros::Subscriber sub_keyupp;
  ros::Subscriber sub_keydwn;
  ros::Timer timer;

public:

  KeyboardToJoyNode(int argc, char** argv);
  ~KeyboardToJoyNode();

  void readKeyUp(const keyboard::Key::ConstPtr& msg);
  void readKeyDown(const keyboard::Key::ConstPtr& msg);
  void updateOperatorSignal(const ros::TimerEvent& event);
  void spin();

};

KeyboardToJoyNode::KeyboardToJoyNode(int argc, char** argv)
{
  ros::init(argc, argv, "keyboard_to_joy");
  key_up = SingleKey(273);
  key_down = SingleKey(274);
  key_left = SingleKey(276);
  key_right = SingleKey(275);
  ros::NodeHandle nh;
  ros::NodeHandle nh_param("~");
  int hz;
  nh_param.param<int>("hz", hz, 100);
  sub_keyupp = nh.subscribe("keyboard/keyup", 1000, &KeyboardToJoyNode::readKeyUp, this);
  sub_keydwn = nh.subscribe("keyboard/keydown", 1000, &KeyboardToJoyNode::readKeyDown, this);
  pub = nh.advertise<sensor_msgs::Joy>("joy_out", 1000);
  timer = nh.createTimer(ros::Duration(1.0/static_cast<double>(hz)), &KeyboardToJoyNode::updateOperatorSignal, this);
}

KeyboardToJoyNode::~KeyboardToJoyNode()
{
  timer.stop();
  sub_keyupp.shutdown();
  sub_keydwn.shutdown();
}

void KeyboardToJoyNode::readKeyUp(const keyboard::Key::ConstPtr& msg)
{
  key_up.releasedKey(msg);
  key_down.releasedKey(msg);
  key_left.releasedKey(msg);
  key_right.releasedKey(msg);
}

void KeyboardToJoyNode::readKeyDown(const keyboard::Key::ConstPtr& msg)
{
  key_up.pressedKey(msg);
  key_down.pressedKey(msg);
  key_left.pressedKey(msg);
  key_right.pressedKey(msg);
}

void KeyboardToJoyNode::updateOperatorSignal(const ros::TimerEvent& event)
{
  double hori = key_left.getValue() - key_right.getValue();
  double vert = key_up.getValue() - key_down.getValue();
  sensor_msgs::Joy msg;
  msg.header.stamp = ros::Time::now();
  msg.axes.push_back(hori);
  msg.axes.push_back(vert);
  pub.publish(msg);
}

void KeyboardToJoyNode::spin()
{
  ros::spin();
}

int main(int argc, char** argv)
{
  KeyboardToJoyNode node = KeyboardToJoyNode(argc, argv);
  node.spin();
  return 0;
}
