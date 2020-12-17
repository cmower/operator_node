#include "keyboard/Key.h"

class SingleKey
{

  int code;
  bool pressed;
  double pressed_time;
  double unenabled_value;
  double enabled_base_value;
  double enabled_update_factor;

public:

  SingleKey(){}

  SingleKey(int code_in) {
    code = code_in;
    pressed = false;
    unenabled_value = 0.0;
    enabled_base_value = 0.25;
    enabled_update_factor = 0.025;
  }

  ~SingleKey(){}

  void pressedKey(const keyboard::Key::ConstPtr& msg) {
    if (msg->code == code) {
      pressed = true;
      pressed_time = ros::Time::now().toSec();
    }
  }
  void releasedKey(const keyboard::Key::ConstPtr& msg) {
    if (msg->code == code)
      pressed = false;
  }
  double getValue() {
    double out_value;
    out_value = 0.0;

    if (pressed == true)
      out_value += enabled_base_value + enabled_update_factor * (ros::Time::now().toSec() - pressed_time);

    if (out_value > 1.0)
      out_value = 1.0;

    return out_value;
  }

};
