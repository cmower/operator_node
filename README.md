# operator_node

## Usage

Launch using
```
$ roslaunch operator_node run.launch
```
Unless commented out, a [`joy_node`](http://wiki.ros.org/joy) is started too. 

## Parameters

* `input_topic` [`string`], Input topic for joystick/gamepad of type sensor_msgs/Joy.
* `max_velocity` [`double`], Maximum velocity in 2D plane [m/s].

## Input topics

* `sensor_msgs/Joy` messages from gamepad/joystick. Topic name specified by parameter `input_topic`.

## Output topics

* `operator_signal/raw` [`std_msgs/Float64MultiArray`], array of length 2 containing raw human input in range [-1, 1], values are unit-less.
* `operator_signal/velocity` [`std_msgs/Float64MultiArray`], array length 2 containing velocity human input in range scaled by maximum velocity, values in m/s.
