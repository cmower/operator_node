# operator_node

## Install

1. Clone repository
1. `cd` into `operator_node`
1. `$ rosdep update ; rosdep install --from-paths ./ -iry`
1. Build catkin workspace

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

* `operator_signal` [`sensor_msgs/Joy`], operator signal
