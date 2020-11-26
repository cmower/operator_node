# operator_node

The `operator_node` listens to `sensor_msgs/Joy` messages on the `operator_signal/raw` topic, and publishes the parsed data in a `sensor_msgs/Joy` message on the `operator_signal` topic.

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

* `max_velocity` [`double`], Maximum velocity in 2D plane [m/s].


Test
