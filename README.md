# operator_node

The `operator_node` package performs parsing from raw interface signals to the
required format. See `launch/` for examples.

## Install

1. Clone repository
1. `cd` into `operator_node`
1. `$ rosdep update ; rosdep install --from-paths ./ -iry`
1. Build catkin workspace

# Nodes

## `operator_node_method_one.py`

![equation](https://latex.codecogs.com/gif.latex?f%28h%29%20%3D%20%5Ctext%7Bdiag%7D%28m%29%20&plus;%20b)

### Parameters

_Required_:
* `~m`, [list[float]]: list of scaling terms.
* `~b`, [list[float]]: list of shifting terms.

_Optional_:
* `~joy_to_h_map`, [list[int]]: Indices of `joy.axes`.
* `~flip`, [list[int]]: Flips direction of the input: 1 means flip direction, 0 otherwise. Must be same length as `~joy_to_h_map`.

### Topics

_Subscribes_:
* `joy`, [`sensor_msgs/Joy`]: joystick/gamepad messages

_Publishes_:
* `operator_node/u`, [`std_msgs/Float64MultiArray`]: Operator control commands.

## `operator_node_method_two.py`

![equation](https://latex.codecogs.com/gif.latex?f%28h%29%20%3D%20%5Cnu%5Cfrac%7B%5Cmin%280%2C%201%29h%7D%7B%5C%7Ch%5C%7C%7D)

### Parameters

_Required_:
* `~nu`, [float]: Scaling factor.

_Optional_:
* `~joy_to_h_map`, [list[int]]: Indices of `joy.axes`.
* `~flip`, [list[int]]: Flips direction of the input: 1 means flip direction, 0 otherwise. Must be same length as `~joy_to_h_map`.

### Topics

_Subscribes_:
* `joy`, [`sensor_msgs/Joy`]: joystick/gamepad messages

_Publishes_:
* `operator_node/u`, [`std_msgs/Float64MultiArray`]: Operator control commands.

## `keyboard_to_joy_mapper_node.py`

Maps key up/down messages from `ros-keyboard` package to `sensor_msgs/Joy`
messages.

### `Parameters`

_Optional_:
* `~positive_axes`, [list[int]]: Ordered list of keys to use as positive direction.
* `~negative_axes`, [list[int]]: Ordered list of keys to use as negative direction.
* `~buttons`, [list[int]]: Ordered list of keys to use as buttons.
* `~sampling_rate`, [int]: Sampling frequency.

### `Topics`

_Subscribes_:
* `keyboard/keydown`, [`keyboard/Key`]: Key-down messages.
* `keyboard/keyup`, [`keyboard/Key`]: Key-up messages.

_Publishes_
* `joy`, [`sensor_msgs/Joy`]: keyboard messages as a joystick/gamepad

## joy_remap

When launching a `joy_node`, typically you will specify `~autorepeat_rate` as a
parameter. This parameter determines the rate in Hz at which a joystick that has
a non-changing state will resend the previously sent message. The result is that
the frequency of commands is not the same as the `autorepeat_rate` - typically
it will be slightly higher.

The `joy_remap` node simply subscribes to the raw signals from `joy_node` and
republishes the latest received message at a given frequency. This frequency can
be specified in the launch file as a node parameter. This means that you can
subscribe to joystick messages elsewhere at a known, consistent, frequency - if
you care about that sort of thing.
