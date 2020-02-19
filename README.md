# operator_node

## Usage

Launch using
```
$ roslaunch operator_node run.launch
```

## Parameters

* `input_topic` [`string`], Input topic for joystick/gamepad of type sensor_msgs/Joy.
* `max_velocity` [`double`], Maximum velocity in 2D plane [m/s].

## Input topics

* `sensor_msgs/Joy` messages from gamepad/joystick. Topic name specified by parameter `input_topic`.

## Output topics

* `human_input/raw` [`std_msgs/Float64MultiArray`], array of length 2 containing raw human input in range [-1, 1].
* `human_input/velocity` [`std_msgs/Float64MultiArray`], array length 2 containing velocity human input in range scaled by maximum velocity.
