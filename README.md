# operator_node

The `operator_node` package provides tools for mapping raw interface commands to operator signals.

# Nodes

## `joy_remap`

Recieves joy messages from a joy_node and republishes at a given sampling frequency.
Sometimes I have noticed the sampling frequency of a joy_node does not always match the observed frequency (the difference is typically fairly small but significant).

### Parameters

* `~hz` (int, default: 100)

  Sampling frequency.

### Subscribed topics

* `joy_in` ([sensor_msgs/Joy](https://docs.ros.org/en/api/sensor_msgs/html/msg/Joy.html))

  Raw signals from interface driver.

### Published topics

* `joy_out` ([sensor_msgs/Joy](https://docs.ros.org/en/api/sensor_msgs/html/msg/Joy.html))

  Raw signals from interface driver published at given sampling frequency.

## `operator_interface_logger`

This node collects a log of the previous `N` operator signals and
publishes this including timestamps as a flattened `std_msgs/Float64MultiArray`
message. The window duration is given in time (seconds).

You can think of the log as being a `(1+Nd)`-by-`N` array that has been
flattened by columns. `N` is the number of signals in the current
buffer.  `Nd` is the number of dimensions of the operator signal, and
the `1` is for the time stamp.

### Parameters

* `~window_duration` (float)

  The operator signals recieved within this window of time will be published on each topic.

### Subscribed topics

* `operator_node/signal` ([std_msgs/Float64MultiArray](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float64MultiArray.html)

  Operator signals.

### Published topics

* `operator_node/window` ([std_msgs/Float64MultiArray](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float64MultiArray.html)

  A list of operator signals recieved in the previous window of time. Note, a helper function in Python is provided, see example:

  ```python
  import rospy
  import operator_node
  from std_msgs.msg import Float64MultiArray

  Nd = 2  # number of dims for operator signal

  def callback(msg):
      t, h = operator_node.parser.reconstruct_interface_log_msg(msg, Nd)
	  print("-"*70)
	  print(f"{t = }")
	  print(f"{h = }")

  rospy.init_node('test')
  rospy.Subscriber('operator_node/window', Float64MultiArray, callback)
  rospy.spin()
  ```

## `isometric_node.py`

### Parameters

* `~axes` (list[int])

  Indicices of the axes to be used to generate operator signal.

* `~scale`  (either: list[float], float, or str)

  When
    * list[float] must be same length as `~axes`, and is the scale applied in each axes. Note, in this case, all elements in the scale should be equal. It is suggested that you use a single float to represent the scale for this node.
	* float scales each axis
	* str assumes is either a list of floats or a single float (i.e. above applies)

* `~start_on_init` (bool, default: False)

  Start the subscriber on initialization. Otherwise use a service to toggle subscriber on/off.

### Subscribed topics

* `joy` ([sensor_msgs/Joy](https://docs.ros.org/en/api/sensor_msgs/html/msg/Joy.html))

  Interface signals.

### Published topics

* `operator_node/signal` ([std_msgs/Float64MultiArray](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float64MultiArray.html)

  Operator signals.

### Services

* `operator_node/toggle` ([std_srvs/SetBool](http://docs.ros.org/en/api/std_srvs/html/srv/SetBool.html))

  Toggle the operator node on/off.

## `scale_node.py`

### Parameters

* `~axes` (list[int])

  Indicices of the axes to be used to generate operator signal.

* `~scale`  (either: list[float], float, or str)

  When
    * list[float] must be same length as `~axes`, and is the scale applied in each axes
	* float scales each axis
	* str assumes is either a list of floats or a single float (i.e. above applies)

* `~start_on_init` (bool, default: False)

  Start the subscriber on initialization. Otherwise use a service to toggle subscriber on/off.

### Subscribed topics

* `joy` ([sensor_msgs/Joy](https://docs.ros.org/en/api/sensor_msgs/html/msg/Joy.html))

  Interface signals.

### Published topics

* `operator_node/signal` ([std_msgs/Float64MultiArray](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float64MultiArray.html)

  Operator signals.

### Services

* `operator_node/toggle` ([std_srvs/SetBool](http://docs.ros.org/en/api/std_srvs/html/srv/SetBool.html))

  Toggle the operator node on/off.

## `keyboard_to_joy.py`

### Parameters

* `~config` (dict)

  Configuration file, see example in `configs/`.

* `~hz` (int, default: 100)

  Sampling frequency.

### Subscribed topics

* `keyboard/keyup` (keyboard/Key)

  Keyboard key-up events.

* `keyboard/keydown` (keyboard/Key)

  Keyboard key-down events.

### Published topics

* `joy` ([sensor_msgs/Joy](https://docs.ros.org/en/api/sensor_msgs/html/msg/Joy.html))

  Joy messages representing keyboard events.
