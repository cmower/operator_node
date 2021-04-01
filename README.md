# operator_node

The `operator_node` package performs parsing from raw interface signals to the required format.

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
* `~sampling_rate`, [int]: Sampling frequency.
* `~joy_to_h_map`, [list[int]]: Indices of `joy.axes`.
* `~flip`, [list[int]]: Flips direction of the input: 1 means flip direction, 0 otherwise. Must be same length as `~joy_to_h_map`.

### Topics

## `operator_node_method_two.py`

![equation](https://latex.codecogs.com/gif.latex?f%28h%29%20%3D%20%5Cnu%5Cfrac%7B%5Cmin%280%2C%201%29h%7D%7B%5C%7Ch%5C%7C%7D)

### Parameters

_Required_:
* `~nu`, [float]: Scaling factor.

_Optional_:
* `~sampling_rate`, [int]: Sampling frequency.
* `~joy_to_h_map`, [list[int]]: Indices of `joy.axes`.
* `~flip`, [list[int]]: Flips direction of the input: 1 means flip direction, 0 otherwise. Must be same length as `~joy_to_h_map`.


