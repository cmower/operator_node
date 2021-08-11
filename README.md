# operator_node

The `operator_node` package performs parsing from raw interface signals to the required format. 

Nodes:
* `isometric_node`
* `joy_remap_node`
* `interface_logger_node`

See `launch/` for an example.

## Install

1. Clone repository
1. `cd` into `operator_node`
1. `$ rosdep update ; rosdep install --from-paths ./ -iry`
1. Build catkin workspace
