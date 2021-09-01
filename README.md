# operator_node

The `operator_node` package performs parsing from raw interface signals to the required format.

Nodes:
* `isometric_node`
* `joy_remap_node`
* `interface_logger_node`
* `operator_interface_logger_visualizer_node.py`, requires [cmower/pygame_teleop](https://github.com/cmower/pygame_teleop).
* `mouse_input_node.py`

See `launch/` for an example.

## Install

1. Clone repository
1. `cd` into `operator_node`
1. `$ rosdep update ; rosdep install --from-paths ./ -iry`
1. Build catkin workspace

To run the example, and and use `operator_interface_logger_visualizer_node.py` you will need to install [cmower/pygame_teleop](https://github.com/cmower/pygame_teleop).
