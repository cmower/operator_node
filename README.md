# operator_node

The `operator_node` package performs parsing from raw interface signals to the required format.

Nodes:
* `isometric_node`
* `joy_remap_node`
* `interface_logger_node`
* `operator_interface_logger_visualizer_node.py`
* `mouse_input_node.py`

See `launch/` for an example.

## Requirements

* [cmower/pygame_teleop](https://github.com/cmower/pygame_teleop)

## Install

1. Clone repository
1. `cd` into `operator_node`
1. `$ rosdep update ; rosdep install --from-paths ./ -iry`
1. Build catkin workspace

To run the example, and and use `operator_interface_logger_visualizer_node.py` you will need to install [cmower/pygame_teleop](https://github.com/cmower/pygame_teleop).


# TODO

* Need to revamp https://github.com/cmower/pygame_teleop and make public
* mouse node needs to be updated for pygame_teleop updates
* general cleaning and making sure that everything makes sense
* make several launch files, some examples, but also others that start drivers and operator signal nodes for specific joysticks
* consider whether to merge https://github.com/neemoh/sigma7 inside this repo or to fork and has an external
* deps
