#!/usr/bin/env python3
import math
import rospy
from operator_node import OperatorNode, main

class Node(OperatorNode):

    def __init__(self):
        """Initialize node."""
        super().__init__(rospy, 'operator_node')
        self.nu = float(rospy.get_param('~nu'))
        rospy.loginfo(f'{self.name}: Initialization complete.')

    def update(self, event):
        """Main update loop, maps operator signal to control and publishes the result."""
        h = self.get_h()
        norm_h = math.sqrt(sum(hi*hi for hi in h))
        try:
            scaling_factor = self.nu * min(norm_h, 1.0) / norm_h
        except ZeroDivisionError:
            scaling_factor = 0.0
        self.publish([scaling_factor * hi for hi in h])


if __name__ == '__main__':
    main(Node())
