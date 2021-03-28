#!/usr/bin/env python3
import math
import rospy
from operator_node import OperatorNode, main

class Node(OperatorNode):

    tol = 1e-9

    def __init__(self):
        """Initialize node."""
        super().__init__(self, rospy)
        self.nu = float(rospy.get_param('mu'))
        rospy.loginfo(f'[{self.name}] Initialization complete.')

    def update(self, event):
        """Main update loop, maps operator signal to control and publishes the result."""
        h = self.get_h()
        norm_h = math.sqrt(sum(hi*hi for hi in h))
        scaling_factor = float(norm_h > self.tol) * self.nu * min(norm_h, 1.0) / norm_h
        self.publish([scaling_factor * hi for hi in h])


if __name__ == '__main__':
    main(Node())