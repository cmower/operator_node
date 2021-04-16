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

    def f(self, h):
        """Mapping."""
        norm_h = math.sqrt(sum(hi*hi for hi in h))
        theta = math.atan2(h[1], h[0])
        scaling_factor = self.nu * min(norm_h, 1.0)
        return [scaling_factor*math.cos(theta), scaling_factor*math.sin(theta)]
if __name__ == '__main__':
    main(Node())
