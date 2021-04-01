#!/usr/bin/env python3
import rospy
from operator_node import OperatorNode, main

class Node(OperatorNode):

    def __init__(self):
        """Initialize node."""
        super().__init__(self, rospy, 'operator_node')
        self.m = [float(mi) for mi in rospy.get_param('~m')]
        self.b = [float(bi) for bi in rospy.get_param('~b')]
        rospy.loginfo(f'{self.name}: Initialization complete.')

    def f(self, h):
        """Mapping."""
        return [
            h*self.m[i] + self.b[i]
            for i, h in enumerate(self.get_h())
        ]


if __name__ == '__main__':
    main(Node())
