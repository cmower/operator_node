#!/usr/bin/env python3
import rospy
import numpy
from sensor_msgs.msg import Joy
from ros_helper.node import RosNode

class Node(RosNode):

    def __init__(self):
        RosNode.__init__(rospy)
        self.initNode('isometry_two_axes_node')
        self.onShutdownUseBaseShutdownMethod()
        self.getParams([
            ('~axis_0', 0),
            ('~axis_1', 1),
            ('~max_velocity', 1),
        ])
        self.axes = [
            self.params['~axis_0'],
            self.params['~axis_1'],
        ]
        self.setupFloat64MultiArrayPublisher('output', 'operator')
        self.subs['input'] = rospy.Subscriber('joy', Joy, self.callback)

    def callback(self, msg):
        h = numpy.array([msg.axes[a] for a in self.axes])
        theta = numpy.arctan2(h[1], h[0])
        scale = self.params['max_velocity']*min(numpy.linalg.norm(h), 1)
        self.publishFloat64MultiArray(
            'output', [scale*numpy.cos(theta), scale*numpy.sin(theta)]
        )

if __name__ == '__main__':
    Node().spin()
