#!/usr/bin/env python3
import rospy
import numpy
from sensor_msgs.msg import Joy
from ros_helper.node import RosNode

class Node(RosNode):

    def __init__(self):
        RosNode.__init__(rospy)
        self.initNode('scale_joy_axes_node')
        self.onShutdownUseBaseShutdownMethod()
        self.getParams([('~axes')])
        n_axes = len(self.params['~axes'])
        self.getParam([
            ('~lower_joy', -1.0*numpy.ones(n_axes)),
            ('~upper_joy',  1.0*numpy.ones(n_axes)),
            ('~lower_operator', -1.0*numpy.ones(n_axes)),
            ('~upper_operator',  1.0*numpy.ones(n_axes)),
        ])
        self.lower_joy = numpy.asarray(self.params['~lower_joy'])
        self.upper_joy = numpy.asarray(self.params['~upper_joy'])
        self.lower_operator = numpy.asarray(self.params['~lower_operator'])
        self.upper_operator = numpy.asarray(self.params['~upper_operator'])
        self.diff_joy = self.upper_joy - self.lower_joy
        self.setupFloat64MultiArrayPublisher('output', 'operator')
        self.subs['input'] = rospy.Subscriber('joy', Joy, self.callback)

    def callback(self, msg):
        joy = numpy.array([msg.axes[m] for m in self.params['~axes']])
        scale = (joy - self.lower_joy) / self.diff_joy
        self.publishFloat64MultiArray(
            'output', (1-scale)*self.lower_operator + scale*self.upper_operator
        )

if __name__ == '__main__':
    Node().spin()
