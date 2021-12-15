#!/usr/bin/env python3
import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Joy

"""
operator_signal_scaling_node

Computes operator signals.
"""

class Node:

    def __init__(self):

        # Initialize node
        rospy.init_node('operator_signal_scaling_node')

        # Get axes (required)
        axes = rospy.get_param('~axes')  # required variable as string
        self.axes = [int(a) for a in axes.split(' ')]  # list of ints separated by spaces

        # Get scaling terms (optional, defaults to 1s)
        scale_default = ' '.join(['1.0']*len(self.axes))  # i.e. ones - in format as param is expected from user
        scale = rospy.get_param('~scale', scale_default)  # optional variable as string
        self.scale = np.array([float(s) for s in scale.split(' ')])  # list of floats separated by spaces

        # Create ROS publisher and subscriber
        self.pub = rospy.Publisher('operator_node/signal', Float64MultiArray, queue_size=10)
        rospy.Subscriber('joy', Joy, self.callback)

    def callback(self, msg):
        joy_axes = np.array(msg.axes)
        operator_signal = self.scale*joy_axes[self.axes]
        self.pub.publish(Float64MultiArray(data=operator_signal))

    def spin(self):
        rospy.spin()


def main():
    Node().spin()


if __name__ == '__main__':
    main()
