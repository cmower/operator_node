#!/usr/bin/env python3
# BSD 2-Clause License
# Copyright (c) 2022, Christopher E. Mower
# All rights reserved.
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
import sys
import rospy
import numpy as np
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray


class Node:


    def __init__(self):
        rospy.init_node('scale_node')
        self.pub = rospy.Publisher('operator_node/signal', Float64MultiArray, queue_size=10)
        axes = rospy.get_param('~axes')
        self.axes = [int(a) for a in axes.split(' ')]
        scale = rospy.get_param('~scale').split(' ')
        if len(scale) == 1:
            self.scale = np.array(scale*len(self.axes))
        elif len(scale) == len(self.axes):
            self.scale = np.array(scale)
        else:
            rospy.logerr('length of scales parameter is incorrect, got %d, expected %d or 1', len(self.scale), len(self.axes))
            sys.exit(0)
        rospy.Subscriber('joy', Joy, self.callback)


    def callback(self, msg):
        axes = np.array(msg.axes)
        signal = self.scale*axes[self.axes]
        self.pub.publish(Float64MultiArray(data=signal.tolist()))


    def spin(self):
        rospy.spin()


def main():
    Node().spin()


if __name__ == '__main__':
    main()
