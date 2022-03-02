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
import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray
from rpbi.tf_interface import TfInterface

class Node:

    hz = 100
    dt = 1.0/float(hz)

    def __init__(self):
        rospy.init_node('teleop_tf_node')
        self.tf = TfInterface()
        self.parent_frame = rospy.get_param('~parent_frame', 'world')
        self.child_frame = rospy.get_param('~child_frame')
        self.h = np.zeros(3)
        self.pos = np.zeros(3)
        rospy.Subscriber('operator_node/signal', Float64MultiArray, self.callback)
        rospy.Timer(rospy.Duration(self.dt), self.main_loop)

    def callback(self, msg):
        self.h[:3] = np.array(msg.data[:3])

    def main_loop(self, event):
        self.pos += self.dt*self.h
        self.tf.set_tf(self.parent_frame, self.child_frame, self.pos)

    def spin(self):
        rospy.spin()


def main():
    Node().spin()


if __name__ == '__main__':
    main()
