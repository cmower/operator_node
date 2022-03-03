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
import tf_conversions
from std_srvs.srv import SetBool, SetBoolResponse
from std_msgs.msg import Float64MultiArray
from rpbi.tf_interface import TfInterface

class Node:

    def __init__(self):
        rospy.init_node('teleop_tf_node')
        self.tf = TfInterface()
        self.parent_frame = rospy.get_param('~parent_frame', 'world')
        self.child_frame = rospy.get_param('~child_frame')
        hz = rospy.get_param('~hz', 100)
        self.dt = 1.0/float(hz)
        self.h = np.zeros(6)
        self.pose = np.zeros(6)  # position + euler angles
        self.timer = None
        self.sub = None
        rospy.Service('toggle_teleop_tf', SetBool, self.toggle_teleop_tf)

    def toggle_teleop_tf(self, req):
        if req.data:
            success, message = self.start_teleop()
        else:
            success, message = self.stop_teleop()
        return SetBoolResponse(message=message, success=success)

    def start_teleop(self):
        if (self.sub is None) and (self.timer is None):
            self.sub = rospy.Subscriber('operator_node/signal', Float64MultiArray, self.callback)
            self.timer = rospy.Timer(rospy.Duration(self.dt), self.main_loop)
            success = True
            message = 'started tf teleoperation node'
        else:
            success = False
            message = 'user attempted to start tf teleoperation node, but it is already running!'
            rospy.logerr(message)
        return success, message

    def stop_teleop(self):
        if (self.sub is not None) and (self.timer is not None):
            self.sub.unregister()
            self.h = np.zeros(6)  # ensure no more motion
            self.timer.shutdown()
            success = True
            message = 'stopped tf teleoperation node'
        else:
            success = False
            message = 'user attempted to stop ttf teleoperation node, but it is not running!'
        return success, message

    def callback(self, msg):
        self.h[:6] = np.array(msg.data[:6])

    def main_loop(self, event):
        self.pose += self.dt*self.h
        self.tf.set_tf(self.parent_frame, self.child_frame, self.pose[:3], tf_conversions.transformations.quaternion_from_euler(self.pose[3], self.pose[4], self.pose[5]))

    def spin(self):
        rospy.spin()


def main():
    Node().spin()


if __name__ == '__main__':
    main()
