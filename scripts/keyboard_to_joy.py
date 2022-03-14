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
from sensor_msgs.msg import Joy
from keyboard.msg import Key


class Node:

    def __init__(self):

        # Initialize ROS
        rospy.init_node('keyboard_to_joy_node')

        # Setup publisher
        self.pub = rospy.Publisher('joy', Joy, queue_size=10)

        # Get parameters
        config = rospy.get_param('~config')
        self.axes_neg_code = [getattr(Key, "KEY_%s"%a[0]) for a in config['axes']]
        self.axes_pos_code = [getattr(Key, "KEY_%s"%a[1]) for a in config['axes']]
        self.buttons_code = [getattr(Key, "KEY_%s"%k) for k in config.get('buttons', [])]

        # Setup joy message
        self.msg = Joy(axes=[0.0]*len(config['axes']), buttons=[0]*len(self.buttons_code))

        # Start subscribers
        rospy.Subscriber('keyboard/keyup', Key, self.callback_keyup)
        rospy.Subscriber('keyboard/keydown', Key, self.callback_keydown)

        # Start timer
        dt = 1.0/float(rospy.get_param('~hz', 100))
        rospy.Timer(rospy.Duration(dt), self.main_loop)

        rospy.loginfo('initialized keyboard_to_joy node')

    def _update_buttons(self, msg, value):
        if msg.code in self.buttons_code:
            self.msg.buttons[self.buttons_code.index(msg.code)] = value
            return True
        return False

    def _update_axes(self, msg, axes_code, value):
        if msg.code in axes_code:
            self.msg.axes[axes_code.index(msg.code)] += value
            return True
        return False


    def callback_keyup(self, msg):
        if self._update_buttons(msg, 0):
            return
        if self._update_axes(msg, self.axes_neg_code, 1.0):
            return
        if self._update_axes(msg, self.axes_pos_code, -1.0):
            return

    def callback_keydown(self, msg):
        if self._update_buttons(msg, 1):
            return
        if self._update_axes(msg, self.axes_neg_code, -1.0):
            return
        if self._update_axes(msg, self.axes_pos_code, 1.0):
            return

    def main_loop(self, event):
        self.msg.header.stamp = rospy.Time.now()
        self.pub.publish(self.msg)

    def spin(self):
        rospy.spin()


def main():
    Node().spin()


if __name__ == '__main__':
    main()
