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

    HZ = 100
    DT = 1.0/float(HZ)


    def __init__(self):

        # Setup
        rospy.init_node('keyboard_to_joy_node')
        self.pub = rospy.Publisher('joy', Joy, queue_size=10)
        self.msg = Joy()

        # Get params
        self.axes_key_ids = []
        axes = rospy.get_param('~axes', [])
        for a in axes:
            neg_key, pos_key = a.split(' ')
            self.axes_key_ids.append([getattr(Key, 'KEY_'+neg_key), getattr(Key, 'KEY_'+pos_key)])
        self.msg.axes = [0.0]*len(axes)

        self.buttons_key_ids = []
        buttons = rospy.get_param('~buttons', []).split(' ')
        for b in buttons:
            self.buttons_key_ids.append(getattr(Key, 'KEY_'+b))
        self.msg.buttons = [0]*len(buttons)

        # Start subscribers
        rospy.Subscriber('keyboard/keyup', Key, self.callback_keyup)
        rospy.Subscriber('keyboard/keydown', Key, self.callback_keydown)

        # Start timer
        rospy.Timer(rospy.Duration(self.DT), self.main_loop)


    def callback_keyup(self, msg):

        # Handle buttons
        try:
            self.msg.buttons[self.buttons_key_ids.index(msg.code)] = 0
            done = True
        except ValueError:
            done = False

        if done:
            return

        # Handle axes
        for i, a in enumerate(self.axes_key_ids):
            if msg.code == a[0]:
                self.msg.axes[i] += 1.0
                return
            elif msg.code == a[1]:
                self.msg.axes[i] -= 1.0
                return


    def callback_keydown(self, msg):

        # Handle buttons
        try:
            self.msg.buttons[self.buttons_key_ids.index(msg.code)] = 1
            done = True
        except ValueError:
            done = False

        if done:
            return

        # Handle axes
        for i, a in enumerate(self.axes_key_ids):
            if msg.code == a[0]:
                self.msg.axes[i] -= 1.0
                return
            elif msg.code == a[1]:
                self.msg.axes[i] += 1.0
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
