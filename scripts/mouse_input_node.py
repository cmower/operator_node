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
import pygame
from std_msgs.msg import Int64MultiArray, UInt8
from sensor_msgs.msg import Joy

"""
Button ID:
1 - left click
2 - middle click
3 - right click
4 - scroll up
5 - scroll down
"""


class Node:


    HZ = 100


    def __init__(self):

        # Setup node
        rospy.init_node('mouse_input_node')

        # Get params
        self.width = rospy.get_param('~width', 500)
        self.norm = 1.0/float(self.width-1)

        # Setup publishers
        self.mouse_position_pub = rospy.Publisher('mouse/position', Int64MultiArray, queue_size=10)
        self.mouse_button_down_event_pub = rospy.Publisher('mouse/buttondown', UInt8, queue_size=10)
        self.mouse_button_up_event_pub = rospy.Publisher('mouse/buttonup', UInt8, queue_size=10)
        self.mouse_joy_pub = rospy.Publisher('mouse/joy', Joy, queue_size=10)

        # Setup pygame window
        pygame.init()
        self.screen = pygame.display.set_mode((self.width, self.width))
        pygame.display.set_caption('Mouse Input')
        self.clock = pygame.time.Clock()
        self.screen.fill(pygame.Color('white'))
        self.running = True
        self.msg = Joy()
        self.msg.axes = [0.0]*2
        self.msg.buttons = [0]*5

        rospy.loginfo('initialized mouse input node')


    def spin(self):

        while self.running:

            for event in pygame.event.get():

                if event.type == pygame.QUIT:
                    self.running = False

                elif event.type == pygame.MOUSEBUTTONUP:
                    self.mouse_button_up_event_pub.publish(UInt8(data=event.button))
                    self.msg.buttons[event.button-1] = 0

                elif event.type == pygame.MOUSEBUTTONDOWN:
                    self.mouse_button_down_event_pub.publish(UInt8(data=event.button))
                    self.msg.buttons[event.button-1] = 1

            mouse_pos = pygame.mouse.get_pos()
            self.mouse_position_pub.publish(Int64MultiArray(data=mouse_pos))
            self.msg.axes = [self.norm*float(m) for m in mouse_pos]
            self.msg.header.stamp = rospy.Time.now()
            self.mouse_joy_pub.publish(self.msg)

            # Tick pygame
            pygame.display.flip()
            self.clock.tick_busy_loop(self.HZ)

        rospy.loginfo('shutting down mouse input node')
        pygame.quit()
        sys.exit(0)


def main():
    Node().spin()


if __name__ == '__main__':
    main()
