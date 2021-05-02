#!/usr/bin/env python3
import rospy
import numpy
from keyboard.msg import Key
from sensor_msgs.msg import Joy
from ros_helper.node import RosNode

class Node(RosNode):

    def __init__(self):
        RosNode.__init__(rospy)
        self.initNode('keyboard_to_joy_mapper_node')
        self.onShutdownUseBaseShutdownMethod()
        self.getParams([
            ("~sampling_rate", 100),
            ("~positive_axes", []),
            ("~negative_axes", []),
            ("~buttons", []),
        ])

        self.pos_axes = [
            getattr(Key, code)
            for code in self.params["~positive_axes"]
        ]
        n_axes = len(self.pos_axes)
        self.pos_axes_states = numpy.zeros(n_axes)
        self.neg_axes = [
            getattr(Key, code)
            for code in self.params["~negative_axes"]
        ]
        self.neg_axes_states = numpy.zeros(n_axes)
        self.buttons = [
            getattr(Key, code)
            for code in self.params["~buttons"]
        ]
        n_buttons = len(self.buttons)

        self.joy = Joy(axes=[0]*n_axes, buttons=[0]*n_buttons)
        self.subs['keydown'] = rospy.Subscriber(
            'keyboard/keydown', Key, self.keyDownCallback
        )
        self.subs['keyup'] = rospy.Subscriber(
            'keyboard/keyup', Key, self.keyUpCallback
        )
        self.setupPublisher('output', 'joy', Joy)
        self.startTimer(
            'main_loop', self.params['~sampling_rate'], self.publishJoy
        )

    def keyDownCallback(self, msg):
        if msg.code in self.pos_axes:
            idx = self.pos_axes.index(msg.code)
            self.pos_axes_states[idx] = 1
        elif msg.code in self.neg_axes:
            idx = self.neg_axes.index(msg.code)
            self.neg_axes_states[idx] = 1
        elif msg.code in self.buttons:
            idx = self.buttons.index(msg.code)
            self.joy.buttons[idx] = 1

    def keyUpCallback(self, msg):
        if msg.code in self.pos_axes:
            idx = self.pos_axes.index(msg.code)
            self.pos_axes_states[idx] = 0
        elif msg.code in self.neg_axes:
            idx = self.neg_axes.index(msg.code)
            self.neg_axes_states[idx] = 0
        elif msg.code in self.buttons:
            idx = self.buttons.index(msg.code)
            self.joy.buttons[idx] = 0

    def publishJoy(self, event):
        self.joy.axes = self.pos_axes_states - self.neg_axes_states
        self.pubs['output'].publish(self.addTimeStampToMsg(self.joy))


if __name__ == '__main__':
    Node().spin()
