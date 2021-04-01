#!/usr/bin/env python3
import rospy
from keyboard.msg import Key
from sensor_msgs.msg import Joy
from operator_node import RosNode

class Node(RosNode):

    def __init__(self):

        # Initialize node
        super().__init__(rospy, 'keyboard_to_joy_mapper_node')

        # Get sampling frequency from ros
        self.hz = self.rospy.get_param('~frequency', 100)

        # Get axes maps from ros
        self.positive_axes_map = [getattr(Key, axis_key) for axis_key in self.rospy.get_param('~positive_axes', [])]
        self.negative_axes_map = [getattr(Key, axis_key)  for axis_key in self.rospy.get_param('~negative_axes', [])]
        assert len(self.positive_axes_map) == len(self.negative_axes_map), "Positive and negative axes maps must have equal length!"

        # Get button map from ros
        self.button_map = [getattr(Key, button_key) for button_key in self.rospy.get_param('~buttons', [])]

        # Setup positive/negative axes and button states
        self.positive_axes_states = [False]*len(self.positive_axes_map)
        self.negative_axes_states = [False]*len(self.negative_axes_map)
        self.button_states = [False]*len(self.button_map)

        # Parse positive/negative axes and button  maps as sets
        self.positive_axes_map_set = set(self.positive_axes_map)
        self.negative_axes_map_set = set(self.negative_axes_map)
        self.button_map_set = set(self.button_map)

        # Report
        self.rospy.loginfo(f'{self.name}: Node initialized.')

    #
    # Helpful methods
    #

    # Getter

    def key_from_msg(self, msg):
        return msg.code

    def index_from_positive_axes_key(self, key):
        return self.positive_axes_map.index(key)

    def index_from_negative_axes_key(self, key):
        return self.negative_axes_map.index(key)

    def index_from_button_key(self, key):
        return self.button_map.index(key)

    # Checks

    def is_positive_axes_key(self, key):
        return key in self.positive_axes_map_set

    def is_negative_axes_key(self, key):
        return key in self.negative_axes_map_set

    def is_button_key(self, key):
        return key in self.button_map_set

    #
    # Setup subscribers
    #

    def setup_keyboard_reader(self):
        self.sub_keydown = self.rospy.Subscriber('keyboard/keydown', Key, self.read_key_down)
        self.sub_keyup = self.rospy.Subscriber('keyboard/keyup', Key, self.read_key_up)

    def read_key(self, msg, state):
        key = self.key_from_msg(msg)
        if self.is_positive_axes_key(key):
            self.positive_axes_states[self.index_from_positive_axes_key(key)] = state
        if self.is_negative_axes_key(key):
            self.negative_axes_states[self.index_from_negative_axes_key(key)] = state
        if self.is_button_key(key):
            self.button_states[self.index_from_button_key(key)] = state

    def read_key_down(self, msg):
        self.read_key(msg, True)

    def read_key_up(self, msg):
        self.read_key(msg, False)

    #
    # Setup publisher and main timer
    #

    def start_joy_publisher(self):
        self.pub = self.rospy.Publisher('joy', Joy, queue_size=10)
        self.timer = self.rospy.Timer(self.rospy.Duration(1.0/float(self.hz)), self.publish_joy_msg)

    def publish_joy_msg(self, event):
        joy = Joy()
        joy.header.stamp = rospy.Time.now()
        joy.axes = [float(p) - float(n) for p, n in zip(self.positive_axes_states, self.negative_axes_states)]
        joy.buttons = self.button_states
        self.pub.publish(joy)

if __name__ == '__main__':
    node = Node()
    node.setup_keyboard_reader()
    node.start_joy_publisher()
    node.spin()
