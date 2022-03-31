import rospy
import numpy as np
from sensor_msgs.msg import Joy
from abc import ABC, abstractmethod
from std_msgs.msg import Float64MultiArray
from custom_ros_tools.ros_comm import ToggleService

class OperatorNode(ABC):

    def __init__(self):

        ########################################
        ## Initialize ROS node
        rospy.init_node('operator_node')

        ########################################
        ## Get config
        self.config = rospy.get_param('~config')

        ########################################
        ## Setup publisher and start subscriber
        self.pub = rospy.Publisher('operator_node/signal', Float64MultiArray, queue_size=10)
        self.sub = None
        ToggleService('operator_node/toggle', self.start, self.stop)

    def post_init(self):
        if rospy.get_param('~start_on_init', False):
            self.start()
        rospy.loginfo('initialized operator node server')

    def start(self):
        if self.sub is None:
            self.sub = rospy.Subscriber('joy', Joy, self.callback)
            success, message = True, "started operator node"
        else:
            success, message = False, "tried to start operator node, but it is already running"
        return success, message

    def stop(self):
        if self.sub is not None:
            self.sub.unregister()
            self.sub = None
            success, message = True, "stopped operator node"
        else:
            success, message = False, "tried to stop operator node, but it is not running"
        return success, message

    @abstractmethod
    def callback(self, msg):
        pass

    def publish(self, signal):
        """Publishes an operator signal to ROS - signal must be a array-like object with float elements."""
        self.pub.publish(Float64MultiArray(data=signal))


    def spin(self):
        rospy.spin()


def main(Node):
    Node().spin()
