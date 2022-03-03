import rospy
from abc import ABC, abstractmethod
import numpy as np
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import SetBool, SetBoolResponse

class BasicOperatorNode(ABC):

    def __init__(self):

        # Init node
        rospy.init_node('operator_node')

        # Setup publisher
        self.pub = rospy.Publisher('operator_node/signal', Float64MultiArray, queue_size=10)

        # Get axes
        self.axes = [int(a) for a in rospy.get_param('~axes').split(' ')]
        naxes = len(self.axes)

        # Get scale
        scale_ = rospy.get_param('~scale', [1.0]*naxes)
        if isinstance(scale_, (float, int)):
            scale = [float(scale_)]*naxes
        elif isinstance(scale_, str):
            scale = [float(s) for s in scale_.split(' ')]
            if len(scale) == 1:
                scale = [scale]*naxes
        elif isinstance(scale_, (list, tuple)):
            scale = [float(s) for s in scale_]
        else:
            raise ValueError(f'Parameter ~scale type ({type(scale_)}) is not recognized!')

        self.scale = np.array(scale).flatten()

        # Setup ros service
        rospy.Service('operator_node/%s/toggle_callback' % self.__class__.__name__.lower(), SetBool, self.toggle)

        # Start on initialization
        success = True
        self.sub = None
        if rospy.get_param('~start_on_init', False):
            success, _ = self.start_callback()

        if success:
            rospy.loginfo('started operator node server')

    @abstractmethod
    def callback(self, msg):
        pass

    def toggle(self, req):
        if req.data:
            success, message = self.start_callback()
        else:
            success, message = self.stop_callback()
        return SetBoolResponse(success=success, message=message)


    def start_callback(self):
        if not self.sub:
            self.sub = rospy.Subscriber('joy', Joy, self.callback)
            success = True
            message = 'started callback'
        else:
            success = False
            message = 'user attempted to register subscriber but it is already registered!'
            rospy.logerr(message)
        return success, message


    def stop_callback(self):
        if self.sub:
            self.sub.unregister()
            self.sub = None
            success = True
            message = 'stopped callback'
        else:
            success = False
            message = 'user attempted to unregister subscriber but it is not registered!'
            rospy.logerr(message)
        return success, message


    def publish(self, signal):
        self.pub.publish(Float64MultiArray(data=np.asarray(signal).tolist()))


    def spin(self):
        rospy.spin()


def main(Node):
    Node().spin()
