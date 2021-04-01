import copy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray

class RosNode:

    """ROS node base class"""

    def __init__(self, rospy, name):
        """Base-initialization."""
        self.rospy = rospy
        self.rospy.init_node(name)
        self.name = rospy.get_name()

    def spin(self):
        """ROS-spin"""
        self.rospy.spin()

class OperatorNode(RosNode):

    """Operator node base class"""

    def __init__(self, rospy, name):
        """Operator node base class"""
        super().__init__(rospy, name)
        self.rospy.on_shutdown(self.shutdown)
        self.joy_to_h_map = [int(idx) for idx in rospy.get_param('~joy_to_h_map')]
        flip = [
            -1 if int(f)>0 else 1
            for f in rospy.get_param('~flip', [])
        ]
        self.flip = flip if len(flip) > 0 else [1]*len(self.joy_to_h_map)

    def setup_joy_reader(self):
        """Setup the joy reader."""
        input_topic = 'joy'
        self.read_joy(self.rospy.wait_for_message(input_topic, Joy))
        self.joy_sub = self.rospy.Subscriber(input_topic, Joy, self.read_joy)
        self.rospy.loginfo(f'{self.name}: Joy subscriber successfully setup.')

    def read_joy(self, msg):
        """Read joy message, map operator input, and publish control signal."""
        h = [self.flip[i]*msg.axes[idx] for i, idx in enumerate(self.joy_to_h_map)]
        u = self.f(h)
        self.publish(u)

    def setup_publishers(self):
        """Set's up the control publisher."""
        output_topic = 'operator_node/u'
        self.u_pub = self.rospy.Publisher(output_topic, Float64MultiArray, queue_size=10)

    def publish(self, u):
        """Publishes control command."""
        self.u_pub.publish(Float64MultiArray(data=u))

    def shutdown(self):
        """Shutdown node."""
        if hasattr(self, 'joy_sub'):
            self.joy_sub.unregister()
        if hasattr(self, 'update_timer'):
            self.update_timer.shutdown()
        self.rospy.loginfo(f'{self.name}: Shutdown complete.')


def main(node):
    node.setup_publishers()
    node.setup_joy_reader()
    node.spin()
