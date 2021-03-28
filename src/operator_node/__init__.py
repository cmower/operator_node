import copy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray

class OperatorNode:

    """Operator node base class"""

    def __init__(self, rospy):
        self.rospy = rospy
        self.rospy.init_node('operator_node')
        self.name = self.rospy.get_name()
        self.rospy.on_shutdown(self.shutdown)
        self.hz = int(rospy.get_param('~sampling_rate', 100))
        self.joy_to_h_map = rospy.get_param('~joy_to_h_map')

    def set_rospy(self, rospy):
        """Passes rospy instance to base-class."""
        self.rospy = rospy

    def start_update(self):
        """Starts the main update loop"""
        dt = 1.0/float(self.hz)
        self.update_timer = self.rospy.Timer(self.rospy.Duration(dt), self.update)
        self.rospy.loginfo(f'[{self.name}] Started update timer.')

    def setup_joy_reader(self):
        """Setup the joy reader."""
        input_topic = 'joy'
        self.read_joy(self.rospy.wait_for_message(input_topic, Joy))
        self.joy_sub = self.rospy.Subscriber(input_topic, Joy, self.read_joy)
        self.rospy.loginfo(f'[{self.name}] Joy subscriber successfully setup.')

    def read_joy(self, msg):
        """Read joy message."""
        self.joy = msg

    def setup_publishers(self):
        """Set's up the control publisher."""
        output_topic = 'operator_node/u'
        self.u_pub = self.rospy.Publisher(output_topic, Float64MultiArray, queue_size=10)

    def get_h(self):
        """Return the current operator input."""
        joy = copy.deepcopy(self.joy)
        return joy.axes[self.joy_to_h_map]

    def publish(self, u):
        self.u_pub.publish(Float64MultiArray(data=u))

    def spin(self):
        """ROS-spin"""
        self.rospy.spin()

    def shutdown(self):
        """Shutdown node."""
        if hasattr(self, 'joy_sub'):
            self.joy_sub.unregister()
        if hasattr(self, 'update_timer'):
            self.update_timer.shutdown()
        self.rospy.loginfo(f'[{self.name}] Shutdown complete.')


def main(node):
    node.setup_publishers()
    node.setup_joy_reader()
    node.start_update()
    node.spin()