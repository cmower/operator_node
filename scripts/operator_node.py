#!/usr/bin/env python
import sys
import rospy
import math
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray


class OperatorNode:

    def __init__(self):
        rospy.init_node('operator_node')
        self.name = rospy.get_name()
        rospy.on_shutdown(self.shutdown)

        # Get ROS Parameters
        self.direction_x0 = float(rospy.get_param('~dr_x0', 1))
        self.direction_x1 = float(rospy.get_param('~dr_x1', 1))
        self.max_velocity = float(rospy.get_param('~max_velocity'))
        self.axis_x0 = rospy.get_param('~ax_x0', 0)
        self.axis_x1 = rospy.get_param('~ax_x1', 1)
        flip_axes = rospy.get_param('~flip_axes', False)
        if flip_axes:
            self.__handle_axes = self.__flip_axes
        else:
            self.__handle_axes = self.__dont_flip_axes
        rospy.loginfo(f'{self.name}: initialized complete')

    def setupPublishers(self):
        self.pub_signal = rospy.Publisher(
            'signal', Float64MultiArray, queue_size=10
        )
        self.pub_norm = rospy.Publisher(
            'normalized', Float64MultiArray, queue_size=10
        )
        rospy.loginfo(f'{self.name}: setup publishers')

    def setupJoyRemapper(self):
        self.sub = rospy.Subscriber('raw', Joy, self.remapJoyToOperatorSignal)
        rospy.loginfo(f'{self.name}: setup subscribers')

    def __flip_axes(self, h):
        h.reverse()

    def __dont_flip_axes(self, h):
        pass

    def __publish(self, pub, h):
        pub.publish(Float64MultiArray(data=h))

    def remapJoyToOperatorSignal(self, msg):
        """Listens for Joy messages and publishes operator signals."""

        # Get raw signal
        hr0 = self.direction_x0 * msg.axes[self.axis_x0]
        hr1 = self.direction_x1 * msg.axes[self.axis_x1]

        # Normalize
        htheta = math.atan2(hr1, hr0)
        hscale = min(math.sqrt(hr0**2 + hr1**2), 1.0)
        hnorm = [hscale * math.cos(htheta), hscale * math.sin(htheta)]
        self.__handle_axes(hnorm)

        # Compute signal
        hsignal = [self.max_velocity*hnorm[0], self.max_velocity*hnorm[1]]

        # Publish messages
        self.__publish(self.pub_norm, hnorm)
        self.__publish(self.pub_signal, hsignal)

    def spin(self):
        rospy.loginfo('Starting to spin...')
        try:
            rospy.spin()
        except KeyboardInterrupt:
            rospy.logwarn('{self.name}: User interrupt, quitting...')
        except rospy.ROSException as error:
            rospy.logerr(f'{self.name}: {error}')
        finally:
            sys.exit(0)

    def shutdown(self):
        if hasattr(self, 'sub'):
            self.sub.unregister()
        rospy.loginfo(f'{self.name}: Shutting down')


if __name__ == '__main__':
    node = OperatorNode()
    node.setupPublishers()
    node.setupJoyRemapper()
    node.spin()