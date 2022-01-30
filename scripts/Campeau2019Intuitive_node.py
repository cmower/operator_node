#!/usr/bin/env python3
import rospy
import tf2_ros
import numpy as np
import tf_conversions
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TransformStamped

"""

My implementation of [1].

References
----------

 [1] Alexandre Campeau-Lecours, Ulysse Côté-Allard, Dinh-Son Vu,
     François Routhier, Benoit Gosselin, and Clément Gosselin,
     Intuitive Adaptive Orientation Control for Enhanced Human–Robot
     Interaction, IEEE Transactions On Robotics, Vol. 35, No. 2, April
     2019.


"""

ex0, ey0, ez0 = np.hsplit(np.eye(3), 3)
alpha_min = 0.001  # TODO: tune


class Node:

    hz = 50
    dt = 1.0/float(hz)

    def __init__(self):

        # Init ros node
        rospy.init_node('Campeau2019Intuitive_node')

        # Setup tf interface
        self.tfBuffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tfBuffer)
        self.tfBroadcaster = tf2_ros.TransformBroadcaster()

        # Get frames
        self.world_frame = rospy.get_param('~world_frame')
        self.eff_frame = rospy.get_param('~eff_frame')
        self.goal_frame = rospy.get_param('~goal_frame')

        # Subscribe to operator signal messages
        self.h = np.zeros(6)
        rospy.Subscriber('operator_node/signal', Float64MultiArray, self.callback)

        # Start main loop
        rospy.Timer(rospy.Duration(self.dt), self.main_loop)

    def callback(self, msg):
        self.h = np.array(msg.data[:6])  # TODO: check required dimensions

    def main_loop(self):

        # Get transform
        failed_to_retrieve_tf = False
        try:
            pos, rot = tfBuffer.lookup_transform(self.world_frame, self.eff_frame, rospy.Time())
            if (pos is None) or (rot is None):
                failed_to_retrieve_tf = True
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            failed_to_retrieve_tf = True
        if failed_to_retrieve_tf:
            rospy.logwarn(f'Failed to retrieve link between {self.world_frame} and {self.eff_frame}.')
            return

        # Get eff transform unit vectors
        T = tf_conversions.transformations.quaternion_matrix(rot)
        ex1, ey1, ez1 = np.hsplit(T[:3,:], 3)

        # Compute new transformation matrix. See [1, Sec II(C)].
        ez2 = ez1.copy()
        alpha = np.arccos(ez0.dot(ez2))
        if alpha >= alpha_min:
            ez0_x_ez2 = np.cross(ez0, ez2)
            ex2 = ez0_x_ez2/np.linalg.norm(ez0_x_ez2)
        else:
            ex2 = ex0.copy()
        ey2 = np.cross(ez2, ex2)

        # Update transformation with user input
        d = self.dt*self.h.copy()
        p = np.array(pos)
        R = np.eye(3)
        R[:,0] = ex2
        R[:,1] = ey2
        R[:,2] = ez2
        new_pos = p + R@d[:3]
        eul = tf_conversions.transformations.euler_from_matrix(R)
        new_eul = eul + d[3:]
        new_quat = tf_conversions.transformations.quaternion_from_euler(*new_eul.tolist())

        # Set new transformation
        tf = TransformStamped()
        tf.header.stamp = rospy.Time.now()
        tf.header.frame_id = self.world_frame
        tf.child_frame_id = self.goal_frame
        tf.transform.translation.x = new_pos[0]
        tf.transform.translation.y = new_pos[1]
        tf.transform.translation.z = new_pos[2]
        tf.transform.rotation.x = new_quat[0]
        tf.transform.rotation.y = new_quat[1]
        tf.transform.rotation.z = new_quat[2]
        tf.transform.rotation.w = new_quat[3]
        self.tfBroadcaster.sendTransform(tf)


    def spin(self):
        rospy.spin()


def main():
    Node().spin()


if __name__ == '__main__':
    main()
