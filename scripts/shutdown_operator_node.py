#!/usr/bin/env python3
import rospy
from operator_node.srv import ShutdownOperatorNode

rospy.wait_for_service('shutdown_operator_node')
try:
    handle = rospy.ServiceProxy('shutdown_operator_node', ShutdownOperatorNode)
    handle(0)
except:
    print("Did not shutdown operator_node!")
