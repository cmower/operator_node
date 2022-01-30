#!/usr/bin/env python3
import sys
import rospy
import pygame
from std_msgs.msg import Int64MultiArray, UInt8


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
        width = rospy.get_param('~width', 500)
        height = rospy.get_param('~height', 500)

        # Setup publishers
        self.mouse_position_pub = rospy.Publisher('mouse/position', Int64MultiArray, queue_size=10)
        self.mouse_button_down_event_pub = rospy.Publisher('mouse/buttondown', UInt8, queue_size=10)
        self.mouse_button_up_event_pub = rospy.Publisher('mouse/buttonup', UInt8, queue_size=10)

        # Setup pygame window
        pygame.init()
        self.screen = pygame.display.set_mode((width, height))
        pygame.display.set_caption('Mouse Input')
        self.clock = pygame.time.Clock()
        self.screen.fill(pygame.Color('white'))
        self.running = True

        rospy.loginfo('initialized mouse input node')


    def spin(self):

        while self.running:

            for event in pygame.event.get():

                if event.type == pygame.QUIT:
                    self.running = False

                elif event.type == pygame.MOUSEBUTTONUP:
                    self.mouse_button_up_event_pub.publish(UInt8(data=event.button))

                elif event.type == pygame.MOUSEBUTTONDOWN:
                    self.mouse_button_down_event_pub.publish(UInt8(data=event.button))

            self.mouse_position_pub.publish(Int64MultiArray(data=pygame.mouse.get_pos()))
            pygame.display.flip()
            self.clock.tick_busy_loop(self.HZ)

        rospy.loginfo('shutting down mouse input node')
        pygame.quit()
        sys.exit(0)


def main():
    Node().spin()


if __name__ == '__main__':
    main()
