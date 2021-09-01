#!/usr/bin/env python3
import rospy
import pygame
import random
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from pygame_teleop.screen import Screen


"""

Creates a window that allows the user to draw paths.  The path between
mouse button down and up events is recorded and published as a
nav_msg/Path message. The mouse position is continuously streamed as a
geometry_msgs/PoseStamped. Additionally, while the mouse button is
pressed the mouse position is published.

"""


def main():

    # Initialize ROS
    rospy.init_node('mouse_input_node')
    draw = rospy.get_param('~draw', True)
    # TODO: give use ability to specify size of screen and robot env using ROS parameters
    pub_mouse_cts = rospy.Publisher('operator_node/mouse_position_continuous', PoseStamped, queue_size=10)
    pub_mouse_int = rospy.Publisher('operator_node/mouse_position_on_interaction', PoseStamped, queue_size=10)
    pub_mouse_path = rospy.Publisher('operator_node/mouse_position_path_after_interation', Path, queue_size=10)

    # Setup
    config = {
        'caption': 'Drawing example',
        'width': 500,
        'height': 500,
        'background_color': 'darkslateblue',
        'windows':{
            'robotenv': {
                'origin': (10, 10),
                'width': 480,
                'height': 480,
                'background_color': 'white',
                'type': 'RobotEnvironment',
                'robotenv_width': 1.0,
                'robotenv_height': 1.0,
                'robotenv_origin_location': 'lower_left',
                'show_origin': True,
                'robots': {
                    'robot1': {
                        'show_path': False,
                        'robot_radius': 0.025,
                        'robot_color': 'black'
                    }
                }

            }
        }
    }
    screen = Screen(config)
    clock = pygame.time.Clock()
    path = None
    hz = 100
    user_interating = False
    all_colors = list(pygame.colordict.THECOLORS.keys())

    def random_path_color():
        ridx = random.randint(0, len(all_colors)-1)
        return all_colors[ridx]

    path_color = random_path_color()

    running = True

    # Main loop
    try:

        while running:

            events = pygame.event.get()
            pos = screen.windows['robotenv'].get_mouse_position()
            msg = PoseStamped()
            msg.header.stamp = rospy.Time.now()
            msg.pose.position.x = pos[0]
            msg.pose.position.y = pos[1]
            pub_mouse_cts.publish(msg)

            for event in events:
                if event.type == pygame.QUIT:
                    running = False

                if event.type == pygame.MOUSEBUTTONDOWN:
                    user_interating = True

                if event.type == pygame.MOUSEBUTTONUP:
                    path.header.stamp = rospy.Time.now()
                    pub_mouse_path.publish(path)
                    rospy.loginfo('sent mouse path on topic operator_node/mouse_position_path_after_interation')
                    user_interating = False
                    path = None
                    path_color = random_path_color()

            if user_interating:
                pub_mouse_int.publish(msg)
                if path is None:
                    path = Path()
                path.poses.append(msg)

            screen.reset()
            screen.windows['robotenv'].robots['robot1'].draw(pos)
            if draw and user_interating:
                screen.windows['robotenv'].static_circle(
                    path_color,
                    screen.windows['robotenv'].convert_position(pos),
                    screen.windows['robotenv'].convert_scalar(0.01),
                )
            screen.final()
            clock.tick_busy_loop(hz)

    except KeyboardInterrupt:
        rospy.logwarn('user quit mouse_input_node.py')

    pygame.quit()


if __name__ == '__main__':
    main()
