import rospy
import pygame
from std_msgs.msg import Float64MultiArray
from operator_node.parser import ParseInterfaceLog
from pygame_teleop.screen import Screen

class Node:

    hz = 40
    dt = 1.0/float(hz)


    def __init__(self):

        # Initialize ros
        rospy.init_node('operator_interface_logger_visualizer_node')

        # Setup screen
        config = {
            'caption': 'Operator interface logger visualization',
            'width': 800,
            'height': 600,
            'background_color': 'darkslateblue',
            'windows': {
                'vis': {
                    'background_color': 'white',
                    'origin': (20, 20),
                    'width': 800-40,
                    'height': 600-40,
                    'type': 'TimeSeries',
                    'tf': 0.5,
                    'tp': 4.0,
                    'y_lo': -0.04,
                    'y_up': 0.04,
                    'n': 500,
                    'show_axis': True,
                    'axis_width': 3,
                }
            }
        }

        self.screen = Screen(config)
        self.clock = pygame.time.Clock()

        # Setup ROS subscribers and timers
        self.window = None
        rospy.Subscriber('operator_node/window', Float64MultiArray, self.callback)


    def callback(self, msg):
        self.window = ParseInterfaceLog(msg)


    def main_loop(self):
        running = True
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                rospy.loginfo('User quit pygame window.')
                pygame.quit()
                running = False
        self.screen.reset()
        if self.window is not None:
            self.screen.windows['vis'].plot_line(self.window.t-self.window.t_now(), self.window.h[0,:], 'blue', 3)
            self.screen.windows['vis'].plot_line(self.window.t-self.window.t_now(), self.window.h[1,:], 'red', 3)
            self.screen.windows['vis'].plot_point(0, self.window.h_now()[0], 'blue', 10)
            self.screen.windows['vis'].plot_point(0, self.window.h_now()[1], 'red', 10)
        self.screen.final()
        self.clock.tick(self.hz)
        return running


    def spin(self):
        while self.main_loop():
            pass


def main():
    Node().spin()


if __name__ == '__main__':
    main()
