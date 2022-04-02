#!/usr/bin/env python3
import numpy as np
from operator_node.node import OperatorNode, main

class ScaleNode(OperatorNode):

    def __init__(self):

        # Initialize
        super().__init__()

        # Get parameters
        self.axes_id = np.array(self.config['axes_idx'], dtype=int)
        scale = self.config.get('scale', 1.0)
        if isinstance(scale, (float, int)):
            self.scale = float(scale)*np.ones(self.axes_id.shape[0], dtype=float)
        elif isinstance(scale, (list, tuple)):
            self.scale = np.array(scale)

        # Post init
        self.post_init()

    def callback(self, msg):
        axes = np.array(msg.axes)
        self.publish(self.scale*axes[self.axes_id])

if __name__ == '__main__':
    main(ScaleNode)
