#!/usr/bin/env python3
import numpy as np
from operator_node.node import OperatorNode, main


class IsometricNode(OperatorNode):

    def __init__(self):

        # Initialize
        super().__init__()

        # Get parameters
        self.axes_id = np.array(self.config['axes_idx'], dtype=int)
        scale = float(self.config.get('scale', 1.0))
        self.scale = np.clip(scale, 0.0, np.inf)

        # Post init
        self.post_init()

    def callback(self, msg):

        # Get axes
        _axes = np.array(msg.axes)
        axes = _axes[self.axes_id]

        # Ensure isometry
        nrm = np.linalg.norm(axes)
        if nrm > 0.0:
            signal = (self.scale*min(nrm, 1.0)/nrm) * axes
        else:
            signal = np.zeros(axes.shape)

        # Publish
        self.publish(signal)

if __name__ == '__main__':
    main(IsometricNode)
