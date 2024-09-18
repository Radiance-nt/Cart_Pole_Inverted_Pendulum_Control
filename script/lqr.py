#!/usr/bin/env python3
import rospy
import control
import numpy as np
import time
from controller_base import InvertedPendulumController


class LQRInvertedPendulumController(InvertedPendulumController):
    def __init__(self, A, B, Q, R, **kwargs):
        super().__init__(**kwargs)
        # Compute LQR gain
        K, S, E = control.lqr(A, B, Q, R)
        self.K = np.asarray(K).flatten()  # Ensure K is a 1D array
        print("LQR Gain K:", self.K)

    def get_output(self):
        """
        Compute the control force using the LQR controller.
        """
        error = self.desired_state - self.current_state
        control_force = np.dot(self.K, error)
        return control_force
