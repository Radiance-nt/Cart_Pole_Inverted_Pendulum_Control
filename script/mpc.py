#!/usr/bin/env python3
import rospy
import numpy as np
import time
from controller_base import InvertedPendulumController
from scipy.signal import cont2discrete
from pyMPC.mpc import MPCController


class MPCInvertedPendulumController(InvertedPendulumController):
    def __init__(self, A_cont, B_cont, Q, R, dt, N=10, **kwargs):
        super().__init__(**kwargs)

        # Discretize the system matrices
        C = np.eye(A_cont.shape[0])
        D = np.zeros((A_cont.shape[0], B_cont.shape[1]))
        sys_continuous = (A_cont, B_cont, C, D)
        sys_discrete = cont2discrete(sys_continuous, dt)

        A_d = sys_discrete[0]
        B_d = sys_discrete[1]

        self.controller = MPCController(A_d, B_d, Np=N, Qx=Q, Qu=R, xref=self.x_ref)
        self.controller.setup()

    def get_output(self):
        """
        Compute the control output using MPC.
        """
        u = self.controller.output()
        self.controller.update(self.current_state)
        return u.item()
