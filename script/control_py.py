#!/usr/bin/env python3
import numpy as np
import rospy

from lqr import LQRInvertedPendulumController
from mpc import MPCInvertedPendulumController


def main():
    # Initialize the ROS node
    rospy.init_node('InvertedPendulumController', anonymous=True)
    controller_type = rospy.get_param("~controller", "LQR")

    # Cartpole
    M = 2.0  # kg
    m = 0.1  # kg
    g = 9.8  # m/s²
    l = 0.5  # meters
    l /= 2
    I = 1 / 3 * m * l * l
    b = 0.0  # N·m·s

    P = (M + m) * I + M * m * l * l

    # System matrices
    A = np.array([[0, 1, 0, 0],
                  [0, -b * (I + m * l * l) / P, m * m * g * l * l / P, 0],
                  [0, 0, 0, 1],
                  [0, -b * m * l / P, m * g * l * (M + m) / P, 0]])

    B = np.array([[0],
                  [(I + m * l * l) / P],
                  [0],
                  [m * l / P]])

    Q = np.array([[10.0, 0, 0, 0],
                  [0, 10.0, 0, 0],
                  [0, 0, 10.0, 0],
                  [0, 0, 0, 10.0]])

    R = np.array([[0.1]])

    x_ref = np.array([0.0, 0.0, 0.0, 0.0])

    if controller_type.lower() == "lqr":
        controller = LQRInvertedPendulumController(A, B, Q, R, x_ref=x_ref)
    elif controller_type.lower() == "mpc":
        dt = 0.02
        N = 200
        controller = MPCInvertedPendulumController(A, B, Q, R, dt, N, x_ref=x_ref)
    else:
        raise NameError
    controller.run()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
