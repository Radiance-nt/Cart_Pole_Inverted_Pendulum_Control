#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import numpy as np
import time


class InvertedPendulumController:
    def __init__(self, x_ref=None, rate=100):
        # Publisher: Command force to the pendulum
        self.command_pub = rospy.Publisher('/pendulum/x_controller/command',
                                           Float64, queue_size=1)

        # Subscriber: Get joint states
        rospy.Subscriber('/pendulum/joint_states', JointState, self.joint_state_callback, queue_size=1)

        # Initialize state variables
        self.current_state = np.array([0.0, 0.0, 0.0, 0.0])  # [x, x_dot, theta, theta_dot]
        if x_ref is None:
            self.x_ref = np.zeros(4)
        else:
            self.x_ref = x_ref

        self.rate = rospy.Rate(rate)
        self.freq = rate

        # Allow some time for connections to establish
        time.sleep(1.5)

    def joint_state_callback(self, data):
        """
        Callback function to update the state variables from joint states.
        Assumes:
        - data.position[0]: Cart position (x)
        - data.velocity[0]: Cart velocity (x_dot)
        - data.position[1]: Pendulum angle (theta)
        - data.velocity[1]: Pendulum angular velocity (theta_dot)
        """
        self.current_state[0] = data.position[0]  # x
        self.current_state[1] = data.velocity[0]  # x_dot
        self.current_state[2] = -data.position[1]  # theta (inverted)
        self.current_state[3] = -data.velocity[1]  # theta_dot (inverted)

        # Logging
        rospy.loginfo_throttle(1, f'x_pos: {self.current_state[0]:.4f} m')
        rospy.loginfo_throttle(1, f'x_vel: {self.current_state[1]:.4f} m/s')
        rospy.loginfo_throttle(1, f'theta_pos: {self.current_state[2]:.4f} rad')
        rospy.loginfo_throttle(1, f'theta_vel: {self.current_state[3]:.4f} rad/s')

    def balance(self):
        """
        Compute the control output and publish the command.
        """
        output = self.get_output()
        command_msg = Float64()
        command_msg.data = output
        self.command_pub.publish(command_msg)
        rospy.loginfo_throttle(1, f'commanding: {command_msg.data:.4f}')

    def get_output(self):
        """
        Abstract method to compute the control output.
        Must be implemented by derived classes.
        """
        raise NotImplementedError("get_output() must be implemented by subclass")

    def run(self):
        """
        Main loop to compute and publish control commands.
        """
        while not rospy.is_shutdown():
            self.balance()
            self.rate.sleep()

    @property
    def desired_state(self):
        return self.x_ref
