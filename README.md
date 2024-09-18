# Inverted Pendulum Control with LQR, MPC in Gazebo (Python, C++)

<p align="center">
  <img src="https://github.com/Radiance-nt/Cart_Pole_Inverted_Pendulum_Control/blob/os/invpend.gif?raw=true">
</p>


#### This repository was created to establish an easy-to-use framework for beginners to learn and test control algorithms with both Python and C++

## Getting started!

### Requirements

#### Python requirements:

[python-control](https://github.com/python-control/python-control), [pyMPC](https://github.com/forgi86/pyMPC)

#### C++ requirements:

Eigen, OsqpEigen

### Launch
Launch LQR controller:
```bash
source devel/setup.bash
roslaunch pendulum_control control_python.launch controller:=LQR # or
roslaunch pendulum_control control_cpp.launch controller:=LQR 
```

Launch MPC controller:
```bash
source devel/setup.bash
roslaunch pendulum_control control_python.launch controller:=MPC # or
roslaunch pendulum_control control_cpp.launch controller:=MPC
```

In Gazebo, right-click on the pole to apply a force and observe the stabilization process.

## Constants in this experiment:

```python
M = 2.0  # Cart
m = 0.1  # Pole
g = 9.8
l = 0.5  # Length of the pole
l /= 2  # The distance from the axis of rotation to the center of mass of the pole.
I = 1 / 3 * m * l * l  # Inertia
b = 0.0  # Friction
```

## An Easy Framework for Learning Usages

Designing a new algorithm is very simple; you just need to inherit the `get_output` function and use the member variables.

### Python framework

Python base class `InvertedPendulumController`, only need to overwrite `get_output` function:

```python

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

```

### C++ framework

C++ base class `InvertedPendulumController`, only need to overwrite `get_output` function:

```cpp

class InvertedPendulumController
{
public:
    InvertedPendulumController(ros::NodeHandle &nh)
    {
        command_pub = nh.advertise<std_msgs::Float64>("/pendulum/x_controller/command", 10);

        joint_state_sub = nh.subscribe("/pendulum/joint_states", 10, &InvertedPendulumController::jointStateCallback, this);

        current_state << 0.0, 0.0, 0.0, 0.0;
        desired_state << 0.0, 0.0, 0.0, 0.0;
    }

    virtual ~InvertedPendulumController() {}
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr &data)
    {
        current_state(0) = data->position[0];
        current_state(1) = data->velocity[0];
        current_state(2) = -data->position[1];
        current_state(3) = -data->velocity[1];

        ROS_INFO_THROTTLE(1, "x_pos: %f m", current_state(0));
        ROS_INFO_THROTTLE(1, "x_vel: %f m/s", current_state(1));
        ROS_INFO_THROTTLE(1, "theta_pos: %f rad", current_state(2));
        ROS_INFO_THROTTLE(1, "theta_vel: %f rad/s", current_state(3));
    }

    void balance()
    {
        double output = get_output();
        command_msg.data = output;
        command_pub.publish(command_msg);
        last_balance_time = ros::Time::now();
        ROS_INFO_THROTTLE(1, "commanding: %f", command_msg.data);
    }

    virtual double get_output() = 0;

protected:
    Eigen::Vector4d current_state;
    Eigen::Vector4d desired_state;
    ros::Time last_balance_time;

private:
    ros::Publisher command_pub;
    ros::Subscriber joint_state_sub;
    std_msgs::Float64 command_msg;
};
```