#ifndef CONTROLLER_BASE_H
#define CONTROLLER_BASE_H

#include <ros/ros.h>
#include <control_msgs/JointControllerState.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <Eigen/Dense>
#include <math.h>
#include <memory>

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

#endif