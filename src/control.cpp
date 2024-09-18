#include <ros/ros.h>
#include <control_msgs/JointControllerState.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <Eigen/Dense>
#include <math.h>
#include <memory>

#include "controller_base.h"
#include "controller_lqr.h"
#include "controller_mpc.h"

const int control_freq = 100;

const double M = 2.0;
const double m = 0.1;
const double g = 9.8;
const double l = 0.5 / 2.0;
const double I = 1.0 / 3.0 * m * l * l;
const double b = 0.0;
const double P = (M + m) * I + M * m * l * l;

std::string CONTROLLER;

// A and B matrices
Eigen::Matrix4d A;
Eigen::Matrix<double, 4, 1> B;

Eigen::Matrix4d Q;
Eigen::Matrix<double, 1, 1> R;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Controller");
    ros::NodeHandle nh("~");

    std::string default_value = "LQR";
    nh.param("controller", CONTROLLER, default_value);
    ROS_INFO_STREAM("Controller: " << CONTROLLER);
    ros::Rate loop_rate(control_freq);

    A << 0, 1, 0, 0,
        0, -b * (I + m * l * l) / P, m * m * g * l * l / P, 0,
        0, 0, 0, 1,
        0, -b * m * l / P, m * g * l * (M + m) / P, 0;
    B << 0, (I + m * l * l) / P, 0, m * l / P;

    Q << 10.0, 0.0, 0.0, 0.0,
        0.0, 10.0, 0.0, 0.0,
        0.0, 0.0, 10.0, 0.0,
        0.0, 0.0, 0.0, 10.0;
    R << 0.1;

    std::unique_ptr<InvertedPendulumController> controller;
    if (CONTROLLER == "LQR")
    {
        controller = std::make_unique<LQRInvertedPendulumController>(nh, A, B, Q, R);
    }
    else if (CONTROLLER == "MPC")
    {
        controller = std::make_unique<MPCInvertedPendulumController>(nh, A, B, Q, R, 0.02);
    }
    else
    {
        ROS_ERROR_STREAM("Controller " << CONTROLLER << " is not available");
        exit(1);
    }

    while (ros::ok())
    {
        ros::spinOnce();
        controller->balance();
        loop_rate.sleep();
    }

    return 0;
}
