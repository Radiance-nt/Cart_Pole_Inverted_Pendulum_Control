#ifndef CONTROLLER_MPC_H
#define CONTROLLER_MPC_H

#include "controller_base.h"
#include "mpc_solver/mpc_solver.h"

const int _mpc_window = 20;

class MPCInvertedPendulumController : public InvertedPendulumController
{
public:
    MPCInvertedPendulumController(ros::NodeHandle &nh,
                                  const Eigen::Matrix4d &A,
                                  const Eigen::Matrix<double, 4, 1> &B,
                                  const Eigen::Matrix4d &Q,
                                  const Eigen::Matrix<double, 1, 1> &R,
                                  double dt)
        : InvertedPendulumController(nh), A_(A), B_(B), Q_(Q), R_(R), dt_(dt)
    {
        ROS_INFO_STREAM("A:\n"
                        << A_);
        ROS_INFO_STREAM("B:\n"
                        << B_);
        ROS_INFO_STREAM("Q:\n"
                        << Q_);
        ROS_INFO_STREAM("R:\n"
                        << R_);
        ROS_INFO_STREAM("dt:\n"
                        << dt_);

        mpc_solver = std::make_shared<MpcSolver<4, 1, _mpc_window>>(A_, B_, Q_, R_, dt);
        mpc_solver->updateReference(desired_state);
    }

    double get_output() override;

private:
    Eigen::Matrix4d A_;
    Eigen::Matrix<double, 4, 1> B_;

    Eigen::Matrix4d Q_;
    Eigen::Matrix<double, 1, 1> R_;
    Eigen::Matrix<double, 1, 1> u;
    double dt_;
    std::shared_ptr<MpcSolver<4, 1, _mpc_window>> mpc_solver;
};

#endif