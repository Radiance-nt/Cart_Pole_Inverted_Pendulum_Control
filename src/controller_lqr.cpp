#include "controller_lqr.h"

double LQRInvertedPendulumController::get_output()
{
    Eigen::Matrix<double, 4, 1> state_error = desired_state - current_state;
    return K.dot(state_error);
}

Eigen::Matrix<double, 4, 1> LQRInvertedPendulumController::computeLQR()
{
    // Invert R matrix
    Eigen::Matrix<double, 1, 1> R_inv = R_.inverse();

    // Compute BR^-1B^T term
    Eigen::Matrix<double, 4, 4> BRB = B_ * R_inv(0, 0) * B_.transpose();

    // Construct the Hamiltonian matrix H
    Eigen::Matrix<double, 8, 8> H;
    H.topLeftCorner(4, 4) = A_;
    H.topRightCorner(4, 4) = -BRB;
    H.bottomLeftCorner(4, 4) = -Q_;
    H.bottomRightCorner(4, 4) = -A_.transpose();

    // Compute eigenvalues and eigenvectors of the Hamiltonian matrix
    Eigen::ComplexEigenSolver<Eigen::Matrix<double, 8, 8>> ces;
    ces.compute(H);

    Eigen::VectorXcd eigenvalues = ces.eigenvalues();
    Eigen::MatrixXcd eigenvectors = ces.eigenvectors();

    // Select eigenvectors corresponding to eigenvalues with negative real parts
    Eigen::MatrixXcd Vs(8, 4);
    int index = 0;
    for (int i = 0; i < 8; ++i)
    {
        if (eigenvalues(i).real() < 0 && index < 4)
        {
            Vs.col(index) = eigenvectors.col(i);
            ++index;
        }
    }

    // Check if we found enough stable eigenvalues
    if (index != 4)
    {
        ROS_ERROR("computeLQR: Unable to find 4 stable eigenvalues.");
        return K; // Return current K if computation fails
    }

    // Partition Vs into Vs1 and Vs2
    Eigen::MatrixXcd Vs1 = Vs.topRows(4);
    Eigen::MatrixXcd Vs2 = Vs.bottomRows(4);

    // Compute the solution P of the Riccati equation
    Eigen::MatrixXcd P_c = Vs2 * Vs1.inverse();
    Eigen::Matrix4d P = P_c.real(); // Take the real part

    // Compute the LQR gain matrix K
    Eigen::Matrix<double, 1, 4> K_temp = R_inv(0, 0) * B_.transpose() * P;

    // Return K as a column vector to match Eigen::Matrix<double, 4, 1>
    return K_temp.transpose();
}
