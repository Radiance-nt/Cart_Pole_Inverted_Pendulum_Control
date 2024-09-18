//
// created by linxif2008 on 04/23/2024
//

#ifndef MPC_SOLVER_TPP
#define MPC_SOLVER_TPP

#include "mpc_solver/mpc_solver.h"

template <int _states, int _commands, int _mpc_window>
MpcSolver<_states, _commands, _mpc_window>::MpcSolver(
    const Eigen::Matrix<double, _states, _states> &A,
    const Eigen::Matrix<double, _states, _commands> &B,
    const Eigen::Matrix<double, _states, _states> &Q,
    const Eigen::Matrix<double, _commands, _commands> &R,
    const double &predict_interval)
    : _A(A), _B(B), _predict_interval(predict_interval)
{
    // calculate A bar and B bar
    _A_bar = (Eigen::Matrix<double, _states, _states>::Identity() - 0.5 * _predict_interval * _A).inverse() * (Eigen::Matrix<double, _states, _states>::Identity() + 0.5 * _predict_interval * _A);

    _B_bar = _predict_interval * (Eigen::Matrix<double, _states, _states>::Identity() - 0.5 * _predict_interval * _A).inverse() * _B;

    // calculate phi
    _phi.block(0, 0, _states, _states) = _A_bar;
    for (int i = 1; i < _mpc_window; i++)
        _phi.block(i * _states, 0, _states, _states) = _A_bar * _phi.block((i - 1) * _states, 0, _states, _states);

    // calculate theta
    Eigen::Matrix<double, _states, _states> _A_bar_inv = _A_bar.inverse();
    Eigen::Matrix<double, _states, _commands> _theta_block = _B_bar;
    for (int i = 0; i < _mpc_window; i++)
    {
        for (int j = _mpc_window; j > i; j--)
        {
            int col = _mpc_window - j, row = i + col;
            _theta.block(row * _states, col * _commands, _states, _commands) = _theta_block;
        }
        _theta_block = _A_bar_inv * _theta_block;
    }

    // update Q matrix and R matrix
    this->updateQR(Q, R);
}

template <int _states, int _commands, int _mpc_window>
void MpcSolver<_states, _commands, _mpc_window>::updateQR(const Eigen::Matrix<double, _states, _states> &Q, const Eigen::Matrix<double, _commands, _commands> &R)
{
    // set Q
    for (int i = 0; i < _mpc_window; i++)
        _Q.block(i * _states, i * _states, _states, _states) = Q;

    // set R
    for (int i = 0; i < _mpc_window; i++)
        _R.block(i * _commands, i * _commands, _commands, _commands) = R;
}

template <int _states, int _commands, int _mpc_window>
void MpcSolver<_states, _commands, _mpc_window>::updateReference(const Eigen::Matrix<double, _states, 1> &ref)
{
    _x_ref = ref;

    // generate x_R
    for (int i = 0; i < _mpc_window; i++)
        _x_R.block(i * _states, 0, _states, 1) = _x_ref;
}

template <int _states, int _commands, int _mpc_window>
void MpcSolver<_states, _commands, _mpc_window>::updateFeedBack(const Eigen::Matrix<double, _states, 1> &fdb)
{
    _x_fdb = fdb;
}

template <int _states, int _commands, int _mpc_window>
bool MpcSolver<_states, _commands, _mpc_window>::solveMPC(Eigen::Matrix<double, _commands, 1> &u)
{
    // calculate E
    _E = _phi * _x_fdb - _x_R;

    // calcuate Hassain matrix and gradient vector for QP problem
    Eigen::Matrix<double, _commands * _mpc_window, _commands * _mpc_window> _H;
    Eigen::SparseMatrix<double> H;
    Eigen::Matrix<double, _commands * _mpc_window, 1> f;
    _H = 2 * (_theta.transpose() * _Q * _theta + _R);
    H.resize(_commands * _mpc_window, _commands * _mpc_window);
    for (int i = 0; i < _commands * _mpc_window; i++)
        for (int j = 0; j < _commands * _mpc_window; j++)
            H.insert(i, j) = _H(i, j);
    f = (2 * _E.transpose() * _Q * _theta).transpose();

    // generate QP problem solver
    OsqpEigen::Solver solver;
    solver.settings()->setWarmStart(true);
    solver.settings()->setVerbosity(false);

    solver.data()->setNumberOfVariables(_mpc_window);
    solver.data()->setNumberOfConstraints(0);

    if (!solver.data()->setHessianMatrix(H))
        return false;
    if (!solver.data()->setGradient(f))
        return false;

    if (!solver.initSolver())
        return false;
    if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError)
        return false;

    Eigen::Matrix<double, _commands * _mpc_window, 1> result = solver.getSolution();
    _u = result.block(0, 0, _commands, 1);
    u = _u;
    return true;
}

#endif // MPC_SOLVER_TPP