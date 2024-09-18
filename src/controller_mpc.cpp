#include "controller_mpc.h"

double MPCInvertedPendulumController::get_output()
{
    mpc_solver->updateFeedBack(current_state);

    if (mpc_solver->solveMPC(u))
    {
        return u.value();
    }
    return 0.0;
}