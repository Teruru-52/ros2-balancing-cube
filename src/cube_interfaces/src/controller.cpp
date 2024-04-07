#include "cube_interfaces/controller.h"

Controller::Controller()
{
    K = Eigen::Matrix<double, 6, 12>::Zero();
}

void Controller::update(const CubeState &state)
{
    x(0) = state.psi1;
    x(1) = state.dot_psi1;
    x(2) = state.psi2;
    x(3) = state.dot_psi2;
    x(4) = state.psi3;
    x(5) = state.dot_psi3;
    x(6) = state.psi4;
    x(7) = state.dot_psi4;
    x(8) = state.psi5;
    x(9) = state.dot_psi5;
    x(10) = state.psi6;
    x(11) = state.dot_psi6;

    u = K * x;
}