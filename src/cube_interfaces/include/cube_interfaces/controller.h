#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <Eigen/Dense>
#include <cube_msgs/msg/cube_state.hpp>

using CubeState = cube_msgs::msg::CubeState;

class Controller
{
public:
    Controller();
    // ~Controller();

    void update(const CubeState &state);
    Eigen::Vector<double, 6> getInput() { return u; };

private:
    Eigen::Vector<double, 6> u;
    Eigen::Vector<double, 12> x;
    Eigen::Matrix<double, 6, 12> K;
};

#endif // CONTROLLER_H