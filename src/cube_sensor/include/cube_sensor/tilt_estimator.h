#ifndef TILE_ESTIMATION_H
#define TILE_ESTIMATION_H

#include <Eigen/Dense>
#include <geometry_msgs/msg/vector3.hpp>
#include <cube_msgs/msg/float32_vector3.hpp>

using Vector3 = geometry_msgs::msg::Vector3;
using Float32Vector3 = cube_msgs::msg::Float32Vector3;

class TiltEstimator
{
public:
    TiltEstimator();
    // ~TiltEstimator();

    void update(Float32Vector3 &acc1, Float32Vector3 &acc2, Float32Vector3 &acc3,
                Float32Vector3 &acc4, Float32Vector3 &acc5, Float32Vector3 &acc6);
    Vector3 getAngle() { return euler_angle; };

private:
    Vector3 euler_angle;

    Eigen::Vector<double, 3> g; // estimated gravity vector
    Eigen::Vector<double, 6> X;
    Eigen::Matrix<double, 3, 6> M;
};

#endif // TILE_ESTIMATION_H