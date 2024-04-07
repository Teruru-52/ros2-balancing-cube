#include "cube_sensor/tilt_estimator.h"

TiltEstimator::TiltEstimator()
{
    X << 0, 0, 0, 0, 0, 0;
}

void TiltEstimator::update(Float32Vector3 &acc1, Float32Vector3 &acc2, Float32Vector3 &acc3,
                           Float32Vector3 &acc4, Float32Vector3 &acc5, Float32Vector3 &acc6)
{
    // imu measurement matrix
    M << acc1.x, acc2.x, acc3.x, acc4.x, acc5.x, acc6.x,
        acc1.y, acc2.y, acc3.y, acc4.y, acc5.y, acc6.y,
        acc1.z, acc2.z, acc3.z, acc4.z, acc5.z, acc6.z;

    // estimate gravity vector
    g = M * X;

    // calculate roll and pitch angles
    euler_angle.x = atan2(g(1), g(2));
    euler_angle.y = atan2(-g(0), sqrt(g(1) * g(1) + g(2) * g(2)));
}