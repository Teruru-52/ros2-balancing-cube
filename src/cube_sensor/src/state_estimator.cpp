#include "cube_sensor/state_estimator.h"

void StateEstimatorBase::setMotor(MotorState &motor1, MotorState &motor2, MotorState &motor3,
                                  MotorState &motor4, MotorState &motor5, MotorState &motor6)
{
    module1.motor = motor1;
    module2.motor = motor2;
    module3.motor = motor3;
    module4.motor = motor4;
    module5.motor = motor5;
    module6.motor = motor6;
}

void StateEstimatorBase::setIMU(ImuData &imu1, ImuData &imu2, ImuData &imu3,
                                ImuData &imu4, ImuData &imu5, ImuData &imu6)
{
    module1.imu = imu1;
    module2.imu = imu2;
    module3.imu = imu3;
    module4.imu = imu4;
    module5.imu = imu5;
    module6.imu = imu6;
}

KalmanFilter::KalmanFilter()
{
    P.diagonal() << 1.09, 1.09, 1.09, 1.09, 1.09, 1.09, 0.72, 1.17;
    P(6, 7) = 0.13;
    P(7, 6) = 0.13;

    A = Eigen::Matrix<double, 8, 8>::Zero();
    B = Eigen::Matrix<double, 8, 6>::Zero();
    C = Eigen::Matrix<double, 12, 8>::Zero();

    Q.diagonal() << 1, 1, 1, 1, 1, 1, 0.01, 1;
    R.diagonal() << 0.1, 1, 0.1, 1, 0.1, 1, 0.1, 1, 0.1, 1, 0.1, 1;
    I.Identity(8, 8);
}

void KalmanFilter::update()
{
    Eigen::Vector<double, 12> y;
    Eigen::Matrix<double, 8, 12> K; // Kalman gain

    x = A * x + B * u;
    P = A * P * A.transpose() + Q;
    K = P * C.transpose() * (C * P * C.transpose() + R).inverse();
    x = x + K * (y - C * x);
    P = (I - K * C) * P;

    state.psi1 = x(0);
    state.psi2 = x(2);
    state.psi3 = x(4);
    state.psi4 = x(6);
    state.psi5 = x(8);
    state.psi6 = x(10);

    state.dot_psi1 = module1.motor.velocity;
    state.dot_psi2 = module2.motor.velocity;
    state.dot_psi3 = module3.motor.velocity;
    state.dot_psi4 = module4.motor.velocity;
    state.dot_psi5 = module5.motor.velocity;
    state.dot_psi6 = module6.motor.velocity;

    state.pitch = euler_angle.y;
    state.roll = euler_angle.x;
    state.dot_pitch = x(11);
    // state.dot_roll = x(12);
}

void ComlimentaryFilter::update()
{
    meanOmega();

    T << 0, sin(euler_angle.x) / cos(euler_angle.y), cos(euler_angle.x) / cos(euler_angle.y),
        0, cos(euler_angle.x), -sin(euler_angle.x),
        1, sin(euler_angle.x) * tan(euler_angle.y), cos(euler_angle.x) * tan(euler_angle.y);
    dot_euler_angle = T * omega;

    // update euler angle
    est_euler_angle.y = coeff_kappa * euler_angle.y + (1.0 - coeff_kappa) * (est_euler_angle.y + dot_euler_angle(1) * dt);
    est_euler_angle.x = coeff_kappa * euler_angle.x + (1.0 - coeff_kappa) * (est_euler_angle.x + dot_euler_angle(2) * dt);

    state.psi1 = module1.motor.angle;
    state.psi2 = module2.motor.angle;
    state.psi3 = module3.motor.angle;
    state.psi4 = module4.motor.angle;
    state.psi5 = module5.motor.angle;
    state.psi6 = module6.motor.angle;

    state.dot_psi1 = module1.motor.velocity;
    state.dot_psi2 = module2.motor.velocity;
    state.dot_psi3 = module3.motor.velocity;
    state.dot_psi4 = module4.motor.velocity;
    state.dot_psi5 = module5.motor.velocity;
    state.dot_psi6 = module6.motor.velocity;

    state.pitch = est_euler_angle.y;
    state.roll = est_euler_angle.x;
    state.dot_pitch = dot_euler_angle(1);
    state.dot_roll = dot_euler_angle(2);
}

void ComlimentaryFilter::meanOmega()
{
    omega(0) = (module1.imu.omega.x + module2.imu.omega.x + module3.imu.omega.x + module4.imu.omega.x + module5.imu.omega.x + module6.imu.omega.x) / sensor_num;
    omega(1) = (module1.imu.omega.y + module2.imu.omega.y + module3.imu.omega.y + module4.imu.omega.y + module5.imu.omega.y + module6.imu.omega.y) / sensor_num;
    omega(2) = (module1.imu.omega.z + module2.imu.omega.z + module3.imu.omega.z + module4.imu.omega.z + module5.imu.omega.z + module6.imu.omega.z) / sensor_num;
}