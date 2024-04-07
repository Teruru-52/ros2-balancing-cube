#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <Eigen/Dense>
#include <cube_msgs/msg/cube_state.hpp>
#include <cube_msgs/msg/module_state.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <cube_msgs/msg/float32_vector3.hpp>

using Vector3 = geometry_msgs::msg::Vector3;
using Float32Vector3 = cube_msgs::msg::Float32Vector3;
using CubeState = cube_msgs::msg::CubeState;
using ModuleState = cube_msgs::msg::ModuleState;
using ImuData = cube_msgs::msg::ImuData;
using MotorState = cube_msgs::msg::MotorState;

class StateEstimatorBase
{
public:
    explicit StateEstimatorBase(){};
    void setMotor(MotorState &motor1, MotorState &motor2, MotorState &motor3,
                  MotorState &motor4, MotorState &motor5, MotorState &motor6);
    void setIMU(ImuData &imu1, ImuData &imu2, ImuData &imu3,
                ImuData &imu4, ImuData &imu5, ImuData &imu6);
    void setEulerAngle(Vector3 &euler) { euler_angle = euler; };
    virtual void update() = 0;
    CubeState getState() { return state; };
    virtual ~StateEstimatorBase(){};

protected:
    CubeState state;
    ModuleState module1, module2, module3, module4, module5, module6;
    Vector3 euler_angle;
    Eigen::Vector<double, 6> u;
};

class KalmanFilter : public StateEstimatorBase
{
public:
    KalmanFilter();

    // void init(Eigen::Vector<double, 8> &x0) { x = x0; }
    void init() { x = Eigen::Vector<double, 8>::Zero(); }
    void update() override;
    Eigen::Vector<double, 8> getState() { return x; };
    Eigen::Matrix<double, 8, 8> getCovariance() { return P; };

private:
    Eigen::Vector<double, 8> x;      // state vector
    Eigen::Matrix<double, 8, 8> P;   // covariance matrix
    Eigen::Matrix<double, 8, 8> A;   // system matrix
    Eigen::Matrix<double, 8, 6> B;   // input matrix
    Eigen::Matrix<double, 12, 8> C;  // measurement matrix
    Eigen::Matrix<double, 8, 8> Q;   // process noise covariance
    Eigen::Matrix<double, 12, 12> R; // measurement noise covariance
    Eigen::Matrix<double, 8, 8> I;   // identity matrix
};

class ComlimentaryFilter : public StateEstimatorBase
{
public:
    ComlimentaryFilter(){};

    void update() override;

private:
    double sensor_num = 6.0;
    const float dt = 0.01;
    const float coeff_kappa = 0.98;
    Eigen::Vector<double, 3> omega;
    Eigen::Vector<double, 3> dot_euler_angle;
    Vector3 est_euler_angle;
    Eigen::Matrix<double, 3, 3> T;

    void meanOmega();
};

#endif // KALMAN_FILTER_H