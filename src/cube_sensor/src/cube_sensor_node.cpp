#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <cube_msgs/msg/cube_state.hpp>
#include <cube_msgs/msg/module_state.hpp>
#include <cube_msgs/msg/imu_data.hpp>
#include <cube_msgs/msg/motor_state.hpp>

#include "cube_sensor/tilt_estimator.h"
#include "cube_sensor/state_estimator.h"

using CubeState = cube_msgs::msg::CubeState;
using ModuleState = cube_msgs::msg::ModuleState;
using ImuData = cube_msgs::msg::ImuData;
using MotorState = cube_msgs::msg::MotorState;
using Float32 = std_msgs::msg::Float32;
using Vector3 = geometry_msgs::msg::Vector3;
using std::placeholders::_1;

class CubeSensor : public rclcpp::Node
{
public:
    CubeSensor(std::string node_name, rclcpp::NodeOptions options)
        : Node(node_name, options)
    {
        // timer
        timer = this->create_wall_timer(std::chrono::milliseconds(loop_ms),
                                        std::bind(&CubeSensor::TimerCallback, this));

        // subscriber
        rclcpp::QoS qos(rclcpp::KeepLast(10));
        module1_sub = this->create_subscription<ModuleState>(
            "/module1", qos, [this](const ModuleState::SharedPtr msg)
            { imu1 = msg->imu;
            motor1 = msg->motor; });
        module2_sub = this->create_subscription<ModuleState>(
            "/module2", qos, [this](const ModuleState::SharedPtr msg)
            { imu2 = msg->imu;
            motor2 = msg->motor; });
        module3_sub = this->create_subscription<ModuleState>(
            "/module3", qos, [this](const ModuleState::SharedPtr msg)
            { imu3 = msg->imu;
            motor3 = msg->motor; });
        module4_sub = this->create_subscription<ModuleState>(
            "/module4", qos, [this](const ModuleState::SharedPtr msg)
            { imu4 = msg->imu;
            motor4 = msg->motor; });
        module5_sub = this->create_subscription<ModuleState>(
            "/module5", qos, [this](const ModuleState::SharedPtr msg)
            { imu5 = msg->imu;
            motor5 = msg->motor; });
        module6_sub = this->create_subscription<ModuleState>(
            "/module6", qos, [this](const ModuleState::SharedPtr msg)
            { imu6 = msg->imu;
            motor6 = msg->motor; });

        // publisher
        state_pub = this->create_publisher<CubeState>("/state", qos);
    }

private:
    void TimerCallback()
    {
        tilt_estimator.update(imu1.acc, imu2.acc, imu3.acc, imu4.acc, imu5.acc, imu6.acc);
        Vector3 euler_angle = tilt_estimator.getAngle();

        // update state estimation
        state_estimator->setMotor(motor1, motor2, motor3, motor4, motor5, motor6);
        state_estimator->setIMU(imu1, imu2, imu3, imu4, imu5, imu6);
        state_estimator->setEulerAngle(euler_angle);
        state_estimator->update();
        state = state_estimator->getState();
        state_pub->publish(state);
    }

    int loop_ms{10};
    ImuData imu1;
    ImuData imu2;
    ImuData imu3;
    ImuData imu4;
    ImuData imu5;
    ImuData imu6;
    MotorState motor1;
    MotorState motor2;
    MotorState motor3;
    MotorState motor4;
    MotorState motor5;
    MotorState motor6;
    CubeState state;

    // instances for state estimation
    TiltEstimator tilt_estimator;
    KalmanFilter kalman_filter;
    ComlimentaryFilter complimentary_filter;
    StateEstimatorBase *state_estimator{&kalman_filter};
    // StateEstimatorBase *state_estimator{&complimentary_filter};

    // timer
    rclcpp::TimerBase::SharedPtr timer;
    // subscriber
    rclcpp::Subscription<ModuleState>::SharedPtr module1_sub;
    rclcpp::Subscription<ModuleState>::SharedPtr module2_sub;
    rclcpp::Subscription<ModuleState>::SharedPtr module3_sub;
    rclcpp::Subscription<ModuleState>::SharedPtr module4_sub;
    rclcpp::Subscription<ModuleState>::SharedPtr module5_sub;
    rclcpp::Subscription<ModuleState>::SharedPtr module6_sub;
    // publisher
    rclcpp::Publisher<CubeState>::SharedPtr state_pub;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node_options = rclcpp::NodeOptions();
    auto node = std::make_shared<CubeSensor>("cube_sensor_node", node_options);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}