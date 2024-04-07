#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <cube_msgs/msg/cube_state.hpp>

#include "cube_interfaces/controller.h"

using CubeState = cube_msgs::msg::CubeState;
using Float32 = std_msgs::msg::Float32;
using std::placeholders::_1;

class CubeInterfaces : public rclcpp::Node
{
public:
    CubeInterfaces(std::string node_name, rclcpp::NodeOptions options)
        : Node(node_name, options)
    {
        // timer
        timer = this->create_wall_timer(std::chrono::milliseconds(loop_ms),
                                        std::bind(&CubeInterfaces::TimerCallback, this));

        // subscriber
        rclcpp::QoS qos(rclcpp::KeepLast(10));
        state_sub = this->create_subscription<CubeState>(
            "/state", qos, [this](const CubeState::SharedPtr msg)
            { state = *msg; });

        // publisher
        module1_input_pub = this->create_publisher<Float32>("/module1_input", qos);
        module2_input_pub = this->create_publisher<Float32>("/module2_input", qos);
        module3_input_pub = this->create_publisher<Float32>("/module3_input", qos);
        module4_input_pub = this->create_publisher<Float32>("/module4_input", qos);
        module5_input_pub = this->create_publisher<Float32>("/module5_input", qos);
        module6_input_pub = this->create_publisher<Float32>("/module6_input", qos);
    }

private:
    void TimerCallback()
    {
        controller.update(state);
        Eigen::Vector<double, 6> u = controller.getInput();
        module1_input.data = u(0);
        module2_input.data = u(1);
        module3_input.data = u(2);
        module4_input.data = u(3);
        module5_input.data = u(4);
        module6_input.data = u(5);

        module1_input_pub->publish(module1_input);
        module2_input_pub->publish(module2_input);
        module3_input_pub->publish(module3_input);
        module4_input_pub->publish(module4_input);
        module5_input_pub->publish(module5_input);
        module6_input_pub->publish(module6_input);
    }

    int loop_ms{10};
    CubeState state;
    Float32 module1_input;
    Float32 module2_input;
    Float32 module3_input;
    Float32 module4_input;
    Float32 module5_input;
    Float32 module6_input;

    // intance of controller
    Controller controller;

    // timer
    rclcpp::TimerBase::SharedPtr timer;
    // subscriber
    rclcpp::Subscription<CubeState>::SharedPtr state_sub;
    // publisher
    rclcpp::Publisher<Float32>::SharedPtr module1_input_pub;
    rclcpp::Publisher<Float32>::SharedPtr module2_input_pub;
    rclcpp::Publisher<Float32>::SharedPtr module3_input_pub;
    rclcpp::Publisher<Float32>::SharedPtr module4_input_pub;
    rclcpp::Publisher<Float32>::SharedPtr module5_input_pub;
    rclcpp::Publisher<Float32>::SharedPtr module6_input_pub;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node_options = rclcpp::NodeOptions();
    auto node = std::make_shared<CubeInterfaces>("cube_interfaces_node", node_options);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}