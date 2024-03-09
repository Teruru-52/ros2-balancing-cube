#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <cube_msgs/msg/imu_data.hpp>

using ImuData = cube_msgs::msg::ImuData;
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
        imu_sub = this->create_subscription<ImuData>(
            "/imu", qos, [this](const ImuData::SharedPtr msg)
            { imu_data = *msg; });
        // publisher
        imu_pub = this->create_publisher<ImuData>("/imu_data", qos);
    }

private:
    void TimerCallback()
    {
        imu_pub->publish(imu_data);
    }

    int loop_ms{1000};
    ImuData imu_data;

    // timer
    rclcpp::TimerBase::SharedPtr timer;
    // subscriber
    rclcpp::Subscription<ImuData>::SharedPtr imu_sub;
    // publisher
    rclcpp::Publisher<ImuData>::SharedPtr imu_pub;
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