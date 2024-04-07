#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
// #include <geometry_msgs/msg/transform_stamped.hpp>
#include <cube_msgs/msg/cube_state.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

using CubeState = cube_msgs::msg::CubeState;
using JointState = sensor_msgs::msg::JointState;
using Float32 = std_msgs::msg::Float32;
using std::placeholders::_1;

class CubeDescription : public rclcpp::Node
{
public:
    CubeDescription(std::string node_name, rclcpp::NodeOptions options)
        : Node(node_name, options),
          static_broadcaster(this)
    {
        // timer
        timer = this->create_wall_timer(std::chrono::milliseconds(loop_ms),
                                        std::bind(&CubeDescription::TimerCallback, this));

        // subscriber
        rclcpp::QoS qos(rclcpp::KeepLast(10));
        state_sub = this->create_subscription<CubeState>(
            "/state", qos, [this](const CubeState::SharedPtr msg)
            { state = *msg; });

        // publisher
        joint_state_pub =
            create_publisher<sensor_msgs::msg::JointState>("/joint_states", qos);
    }

private:
    void TimerCallback()
    {
        PublishJointState();
    }

    void PublishJointState()
    {
        // publish joint state
        auto joint_state = std::make_unique<JointState>();
        joint_state->name = {"module1_joint", "module2_joint", "module3_joint", "module4_joint", "module5_joint", "module6_joint"};
        joint_state->header.stamp = this->get_clock()->now();
        joint_state->position = {state.psi1, state.psi2, state.psi3, state.psi4, state.psi5, state.psi6};
        // joint_state->position = {M_PI / 2, 0, 0, 0, 0, 0};
        joint_state_pub->publish(std::move(joint_state));

        // publish static transform
        geometry_msgs::msg::TransformStamped static_transformStamped;
        static_transformStamped.header.stamp = this->get_clock()->now();
        static_transformStamped.header.frame_id = "base_link";
        static_transformStamped.child_frame_id = "body_link";
        static_transformStamped.transform.translation.x = 0;
        static_transformStamped.transform.translation.y = 0;
        static_transformStamped.transform.translation.z = 0;
        tf2::Quaternion quat;
        // quat.setRPY(state.roll, state.pitch, 0);
        quat.setRPY(0, M_PI / 4, 0);
        // quat.setRPY(-atan2(1, sqrt(2)), M_PI / 4, 0);
        static_transformStamped.transform.rotation.x = quat.x();
        static_transformStamped.transform.rotation.y = quat.y();
        static_transformStamped.transform.rotation.z = quat.z();
        static_transformStamped.transform.rotation.w = quat.w();
        static_broadcaster.sendTransform(static_transformStamped);
    }

    int loop_ms{10};
    CubeState state;

    // tf2_ros
    tf2_ros::StaticTransformBroadcaster static_broadcaster;
    // timer
    rclcpp::TimerBase::SharedPtr timer;
    // subscriber
    rclcpp::Subscription<CubeState>::SharedPtr state_sub;
    // publisher
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node_options = rclcpp::NodeOptions();
    auto node = std::make_shared<CubeDescription>("cube_description_node", node_options);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}