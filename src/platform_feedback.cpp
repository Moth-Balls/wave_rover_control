#include "platform_feedback.hpp"
#include <signal.h>


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("platform_feedback");

    odom_pub = node->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    imu_pub = node->create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);
    serial_tx_pub = node->create_publisher<std_msgs::msg::String>("serial_tx", 10);
    serial_rx_sub = node->create_subscription<std_msgs::msg::String>("serial_rx", 10, serial_rx_callback);
    tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(node);

    configure_imu_filters(node);

    timer_ = node->create_wall_timer(std::chrono::milliseconds(100), send_request);

    RCLCPP_INFO(node->get_logger(), "Starting Feedback Publisher");

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
