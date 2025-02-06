#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "uart.hpp"

std::string twist_to_json(const geometry_msgs::msg::Twist::SharedPtr msg);

class ControllerNode : public rclcpp::Node
{
public:
    ControllerNode();

private:
    void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub;
    SerialDevice& uart;
};