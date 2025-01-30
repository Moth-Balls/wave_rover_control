#pragma once

#include "uart.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

std::string twist_to_json(const geometry_msgs::msg::Twist::SharedPtr msg);

class ControllerNode : public rclcpp::Node
{
public:
    ControllerNode(); 
    void send_data(const std::string &data);
private:
    UART uart;
    void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub;
};