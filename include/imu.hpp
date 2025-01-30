#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <array>
#include "uart.hpp"

std::array<double, 4> euler_to_quaternion(double roll, double pitch, double yaw);

class IMU : public rclcpp::Node
{
public:
    IMU();
      
private:
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub;

  
}