#include "imu.hpp"
#include <cmath>
#include <chrono>
#include <thread>
#include <sstream>
#include <iostream>
#include <signal.h>

//using json = nlohmann::json;

std::array<double, 4> euler_to_quaternion(double roll, double pitch, double yaw)
{
    std::array<double, 4> q = {0, 0, 0, 0};
    q[0] = sin(roll / 2) * cos(pitch / 2) * cos(yaw / 2) - 
           cos(roll / 2) * sin(pitch / 2) * sin(yaw / 2);
    q[1] = cos(roll / 2) * sin(pitch / 2) * cos(yaw / 2) + 
           sin(roll / 2) * cos(pitch / 2) * sin(yaw / 2);
    q[2] = cos(roll / 2) * cos(pitch / 2) * sin(yaw / 2) - 
           sin(roll / 2) * sin(pitch / 2) * cos(yaw / 2);
    q[3] = cos(roll / 2) * cos(pitch / 2) * cos(yaw / 2) + 
           sin(roll / 2) * sin(pitch / 2) * sin(yaw / 2);
    return q;
}

IMU::IMU() : Node("imu"), uart(SerialDevice::getInstance())
{
    pub = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);
    RCLCPP_INFO(this->get_logger(), "Starting /imu/data Publisher");

    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&IMU::imu_callback, this)
    );
}

void IMU::imu_callback()
{
    json imu_request;
    imu_request["T"] = 126;
    std::string imu_request_json = imu_request.dump() + "\n";

    uart.send_data(imu_request_json);

    std::string response = uart.read_data();
    RCLCPP_INFO(this->get_logger(), "Received response: %s", response.c_str());

    try {
        json imu_data = json::parse(response);
        if (imu_data.contains("T") && imu_data["T"] == 126) {
            return;
        }

        sensor_msgs::msg::Imu imu_msg;
        imu_msg.header.stamp = this->now();
        imu_msg.header.frame_id = "imu_link";

        double roll  = imu_data["r"];
        double pitch = imu_data["p"];
        double yaw   = imu_data["y"];

        std::array<double, 4> quaternion = euler_to_quaternion(roll, pitch, yaw);
        imu_msg.orientation.x = quaternion[0];
        imu_msg.orientation.y = quaternion[1];
        imu_msg.orientation.z = quaternion[2];
        imu_msg.orientation.w = quaternion[3];

        imu_msg.angular_velocity.x = imu_data["gx"];
        imu_msg.angular_velocity.y = imu_data["gy"];
        imu_msg.angular_velocity.z = imu_data["gz"];

        imu_msg.linear_acceleration.x = imu_data["ax"];
        imu_msg.linear_acceleration.y = imu_data["ay"];
        imu_msg.linear_acceleration.z = imu_data["az"];

        pub->publish(imu_msg);
    } catch (const json::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to parse IMU data: %s", e.what());
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IMU>());
    rclcpp::shutdown();
    return 0;
}
