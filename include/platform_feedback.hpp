#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
//#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <array>
#include <nlohmann/json.hpp>
#include <nav_msgs/msg/odometry.hpp>

#define COMMAND_ID 130

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

nav_msgs::msg::Odometry convert_to_odometry(double left_wheel_velocity, double right_wheel_velocity, rclcpp::Time current_time)
{
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = current_time;
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";
    double wheel_base = 0.097; // 0.097 meters

    // Calculate linear and angular velocities
    double linear_velocity = (left_wheel_velocity + right_wheel_velocity) / 2.0;
    double angular_velocity = (right_wheel_velocity - left_wheel_velocity) / wheel_base;

    odom_msg.twist.twist.linear.x = linear_velocity;
    odom_msg.twist.twist.angular.z = angular_velocity;

    return odom_msg;
}

class PlatformFeedback : public rclcpp::Node
{
public:
    PlatformFeedback()
        : Node("platform_feedback")
    {
        imu_pub = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);
        odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
        serial_tx_pub = this->create_publisher<std_msgs::msg::String>("serial_tx", 10);
        serial_rx_sub = this->create_subscription<std_msgs::msg::String>("serial_rx", 10,
            std::bind(&PlatformFeedback::serial_rx_callback, this, std::placeholders::_1)
        );

        timer_ = this->create_wall_timer(
          std::chrono::seconds(1),
          std::bind(&PlatformFeedback::send_request, this)
        );

        RCLCPP_INFO(this->get_logger(), "Starting Feedback Publisher");
    }

private:
    void send_request()
    {
        nlohmann::json feedback_request;
        feedback_request["T"] = COMMAND_ID;
        std::string feedback_request_json = feedback_request.dump() + "\n";

        std_msgs::msg::String msg;
        msg.data = feedback_request_json;
        serial_tx_pub->publish(msg);
    }

    void serial_rx_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::string response = msg->data;
        RCLCPP_INFO(this->get_logger(), "Received response: %s", response.c_str());

        try {
            nlohmann::json feedback_data = nlohmann::json::parse(response);

            if (feedback_data.contains("T") && feedback_data["T"] == COMMAND_ID)
                return;

            sensor_msgs::msg::Imu imu_msg;
            imu_msg.header.stamp = this->now();
            imu_msg.header.frame_id = "imu_link";

            double roll  = feedback_data["r"];
            double pitch = feedback_data["p"];
            double yaw   = feedback_data["y"];

            std::array<double, 4> quaternion = euler_to_quaternion(roll, pitch, yaw);
            imu_msg.orientation.x = quaternion[0];
            imu_msg.orientation.y = quaternion[1];
            imu_msg.orientation.z = quaternion[2];
            imu_msg.orientation.w = quaternion[3];

            imu_msg.angular_velocity.x = feedback_data.value("gx", 0.0);
            imu_msg.angular_velocity.y = feedback_data.value("gy", 0.0);
            imu_msg.angular_velocity.z = feedback_data.value("gz", 0.0);
            imu_msg.linear_acceleration.x = feedback_data.value("ax", 0.0);
            imu_msg.linear_acceleration.y = feedback_data.value("ay", 0.0);
            imu_msg.linear_acceleration.z = feedback_data.value("az", 0.0);
            imu_pub->publish(imu_msg);

            double left_wheel_velocity = feedback_data["L"];
            double right_wheel_velocity = feedback_data["R"];

            nav_msgs::msg::Odometry odom_msg = convert_to_odometry(left_wheel_velocity, right_wheel_velocity, this->now());
            odom_pub->publish(odom_msg);

        } catch (const nlohmann::json::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to parse feedback data: %s", e.what());
        }
    }

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr serial_tx_pub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr serial_rx_sub;
    rclcpp::TimerBase::SharedPtr timer_;
};