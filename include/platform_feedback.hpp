#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/string.hpp>
#include <array>
#include <nlohmann/json.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "covariance.hpp"
#include "low_pass.hpp"

using json = nlohmann::json;

#define COMMAND_ID1 130
#define COMMAND_ID2 126

rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr serial_tx_pub;
rclcpp::Subscription<std_msgs::msg::String>::SharedPtr serial_rx_sub;
rclcpp::TimerBase::SharedPtr timer_;
rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;
std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

int previous_command_id = COMMAND_ID1; // Initial command ID

double deg_to_rad(double deg)
{
    return deg * M_PI / 180.0;
}

void handle_imu(const json& feedback_data)
{
    IMUData current_imu_data;
    current_imu_data.roll = deg_to_rad(feedback_data["r"]);
    current_imu_data.pitch = deg_to_rad(feedback_data["p"]);
    current_imu_data.yaw = deg_to_rad(feedback_data["y"]);
    current_imu_data.gx = deg_to_rad(feedback_data["gx"]);
    current_imu_data.gy = deg_to_rad(feedback_data["gy"]);
    current_imu_data.gz = deg_to_rad(feedback_data["gz"]);
    current_imu_data.ax = feedback_data["ax"];
    current_imu_data.ay = feedback_data["ay"];
    current_imu_data.az = feedback_data["az"];

    // Apply low-pass filter
    IMUData filtered_imu_data = apply_low_pass_filter(current_imu_data);

    tf2::Quaternion q;
    q.setRPY(filtered_imu_data.roll, filtered_imu_data.pitch, filtered_imu_data.yaw);

    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header.stamp = rclcpp::Clock().now();
    imu_msg.header.frame_id = "imu_link";

    imu_msg.orientation = tf2::toMsg(q);
    
    imu_msg.angular_velocity.x = filtered_imu_data.gx;
    imu_msg.angular_velocity.y = filtered_imu_data.gy;
    imu_msg.angular_velocity.z = filtered_imu_data.gz;
    
    imu_msg.linear_acceleration.x = filtered_imu_data.ax;
    imu_msg.linear_acceleration.y = filtered_imu_data.ay;
    imu_msg.linear_acceleration.z = filtered_imu_data.az;
    
    RCLCPP_INFO(rclcpp::get_logger("platform_feedback"), "Publishing IMU data");
    imu_pub->publish(imu_msg);
}

nav_msgs::msg::Odometry convert_to_odometry(double left_wheel_velocity, double right_wheel_velocity, rclcpp::Time current_time)
{
    static double x = 0.0;
    static double y = 0.0;
    static double theta = 0.0;
    static rclcpp::Time last_time = current_time;

    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = current_time;
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";

    double wheel_base = 0.097; // 0.097 meters
    
    left_wheel_velocity = -left_wheel_velocity; // inverted because of hardware
    right_wheel_velocity = -right_wheel_velocity; // inverted because of hardware

    // Calculate linear and angular velocities.
    double linear_velocity = (left_wheel_velocity + right_wheel_velocity) / 2.0;
    double angular_velocity = (right_wheel_velocity - left_wheel_velocity) / wheel_base;

    // Calculate time difference
    double dt = (current_time - last_time).seconds();
    last_time = current_time;

    // Update pose
    x += linear_velocity * cos(theta) * dt;
    y += linear_velocity * sin(theta) * dt;
    theta -= angular_velocity * dt; // Invert the sign of theta for hardware

    // Set the position
    odom_msg.pose.pose.position.x = x;
    odom_msg.pose.pose.position.y = y;
    odom_msg.pose.pose.position.z = 0.0;

    // Set the orientation
    tf2::Quaternion q;
    q.setRPY(0, 0, theta); // Convert theta to radians
    odom_msg.pose.pose.orientation = tf2::toMsg(q);

    // Set the twist
    odom_msg.twist.twist.linear.x = linear_velocity;
    odom_msg.twist.twist.angular.z = -angular_velocity; // inverted because of hardware

    return odom_msg;
}

void send_request()
{
    json feedback_request;
    if (previous_command_id == COMMAND_ID1) {
        feedback_request["T"] = COMMAND_ID2;
        previous_command_id = COMMAND_ID2;
    } else {
        feedback_request["T"] = COMMAND_ID1;
        previous_command_id = COMMAND_ID1;
    }
    std::string feedback_request_json = feedback_request.dump() + "\n";

    std_msgs::msg::String msg;
    msg.data = feedback_request_json;
    serial_tx_pub->publish(msg);

    RCLCPP_INFO(rclcpp::get_logger("platform_feedback"), "Sending command %d", feedback_request["T"].get<int>());
}

void handle_odom(const json& feedback_data)
{
    double left_wheel_velocity = feedback_data["L"];
    double right_wheel_velocity = feedback_data["R"];

    nav_msgs::msg::Odometry odom_msg = convert_to_odometry(left_wheel_velocity, right_wheel_velocity, rclcpp::Clock().now());
    RCLCPP_INFO(rclcpp::get_logger("platform_feedback"), "Publishing odometry data");
    odom_pub->publish(odom_msg);

    // Broadcast the transform from odom to base_link
    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.header.stamp = odom_msg.header.stamp;
    transform_stamped.header.frame_id = "odom";
    transform_stamped.child_frame_id = "base_link";
    transform_stamped.transform.translation.x = odom_msg.pose.pose.position.x;
    transform_stamped.transform.translation.y = odom_msg.pose.pose.position.y;
    transform_stamped.transform.translation.z = 0.0;
    transform_stamped.transform.rotation = odom_msg.pose.pose.orientation;

    tf_broadcaster->sendTransform(transform_stamped);
}

void parse_feedback(const json& feedback_data)
{
    if (feedback_data["T"] == 1001)
    {
        handle_odom(feedback_data);
    }
    else if (feedback_data["T"] == 1002)
    {
        handle_imu(feedback_data);
    }
}

void serial_rx_callback(const std_msgs::msg::String::SharedPtr msg)
{
    std::string response = msg->data;
    
    json feedback = json::parse(response);

    parse_feedback(feedback);
}

void configure_imu_filters(const rclcpp::Node::SharedPtr& node)
{
    configure_filters(node);
}