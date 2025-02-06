#include "imu.hpp"
#include <cmath>
#include <chrono>
#include <thread>

using json = nlohmann::json;

std::array<double, 4> euler_to_quaternion(double roll, double pitch, double yaw)
{
    std::array<double, 4> q;
    q[0] = sin(roll / 2) * cos(pitch / 2) * cos(yaw / 2) - cos(roll / 2) * sin(pitch / 2) * sin(yaw / 2);
    q[1] = cos(roll / 2) * sin(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * cos(pitch / 2) * sin(yaw / 2);
    q[2] = cos(roll / 2) * cos(pitch / 2) * sin(yaw / 2) - sin(roll / 2) * sin(pitch / 2) * cos(yaw / 2);
    q[3] = cos(roll / 2) * cos(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * sin(pitch / 2) * sin(yaw / 2);
    return q;
}

IMU::IMU() : Node("imu"), uart() 
{
    pub = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);

    RCLCPP_INFO(this->get_logger(), "Starting /imu/data Publisher");
}

void IMU::imu_callback(const sensor_msgs::msg::Imu::SharedPtr /*msg*/)
{
    json imu_request;
    imu_request["T"] = 126;

    std::string imu_request_json = imu_request.dump() + "\n";

    uart.send_data(imu_request_json);

    // Sleep for 0.5 seconds
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    std::string imu_response = uart.read_data();

    json imu_data = json::parse(imu_response);

    double roll = imu_data["r"];
    double pitch = imu_data["p"];
    double yaw = imu_data["y"];
    double accel_x = imu_data["ax"];
    double accel_y = imu_data["ay"];
    double accel_z = imu_data["az"];
    double gyro_x = imu_data["gx"];
    double gyro_y = imu_data["gy"];
    double gyro_z = imu_data["gz"];

    std::array<double, 4> quaternion = euler_to_quaternion(roll, pitch, yaw);

    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header.stamp = this->now();
    imu_msg.header.frame_id = "imu_link";
    imu_msg.orientation.x = quaternion[0]; // qx
    imu_msg.orientation.y = quaternion[1]; // qy
    imu_msg.orientation.z = quaternion[2]; // qz
    imu_msg.orientation.w = quaternion[3]; // qw
    imu_msg.linear_acceleration.x = accel_x;
    imu_msg.linear_acceleration.y = accel_y;
    imu_msg.linear_acceleration.z = accel_z;
    imu_msg.angular_velocity.x = gyro_x;
    imu_msg.angular_velocity.y = gyro_y;
    imu_msg.angular_velocity.z = gyro_z;

    pub->publish(imu_msg);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IMU>());
    rclcpp::shutdown();
    return 0;
}
