#pragma once

#include <libserial/SerialPort.h>
#include <libserial/SerialStream.h>
#include <nlohmann/json.hpp>
#include <string>
#include <memory>
#include <cstdint>
#include <iostream>
#include <thread>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <csignal>

#define SERIAL_PORT "/dev/ttyS0"
#define BAUD_RATE LibSerial::BaudRate::BAUD_115200
#define CHAR_SIZE LibSerial::CharacterSize::CHAR_SIZE_8
#define STOP_BITS LibSerial::StopBits::STOP_BITS_1
#define PARITY LibSerial::Parity::PARITY_NONE

using namespace LibSerial;
using namespace nlohmann;

class SerialDevice
{
public:
    static SerialDevice& getInstance();
    void send_data(const std::string &data);
    std::string read_data();

private:
    SerialDevice();
    SerialStream serialStream; // Using SerialStream

    SerialDevice(const SerialDevice&) = delete;
    SerialDevice& operator=(const SerialDevice&) = delete;
};

class SerialGateway : public rclcpp::Node
{
public:
    SerialGateway();
    ~SerialGateway();
    static void signal_handler(int signal);
    void stop();

private:
    void tx_callback(const std_msgs::msg::String::SharedPtr msg);
    void read_loop();

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr tx_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr rx_pub_;

    SerialDevice& uart_;
    std::thread read_thread_;
    std::atomic<bool> running_;
};
