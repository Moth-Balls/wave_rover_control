#include "uart.hpp"
#include <csignal>

// Implementation of SerialDevice
SerialDevice::SerialDevice() : serialStream(SERIAL_PORT)
{
    serialStream.SetBaudRate(BAUD_RATE);
    serialStream.SetCharacterSize(CHAR_SIZE);
    serialStream.SetStopBits(STOP_BITS);
    serialStream.SetParity(PARITY);
}

SerialDevice& SerialDevice::getInstance() {
    static SerialDevice instance;
    return instance;
}

void SerialDevice::send_data(const std::string &data)
{
    serialStream.write(data.c_str(), data.size());
    serialStream.flush();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
}

std::string SerialDevice::read_data()
{
    std::string response;
    while (true) {
        std::getline(serialStream, response);
        if (!response.empty() && response.back() == '\r') {
            response.pop_back();
        }
        RCLCPP_INFO(rclcpp::get_logger("SerialDevice"), "Raw response: %s", response.c_str());
        try {
            json imu_data = json::parse(response);
            if (imu_data.contains("T") && imu_data["T"] == 1001) {
                std::cout << response << std::endl;
                return response;
            }
        } catch (const std::exception &e) {
            RCLCPP_ERROR(rclcpp::get_logger("SerialDevice"), "JSON parse error: %s", e.what());
        }
    }
}

SerialGateway::SerialGateway() 
  : Node("serial_gateway"),
    uart_(SerialDevice::getInstance()),
    running_(true)
{
    tx_sub_ = this->create_subscription<std_msgs::msg::String>(
        "serial_tx", 10,
        std::bind(&SerialGateway::tx_callback, this, std::placeholders::_1)
    );

    rx_pub_ = this->create_publisher<std_msgs::msg::String>("serial_rx", 10);

    read_thread_ = std::thread(&SerialGateway::read_loop, this);

    // Register signal handler
    std::signal(SIGINT, SerialGateway::signal_handler);

    RCLCPP_INFO(this->get_logger(), "SerialGateway started");
}

SerialGateway::~SerialGateway()
{
    running_ = false;
    if (read_thread_.joinable()) {
        read_thread_.join();
    }
}

void SerialGateway::tx_callback(const std_msgs::msg::String::SharedPtr msg)
{
    uart_.send_data(msg->data);
}

void SerialGateway::read_loop()
{
    while (running_) {
        std::string data = uart_.read_data();
        if (!data.empty()) {
            RCLCPP_INFO(this->get_logger(), "Read data from serial: %s", data.c_str());
            std_msgs::msg::String rx_msg;
            rx_msg.data = data;
            rx_pub_->publish(rx_msg);
        }
    }
}

void SerialGateway::signal_handler(int signal)
{
    if (signal == SIGINT) {
        rclcpp::shutdown();
    }
}

void SerialGateway::stop()
{
    running_ = false;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SerialGateway>();
    rclcpp::spin(node);
    node->stop();  // Ensure the read loop stops
    rclcpp::shutdown();
    return 0;
}
