#include "controller.hpp"

using json = nlohmann::json;

std::string twist_to_json(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    json j;
    j["T"] = 13;
    j["X"] = msg->linear.x;
    j["Z"] = msg->angular.z;
    
    return j.dump() + "\n";
}

ControllerNode::ControllerNode() : Node("controller_node"), uart()
{
    sub = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, std::bind(&ControllerNode::twist_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Controller Has Started");
}

void ControllerNode::twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    std::string data = twist_to_json(msg);
    uart.send_data(data);
    // std::cout << "Data sent: " << data << std::endl;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControllerNode>());
    rclcpp::shutdown();
    return 0;
}




