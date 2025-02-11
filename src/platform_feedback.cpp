#include "platform_feedback.hpp"
#include <signal.h>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PlatformFeedback>());
    rclcpp::shutdown();
    return 0;
}
