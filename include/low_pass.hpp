#pragma once

#include <filters/filter_chain.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <rclcpp/rclcpp.hpp>

filters::FilterChain<double> roll_filter("double");
filters::FilterChain<double> pitch_filter("double");
filters::FilterChain<double> yaw_filter("double");
filters::FilterChain<double> gx_filter("double");
filters::FilterChain<double> gy_filter("double");
filters::FilterChain<double> gz_filter("double");
filters::FilterChain<double> ax_filter("double");
filters::FilterChain<double> ay_filter("double");
filters::FilterChain<double> az_filter("double");

void configure_filters(const rclcpp::Node::SharedPtr& node)
{
    auto logger = node->get_node_logging_interface();
    auto parameters = node->get_node_parameters_interface();

    roll_filter.configure("low_pass_filter_roll", logger, parameters);
    pitch_filter.configure("low_pass_filter_pitch", logger, parameters);
    yaw_filter.configure("low_pass_filter_yaw", logger, parameters);
    gx_filter.configure("low_pass_filter_gx", logger, parameters);
    gy_filter.configure("low_pass_filter_gy", logger, parameters);
    gz_filter.configure("low_pass_filter_gz", logger, parameters);
    ax_filter.configure("low_pass_filter_ax", logger, parameters);
    ay_filter.configure("low_pass_filter_ay", logger, parameters);
    az_filter.configure("low_pass_filter_az", logger, parameters);
}

struct IMUData
{
    double roll;
    double pitch;
    double yaw;
    double gx;
    double gy;
    double gz;
    double ax;
    double ay;
    double az;
};

IMUData apply_low_pass_filter(const IMUData& current_data)
{
    IMUData filtered_data;
    roll_filter.update(current_data.roll, filtered_data.roll);
    pitch_filter.update(current_data.pitch, filtered_data.pitch);
    yaw_filter.update(current_data.yaw, filtered_data.yaw);
    gx_filter.update(current_data.gx, filtered_data.gx);
    gy_filter.update(current_data.gy, filtered_data.gy);
    gz_filter.update(current_data.gz, filtered_data.gz);
    ax_filter.update(current_data.ax, filtered_data.ax);
    ay_filter.update(current_data.ay, filtered_data.ay);
    az_filter.update(current_data.az, filtered_data.az);

    return filtered_data;
}
