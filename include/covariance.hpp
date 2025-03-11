#pragma once

#include <array>
#include <cmath>

// Function to calculate odometry covariance dynamically based on linear and angular velocities
std::array<double, 36> odom_covariance(double linear_velocity, double angular_velocity)
{
    double base_cv = 0.05; // Base covariance from sensor noise estimation
    double cv = base_cv + 0.02 * std::abs(linear_velocity) + 0.02 * std::abs(angular_velocity);

    // Create a diagonal covariance matrix (6x6), with covariance value on the diagonal
    std::array<double, 36> covariance = {
        cv, 0, 0, 0, 0, 0,
        0, cv, 0, 0, 0, 0,
        0, 0, 0.2, 0, 0, 0,  // Higher uncertainty in Z (vertical)
        0, 0, 0, 0.02, 0, 0, // Roll
        0, 0, 0, 0, 0.02, 0, // Pitch
        0, 0, 0, 0, 0, 0.05  // Yaw
    };
    return covariance;
}

// Function to calculate IMU orientation covariance dynamically based on angular velocity
std::array<double, 9> imu_orientation_covariance(double angular_velocity)
{
    double base_cv = 0.01; // Base covariance
    double cv = base_cv + 0.01 * std::abs(angular_velocity);

    // Create a diagonal covariance matrix (3x3), with covariance value on the diagonal
    std::array<double, 9> covariance = {
        cv, 0, 0,
        0, cv, 0,
        0, 0, 0.05 // Higher uncertainty in yaw due to magnetometer noise
    };
    return covariance;
}

// Function to calculate IMU angular velocity covariance dynamically based on angular velocity
std::array<double, 9> imu_angular_velocity_covariance(double angular_velocity)
{
    double base_cv = 0.005; // Base covariance
    double cv = base_cv + 0.005 * std::abs(angular_velocity);

    // Create a diagonal covariance matrix (3x3), with covariance value on the diagonal
    std::array<double, 9> covariance = {
        cv, 0, 0,
        0, cv, 0,
        0, 0, 0.02 // Yaw tends to drift more, so slightly higher
    };
    return covariance;
}

// Function to calculate IMU linear acceleration covariance dynamically based on linear acceleration
std::array<double, 9> imu_linear_acceleration_covariance(double linear_acceleration)
{
    double base_cv = 0.01; // Base covariance
    double cv = base_cv + 0.01 * std::abs(linear_acceleration);

    // Create a diagonal covariance matrix (3x3), with covariance value on the diagonal
    std::array<double, 9> covariance = {
        cv, 0, 0,
        0, cv, 0,
        0, 0, 0.2 // Vertical acceleration is the least reliable
    };
    return covariance;
}
