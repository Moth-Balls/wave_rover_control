cmake_minimum_required(VERSION 3.8)
project(wave_rover_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# Include the libserial headers
include_directories(/usr/include)

# Include the project headers
include_directories(include)

# Add the executables 
add_executable(controller src/controller.cpp)
add_executable(platform_feedback src/platform_feedback.cpp)
add_executable(serial_gateway src/uart.cpp)

# Link the necessary libraries
ament_target_dependencies(controller rclcpp std_msgs geometry_msgs nlohmann_json)
target_link_libraries(controller serial nlohmann_json::nlohmann_json)

ament_target_dependencies(platform_feedback rclcpp std_msgs sensor_msgs nav_msgs nlohmann_json)
target_link_libraries(platform_feedback serial nlohmann_json::nlohmann_json)

ament_target_dependencies(serial_gateway rclcpp std_msgs nlohmann_json)
target_link_libraries(serial_gateway serial nlohmann_json::nlohmann_json)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  controller
  platform_feedback
  serial_gateway
  DESTINATION lib/${PROJECT_NAME})

# Install the launch file
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
