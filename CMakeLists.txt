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

# Include the libserial headers
include_directories(/usr/include)

# Include the project headers
include_directories(include)

# Add the executable
add_executable(controller_node src/controller.cpp src/uart.cpp)
add_executable(imu_pub src/imu.cpp src/uart.cpp)

# Link the necessary libraries
ament_target_dependencies(controller_node rclcpp std_msgs geometry_msgs nlohmann_json)
target_link_libraries(controller_node serial nlohmann_json::nlohmann_json)

ament_target_dependencies(imu_pub rclcpp std_msgs sensor_msgs nlohmann_json)
target_link_libraries(imu_pub serial nlohmann_json::nlohmann_json)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  controller_node
  imu_pub
  DESTINATION lib/${PROJECT_NAME})

ament_package()
