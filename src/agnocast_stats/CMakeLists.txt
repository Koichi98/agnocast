cmake_minimum_required(VERSION 3.14)
project(agnocast_stats)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(agnocastlib REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(agnocast_stats_runner src/agnocast_stats_runner.cpp)
ament_target_dependencies(agnocast_stats_runner rclcpp agnocastlib)

add_executable(agnocast_stats_imu_test_publisher src/imu_test_publisher.cpp)
ament_target_dependencies(agnocast_stats_imu_test_publisher rclcpp agnocastlib sensor_msgs)

install(TARGETS
  agnocast_stats_runner
  agnocast_stats_imu_test_publisher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
