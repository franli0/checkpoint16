cmake_minimum_required(VERSION 3.8)
project(wheel_velocities_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Add executable
add_executable(wheel_velocities_publisher src/wheel_velocities_publisher.cpp)

# Specify dependencies
ament_target_dependencies(wheel_velocities_publisher
  rclcpp
  std_msgs
)

# Install targets
install(TARGETS
  wheel_velocities_publisher
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()