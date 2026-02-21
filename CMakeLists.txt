cmake_minimum_required(VERSION 3.8)
project(ros2_custom_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find necessary packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate custom message interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotStatus.msg"
)

# Add the publisher executable
add_executable(status_publisher src/status_publisher.cpp)

# Add standard dependencies
ament_target_dependencies(status_publisher rclcpp)

# Link the executable against the generated message target
rosidl_target_interfaces(status_publisher ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Install the executable so `ros2 run` can find it
install(TARGETS
  status_publisher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
