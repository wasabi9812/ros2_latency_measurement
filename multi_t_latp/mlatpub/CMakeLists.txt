cmake_minimum_required(VERSION 3.8)
project(mlatpub)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Executable node
add_executable(mlatpub_node src/latpub.cpp)
ament_target_dependencies(mlatpub_node rclcpp sensor_msgs std_msgs)

# Install
install(TARGETS
  mlatpub_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
