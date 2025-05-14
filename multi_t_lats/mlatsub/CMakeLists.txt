cmake_minimum_required(VERSION 3.8)
project(mlatsub)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# add executable
add_executable(mlatsub_node src/latsub.cpp)
ament_target_dependencies(mlatsub_node rclcpp sensor_msgs std_msgs)

install(TARGETS
  mlatsub_node
  DESTINATION lib/${PROJECT_NAME}
)

# testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
