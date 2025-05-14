cmake_minimum_required(VERSION 3.8)
project(latpub)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)

# build as a shared library (component)
add_library(latpub_component SHARED
  src/LatencyPublisherComponent.cpp
)
ament_target_dependencies(latpub_component
  rclcpp
  rclcpp_components
  sensor_msgs
)
rclcpp_components_register_node(
  latpub_component
  PLUGIN "latpub_ns::LatencyPublisherComponent"
  EXECUTABLE latpub_component_exec
)

# install library and plugin description
install(TARGETS latpub_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
)
install(FILES plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

