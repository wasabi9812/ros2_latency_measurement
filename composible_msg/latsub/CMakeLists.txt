cmake_minimum_required(VERSION 3.8)
project(latsub)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# 1) Shared library로 컴포넌트 빌드
add_library(latsub_component SHARED
  src/LatencySubscriberComponent.cpp
)
ament_target_dependencies(latsub_component
  rclcpp
  rclcpp_components
  sensor_msgs
)

# 2) 컴포넌트 등록
rclcpp_components_register_node(
  latsub_component
  PLUGIN "latsub_ns::LatencySubscriberComponent"
  EXECUTABLE latsub_component_exec
)

# 3) 설치 설정
install(TARGETS latsub_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
)
install(FILES plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

