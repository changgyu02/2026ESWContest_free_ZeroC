cmake_minimum_required(VERSION 3.8)
project(alert_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# ===== speaker_node =====
add_executable(speaker_node src/speaker_node.cpp)
ament_target_dependencies(speaker_node rclcpp std_msgs)

# ===== led_node =====
add_executable(led_node src/led_node.cpp)
ament_target_dependencies(led_node rclcpp std_msgs)

install(TARGETS
  speaker_node
  led_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
