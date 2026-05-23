cmake_minimum_required(VERSION 3.8)
project(sensor_pkg C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# include 디렉토리 추가
include_directories(
  include
)

# 필요한 패키지
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(nlohmann_json REQUIRED)
find_package(std_srvs REQUIRED)

# 실행 노드 정의
add_executable(imu_driver src/imu_driver.cpp)
add_executable(odom_publisher src/odom_publisher.cpp)
add_executable(md_control src/MDControl.cpp)
add_executable(obstacle_detector_node src/obstacle_detector_node.cpp)
add_executable(battery_reader_node src/battery_reader_node.cpp)
add_executable(encoder_driver src/encoder_driver.cpp)


# 타겟 의존성 연결
ament_target_dependencies(imu_driver rclcpp sensor_msgs std_srvs tf2)
ament_target_dependencies(odom_publisher rclcpp geometry_msgs sensor_msgs nav_msgs tf2 tf2_ros)
ament_target_dependencies(md_control rclcpp geometry_msgs)
ament_target_dependencies(obstacle_detector_node rclcpp std_msgs)
ament_target_dependencies(battery_reader_node rclcpp std_msgs)
ament_target_dependencies(encoder_driver rclcpp std_msgs geometry_msgs)



# 설치 대상 지정
install(TARGETS
  imu_driver
  odom_publisher
  md_control
  obstacle_detector_node
  battery_reader_node
  encoder_driver
  DESTINATION lib/${PROJECT_NAME}
)

# launch 및 config 폴더 설치
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
