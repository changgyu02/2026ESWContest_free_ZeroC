#include <chrono>
#include <memory>
#include <cstdint>
#include <string>
#include <vector>
#include <cstring>
#include <fstream>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <stdexcept>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nlohmann/json.hpp>

using namespace std::chrono_literals;
using json = nlohmann::json;

// ==========================================
// 1. BNO055 하드웨어 I2C 드라이버 클래스
// ==========================================
class BNO055 {
public:
  explicit BNO055(const std::string& i2c_device = "/dev/i2c-7", uint8_t address = 0x28)
    : i2c_device_(i2c_device), address_(address) {
    i2c_fd_ = open(i2c_device_.c_str(), O_RDWR);
    if (i2c_fd_ < 0) {
      throw std::runtime_error("I2C 디바이스 열기 실패: " + i2c_device_);
    }
    if (ioctl(i2c_fd_, I2C_SLAVE, address_) < 0) {
      close(i2c_fd_);
      throw std::runtime_error("I2C 슬레이브 주소 설정 실패");
    }
  }

  ~BNO055() {
    if (i2c_fd_ >= 0) {
      close(i2c_fd_);
    }
  }

  bool initialize() {
    setMode(OPERATION_MODE_CONFIG);
    usleep(20000);

    if (!writeByte(BNO055_PWR_MODE_ADDR, 0x00)) return false;  // Normal mode
    usleep(10000);

    if (!writeByte(BNO055_SYS_TRIGGER_ADDR, 0x00)) return false;
    usleep(10000);

    setMode(OPERATION_MODE_NDOF);
    usleep(20000);

    return true;
  }

  bool readQuaternion(float& w, float& x, float& y, float& z) {
    uint8_t buffer[8];
    if (!readBytes(QUATERNION_DATA_W_LSB, buffer, 8)) return false;

    int16_t qw = (buffer[1] << 8) | buffer[0];
    int16_t qx = (buffer[3] << 8) | buffer[2];
    int16_t qy = (buffer[5] << 8) | buffer[4];
    int16_t qz = (buffer[7] << 8) | buffer[6];

    w = qw * QUAT_SCALE;
    x = qx * QUAT_SCALE;
    y = qy * QUAT_SCALE;
    z = qz * QUAT_SCALE;

    return true;
  }

  bool readGyroAndAccel(float& gx, float& gy, float& gz, float& ax, float& ay, float& az) {
    uint8_t gyro_buf[6];
    uint8_t accel_buf[6];

    if (!readBytes(GYRO_DATA_X_LSB, gyro_buf, 6)) return false;
    int16_t raw_gx = (gyro_buf[1] << 8) | gyro_buf[0];
    int16_t raw_gy = (gyro_buf[3] << 8) | gyro_buf[2];
    int16_t raw_gz = (gyro_buf[5] << 8) | gyro_buf[4];

    constexpr float dps_to_rad = M_PI / 180.0f;
    gx = (raw_gx * GYRO_SCALE) * dps_to_rad;
    gy = (raw_gy * GYRO_SCALE) * dps_to_rad;
    gz = (raw_gz * GYRO_SCALE) * dps_to_rad;

    if (!readBytes(ACCEL_DATA_X_LSB, accel_buf, 6)) return false;
    int16_t raw_ax = (accel_buf[1] << 8) | accel_buf[0];
    int16_t raw_ay = (accel_buf[3] << 8) | accel_buf[2];
    int16_t raw_az = (accel_buf[5] << 8) | accel_buf[4];

    ax = raw_ax * ACCEL_SCALE;
    ay = raw_ay * ACCEL_SCALE;
    az = raw_az * ACCEL_SCALE;

    return true;
  }

  bool loadCalibrationData(const std::string& json_file_path) {
    std::ifstream file(json_file_path);
    if (!file.is_open()) return false;

    json j;
    try { file >> j; } catch (...) { return false; }
    file.close();

    std::vector<uint8_t> calib;
    for (const auto& val : j["calibration_data"]) {
      calib.push_back(static_cast<uint8_t>(val));
    }
    if (calib.size() != CALIBRATION_DATA_LENGTH) return false;

    setMode(OPERATION_MODE_CONFIG);
    usleep(25000);

    if (!writeBytes(CALIBRATION_DATA_START_ADDR, calib.data(), calib.size())) return false;

    setMode(OPERATION_MODE_NDOF);
    usleep(20000);
    return true;
  }

private:
  int i2c_fd_{-1};
  std::string i2c_device_;
  uint8_t address_;

  bool writeByte(uint8_t reg, uint8_t value) {
    uint8_t buffer[2] = {reg, value};
    return write(i2c_fd_, buffer, 2) == 2;
  }

  bool readBytes(uint8_t reg, uint8_t* buffer, size_t length) {
    if (write(i2c_fd_, &reg, 1) != 1) return false;
    return read(i2c_fd_, buffer, length) == static_cast<ssize_t>(length);
  }

  bool writeBytes(uint8_t reg, const uint8_t* data, size_t length) {
    std::vector<uint8_t> buffer(length + 1);
    buffer[0] = reg;
    std::memcpy(&buffer[1], data, length);
    return write(i2c_fd_, buffer.data(), buffer.size()) == static_cast<ssize_t>(buffer.size());
  }

  void setMode(uint8_t mode) {
    writeByte(BNO055_OPR_MODE_ADDR, mode);
    usleep(30000);
  }

  static constexpr uint8_t BNO055_OPR_MODE_ADDR = 0x3D;
  static constexpr uint8_t BNO055_PWR_MODE_ADDR = 0x3E;
  static constexpr uint8_t BNO055_SYS_TRIGGER_ADDR = 0x3F;
  static constexpr uint8_t QUATERNION_DATA_W_LSB = 0x20;
  static constexpr uint8_t GYRO_DATA_X_LSB = 0x14;
  static constexpr uint8_t ACCEL_DATA_X_LSB = 0x08;

  static constexpr uint8_t OPERATION_MODE_CONFIG = 0x00;
  static constexpr uint8_t OPERATION_MODE_NDOF = 0x0C;
  static constexpr uint8_t CALIBRATION_DATA_START_ADDR = 0x55;
  static constexpr size_t CALIBRATION_DATA_LENGTH = 22;

  static constexpr float QUAT_SCALE = (1.0f / (1 << 14));
  static constexpr float GYRO_SCALE = (1.0f / 16.0f);
  static constexpr float ACCEL_SCALE = (1.0f / 100.0f);
};

// ==========================================
// 2. ROS 2 EKF 전용 IMU 발행 노드 클래스
// ==========================================
class ImuDriverNode : public rclcpp::Node {
public:
  ImuDriverNode()
  : Node("imu_driver"), imu_("/dev/i2c-7", 0x28) 
  {
    pub_imu_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu", 10);
    timer_ = this->create_wall_timer(50ms, std::bind(&ImuDriverNode::publish_imu_data, this)); // 20Hz 주행

    if (!imu_.initialize()) {
      RCLCPP_FATAL(this->get_logger(), "BNO055 하드웨어 초기화 실패. 노드를 종료합니다.");
      rclcpp::shutdown();
      return;
    }

    if (imu_.loadCalibrationData("/home/changgyu/ros2_ws/src/bno055_calibration.json")) {
      RCLCPP_INFO(this->get_logger(), "JSON 보정 데이터 로드 성공.");
    } else {
      RCLCPP_WARN(this->get_logger(), "보정값 로드 실패. 실시간 자동 보정(NDOF) 모드로 시작합니다.");
    }
  }

private:
  void publish_imu_data() {
    float q_w, q_x, q_y, q_z;
    float g_x, g_y, g_z;
    float a_x, a_y, a_z;

    if (!imu_.readQuaternion(q_w, q_x, q_y, q_z) || !imu_.readGyroAndAccel(g_x, g_y, g_z, a_x, a_y, a_z)) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "IMU 데이터 읽기 실패");
      return;
    }

    auto imu_msg = sensor_msgs::msg::Imu();
    imu_msg.header.stamp = this->now();
    imu_msg.header.frame_id = "imu_link"; // EKF의 URDF/TF 트리 명칭과 매칭

    // 방향 (Orientation)
    imu_msg.orientation.w = q_w;
    imu_msg.orientation.x = q_x;
    imu_msg.orientation.y = q_y;
    imu_msg.orientation.z = q_z;

    // 각속도 (Angular Velocity)
    imu_msg.angular_velocity.x = g_x;
    imu_msg.angular_velocity.y = g_y;
    imu_msg.angular_velocity.z = g_z;

    // 선가속도 (Linear Acceleration)
    imu_msg.linear_acceleration.x = a_x;
    imu_msg.linear_acceleration.y = a_y;
    imu_msg.linear_acceleration.z = a_z;

    // EKF 융합을 안정화하기 위한 고정 공분산(Covariance) 설정
    imu_msg.orientation_covariance[0] = 0.001;
    imu_msg.orientation_covariance[4] = 0.001;
    imu_msg.orientation_covariance[8] = 0.001;

    imu_msg.angular_velocity_covariance[0] = 0.001;
    imu_msg.angular_velocity_covariance[4] = 0.001;
    imu_msg.angular_velocity_covariance[8] = 0.001;

    imu_msg.linear_acceleration_covariance[0] = 0.04;
    imu_msg.linear_acceleration_covariance[4] = 0.04;
    imu_msg.linear_acceleration_covariance[8] = 0.04;

    pub_imu_->publish(imu_msg);
  }

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
  rclcpp::TimerBase::SharedPtr timer_;
  BNO055 imu_;
};

// ==========================================
// 3. 메인 함수
// ==========================================
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuDriverNode>());
  rclcpp::shutdown();
  return 0;
}