#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <cstring>
#include <cstdint>
#include <vector>
#include <thread>
#include <atomic>

// ===== 패킷 정의 (ESP32와 동일) =====
const uint8_t FB_HDR0     = 0xAA;
const uint8_t FB_HDR1     = 0x55;
const uint8_t FB_TYPE_VEL = 0x20;
const uint8_t FB_VEL_LEN  = 12;
const size_t  PACKET_SIZE = 17; // header(2) + type(1) + len(1) + payload(12) + checksum(1)

class EncoderDriverNode : public rclcpp::Node {
public:
  EncoderDriverNode() : Node("encoder_driver"), running_(false) {

    // ===== 파라미터 =====
    port_name_ = this->declare_parameter<std::string>("port", "/dev/ttyCH341USB1");
    baud_rate_ = this->declare_parameter<int>("baud_rate", 115200);

    // ===== 퍼블리셔 =====
    // linear_x, angular_z를 TwistStamped로 발행
    encoder_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
        "/encoder", 10);

    // ===== 시리얼 포트 오픈 =====
    if (!open_serial()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", port_name_.c_str());
      rclcpp::shutdown();
      return;
    }

    RCLCPP_INFO(this->get_logger(), "encoder_driver started (port=%s)", port_name_.c_str());

    // ===== 수신 스레드 시작 =====
    running_ = true;
    recv_thread_ = std::thread(&EncoderDriverNode::recv_loop, this);
  }

  ~EncoderDriverNode() {
    running_ = false;
    if (recv_thread_.joinable()) recv_thread_.join();
    if (fd_ >= 0) close(fd_);
  }

private:
  // ===== ROS IO =====
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr encoder_pub_;

  // ===== 시리얼 =====
  int fd_{-1};
  std::string port_name_;
  int baud_rate_;

  // ===== 스레드 =====
  std::thread recv_thread_;
  std::atomic<bool> running_;

  // ===== 수신 버퍼 =====
  std::vector<uint8_t> buf_;

  // ===== 시리얼 포트 오픈 =====
  bool open_serial() {
    fd_ = open(port_name_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd_ < 0) return false;

    fcntl(fd_, F_SETFL, 0);

    struct termios tty{};
    tcgetattr(fd_, &tty);

    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag |= CLOCAL | CREAD;
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_iflag = IGNPAR;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_oflag = 0;
    tty.c_lflag = 0;
    tty.c_cc[VMIN]  = 1;
    tty.c_cc[VTIME] = 0;

    tcflush(fd_, TCIFLUSH);
    tcsetattr(fd_, TCSANOW, &tty);

    // DTR=ON, RTS=OFF (MDControl과 동일 - 시리얼 모니터 효과 재현)
    int modem_bits;
    ioctl(fd_, TIOCMGET, &modem_bits);
    modem_bits |= TIOCM_DTR;
    modem_bits &= ~TIOCM_RTS;
    ioctl(fd_, TIOCMSET, &modem_bits);

    return true;
  }

  // ===== 수신 루프 =====
  void recv_loop() {
    uint8_t byte;
    while (running_) {
      ssize_t n = read(fd_, &byte, 1);
      if (n <= 0) continue;

      buf_.push_back(byte);

      // 헤더 동기화
      while (buf_.size() >= 2 &&
             !(buf_[0] == FB_HDR0 && buf_[1] == FB_HDR1)) {
        buf_.erase(buf_.begin());
      }

      // 패킷 완성 시 파싱
      if (buf_.size() >= PACKET_SIZE) {
        parse_packet(buf_);
        buf_.clear();
      }
    }
  }

  // ===== 패킷 파싱 =====
  void parse_packet(const std::vector<uint8_t> &pkt) {
    // 헤더 확인
    if (pkt[0] != FB_HDR0 || pkt[1] != FB_HDR1) return;
    if (pkt[2] != FB_TYPE_VEL) return;
    if (pkt[3] != FB_VEL_LEN) return;

    // 체크섬 검증
    uint8_t sum = 0;
    for (size_t i = 2; i < PACKET_SIZE - 1; i++) sum += pkt[i];
    if (sum != pkt[PACKET_SIZE - 1]) {
      RCLCPP_WARN(this->get_logger(), "Checksum mismatch");
      return;
    }

    // 페이로드 파싱
    float linear_x, angular_z;
    uint16_t dt_ms, seq;

    memcpy(&linear_x,  &pkt[4], 4);
    memcpy(&angular_z, &pkt[8], 4);
    memcpy(&dt_ms,     &pkt[12], 2);
    memcpy(&seq,       &pkt[14], 2);

    // TwistStamped로 발행
    geometry_msgs::msg::TwistStamped msg;
    msg.header.stamp    = this->now();
    msg.header.frame_id = "base_link";
    msg.twist.linear.x  = linear_x;
    msg.twist.angular.z = angular_z;

    encoder_pub_->publish(msg);

    RCLCPP_DEBUG(this->get_logger(),
                 "seq=%d, lin=%.3f, ang=%.3f, dt=%dms",
                 seq, linear_x, angular_z, dt_ms);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EncoderDriverNode>());
  rclcpp::shutdown();
  return 0;
}