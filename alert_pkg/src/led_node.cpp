#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <string>

class LedNode : public rclcpp::Node {
public:
  LedNode() : Node("led_node"), serial_fd_(-1) {

    // ===== 파라미터 =====
    serial_port_ = this->declare_parameter<std::string>("serial_port", "/dev/ttyCH341USB2");
    baud_rate_   = this->declare_parameter<int>("baud_rate", 115200);

    // ===== 시리얼 포트 초기화 =====
    if (!open_serial(serial_port_, baud_rate_)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", serial_port_.c_str());
    } else {
      RCLCPP_INFO(this->get_logger(), "Serial port opened: %s @ %d baud",
                  serial_port_.c_str(), baud_rate_);
    }

    // ===== 구독 =====
    mode_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/robot_mode", 10,
        std::bind(&LedNode::mode_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "led_node initialized");
  }

  ~LedNode() {
    send_command("OFF");
    if (serial_fd_ >= 0) {
      close(serial_fd_);
    }
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;

  std::string serial_port_;
  int         baud_rate_;
  int         serial_fd_;
  std::string current_mode_{"STANDBY"};

  void mode_callback(const std_msgs::msg::String::SharedPtr msg) {
    const std::string &mode = msg->data;
    if (mode == current_mode_) return;

    current_mode_ = mode;
    RCLCPP_INFO(this->get_logger(), "robot_mode -> %s", current_mode_.c_str());

    if (current_mode_ == "DRIVING") {
      send_command("ON");
    } else {
      // ARRIVED, RETURNING, STANDBY → 경고등 소등
      send_command("OFF");
    }
  }

  void send_command(const std::string &cmd) {
    if (serial_fd_ < 0) {
      RCLCPP_WARN(this->get_logger(), "Serial not open, skip command: %s", cmd.c_str());
      return;
    }

    std::string data = cmd + "\n";
    ssize_t written = write(serial_fd_, data.c_str(), data.size());
    if (written < 0) {
      RCLCPP_ERROR(this->get_logger(), "Serial write failed for command: %s", cmd.c_str());
    } else {
      RCLCPP_INFO(this->get_logger(), "LED command sent: %s", cmd.c_str());
    }
  }

  bool open_serial(const std::string &port, int baud) {
    serial_fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (serial_fd_ < 0) return false;

    struct termios tty;
    if (tcgetattr(serial_fd_, &tty) != 0) {
      close(serial_fd_);
      serial_fd_ = -1;
      return false;
    }

    speed_t speed = baud_to_speed(baud);
    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);

    // 8N1, raw mode
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |=  CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |=  CREAD | CLOCAL;
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    tty.c_oflag &= ~OPOST;
    tty.c_oflag &= ~ONLCR;
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
      close(serial_fd_);
      serial_fd_ = -1;
      return false;
    }

    return true;
  }

  static speed_t baud_to_speed(int baud) {
    switch (baud) {
      case 9600:   return B9600;
      case 19200:  return B19200;
      case 38400:  return B38400;
      case 57600:  return B57600;
      case 115200: return B115200;
      default:     return B115200;
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LedNode>());
  rclcpp::shutdown();
  return 0;
}
