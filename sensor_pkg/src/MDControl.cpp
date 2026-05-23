// md_control_node.cpp
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <linux/serial.h>
#include <cstring>
#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#define UART_PORT "/dev/ttyCH341USB0"
#define BAUD_RATE B115200

// [0x55, address, speed, checksum]
//  - address: [2:0]=base address(0~7), [3]=right-motor flag
class MDControl {
public:
    MDControl() : uart_fd_(-1) {
        uart_fd_ = ::open(UART_PORT, O_RDWR | O_NOCTTY | O_NDELAY);
        if (uart_fd_ == -1) {
            RCLCPP_ERROR(rclcpp::get_logger("MDControl"), "UART 포트 열기 실패 (%s)", UART_PORT);
            return;
        }

        fcntl(uart_fd_, F_SETFL, 0);

        if (configureUART() != 0) {
            ::close(uart_fd_);
            uart_fd_ = -1;
            return;
        }

        setDTRRTS(/*dtr=*/true, /*rts=*/false);

        RCLCPP_INFO(rclcpp::get_logger("MDControl"),
                    "UART 연결 완료 (포트: %s, 115200-8N1, DTR=ON, RTS=OFF)", UART_PORT);
    }

    ~MDControl() {
        if (uart_fd_ != -1) ::close(uart_fd_);
    }

    void sendSpeed(double linear_x, double angular_z) {
        int left  = static_cast<int>((linear_x - angular_z) * 50.0 + 127.0);
        int right = static_cast<int>((linear_x + angular_z) * 50.0 + 127.0);

        left  = std::clamp(left,  0, 255);
        right = std::clamp(right, 0, 255);

        sendPacket(0, false, static_cast<uint8_t>(left));
        sendPacket(0, true,  static_cast<uint8_t>(right));

        RCLCPP_DEBUG(rclcpp::get_logger("MDControl"), "TX cmd: left=%d, right=%d", left, right);
    }

private:
    int uart_fd_;

    int configureUART() {
        struct termios tty{};
        if (tcgetattr(uart_fd_, &tty) != 0) {
            RCLCPP_ERROR(rclcpp::get_logger("MDControl"), "시리얼 속성 가져오기 실패");
            return -1;
        }

        cfsetospeed(&tty, BAUD_RATE);
        cfsetispeed(&tty, BAUD_RATE);

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

        tcflush(uart_fd_, TCIFLUSH);
        if (tcsetattr(uart_fd_, TCSANOW, &tty) != 0) {
            RCLCPP_ERROR(rclcpp::get_logger("MDControl"), "시리얼 속성 적용 실패");
            return -1;
        }
        return 0;
    }

    void setDTRRTS(bool dtr_on, bool rts_on) {
        int status = 0;
        if (ioctl(uart_fd_, TIOCMGET, &status) == -1) {
            RCLCPP_WARN(rclcpp::get_logger("MDControl"), "TIOCMGET 실패");
            return;
        }
        if (dtr_on) status |= TIOCM_DTR; else status &= ~TIOCM_DTR;
        if (rts_on) status |= TIOCM_RTS; else status &= ~TIOCM_RTS;
        if (ioctl(uart_fd_, TIOCMSET, &status) == -1) {
            RCLCPP_WARN(rclcpp::get_logger("MDControl"), "TIOCMSET 실패");
        }
    }

    void sendPacket(uint8_t base_address, bool is_right_motor, uint8_t speed) {
        if (uart_fd_ == -1) return;

        const uint8_t header  = 0x55;
        uint8_t address       = (base_address & 0x07);
        if (is_right_motor) address |= 0x08;

        const uint8_t command  = speed;
        const uint8_t checksum = static_cast<uint8_t>((header + address + command) & 0xFF);

        uint8_t pkt[4] = { header, address, command, checksum };

        ssize_t w = ::write(uart_fd_, pkt, sizeof(pkt));
        if (w != static_cast<ssize_t>(sizeof(pkt))) {
            RCLCPP_WARN(rclcpp::get_logger("MDControl"), "write 미완료 (%zd/4)", w);
        }

        tcdrain(uart_fd_);
    }
};

class MDControlNode : public rclcpp::Node {
public:
    MDControlNode() : Node("md_control") {
        // ===== PID 파라미터 =====
        kp_           = this->declare_parameter("kp_straight",       0.3);
        ki_           = this->declare_parameter("ki_straight",       0.05);
        kd_           = this->declare_parameter("kd_straight",       0.02);
        w_threshold_  = this->declare_parameter("straight_w_threshold", 0.05);  // [rad/s]

        // ===== 구독 =====
        sub_cmd_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&MDControlNode::onCmdVel, this, std::placeholders::_1));

        sub_enc_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "/encoder", 10,
            std::bind(&MDControlNode::onEncoder, this, std::placeholders::_1));

        // ===== 20Hz PID 루프 타이머 =====
        pid_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&MDControlNode::pidLoop, this));

        last_cmd_time_ = this->now();

        RCLCPP_INFO(this->get_logger(),
            "MDControl Node ready (PID 직진보정 활성화 | kp=%.2f ki=%.2f kd=%.2f thr=%.2f rad/s)",
            kp_, ki_, kd_, w_threshold_);
    }

private:
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr     sub_cmd_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_enc_;
    rclcpp::TimerBase::SharedPtr pid_timer_;
    MDControl md_;

    // 명령 속도
    double cmd_v_{0.0};
    double cmd_w_{0.0};
    rclcpp::Time last_cmd_time_;

    // 엔코더 실측 속도
    double actual_w_{0.0};
    bool has_encoder_{false};

    // PID 상태
    double pid_intg_{0.0};
    double pid_prev_err_{0.0};

    // 파라미터
    double kp_, ki_, kd_, w_threshold_;

    static constexpr double DT             = 0.05;   // [s] 20Hz
    static constexpr double CMD_TIMEOUT_S  = 0.5;    // [s] 이 시간 이상 cmd_vel 없으면 정지
    static constexpr double INTG_LIMIT     = 1.0;    // anti-windup 상한

    void onCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg) {
        cmd_v_ = msg->linear.x;
        cmd_w_ = msg->angular.z;
        last_cmd_time_ = this->now();
    }

    void onEncoder(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
        actual_w_ = msg->twist.angular.z;
        has_encoder_ = true;
    }

    void pidLoop() {
        // cmd_vel 타임아웃 → 정지
        if ((this->now() - last_cmd_time_).seconds() > CMD_TIMEOUT_S) {
            md_.sendSpeed(0.0, 0.0);
            resetPid();
            return;
        }

        // 직진 여부 판단: 선속도 있고 회전 명령이 작을 때만 PID 활성화
        bool is_straight = (std::abs(cmd_v_) > 0.01) &&
                           (std::abs(cmd_w_) < w_threshold_);

        if (!is_straight || !has_encoder_) {
            md_.sendSpeed(cmd_v_, cmd_w_);
            resetPid();
            return;
        }

        // ===== PID 직진 보정 =====
        // 목표: actual_w_ = 0
        double err   = -actual_w_;
        pid_intg_    = std::clamp(pid_intg_ + err * DT, -INTG_LIMIT, INTG_LIMIT);
        double deriv = (err - pid_prev_err_) / DT;
        double corr  = kp_ * err + ki_ * pid_intg_ + kd_ * deriv;
        pid_prev_err_ = err;

        md_.sendSpeed(cmd_v_, cmd_w_ + corr);

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
            "[PID] actual_w=%.3f err=%.3f corr=%.3f w_out=%.3f",
            actual_w_, err, corr, cmd_w_ + corr);
    }

    void resetPid() {
        pid_intg_     = 0.0;
        pid_prev_err_ = 0.0;
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MDControlNode>());
    rclcpp::shutdown();
    return 0;
}
