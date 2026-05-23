#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <cstdlib>
#include <string>
#include <thread>
#include <atomic>
#include <mutex>

class SpeakerNode : public rclcpp::Node {
public:
  SpeakerNode() : Node("speaker_node"), playing_(false) {

    // ===== 파라미터 =====
    alert_mp3_   = this->declare_parameter<std::string>(
                     "alert_mp3", "/home/lee/sounds/alert.mp3");    // 주행 중 경고음
    arrived_mp3_ = this->declare_parameter<std::string>(
                     "arrived_mp3", "/home/lee/sounds/arrived.mp3"); // 도착 알림음

    // ===== 구독 =====
    mode_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/robot_mode", 10,
        std::bind(&SpeakerNode::mode_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "speaker_node initialized");
    RCLCPP_INFO(this->get_logger(), "alert_mp3:   %s", alert_mp3_.c_str());
    RCLCPP_INFO(this->get_logger(), "arrived_mp3: %s", arrived_mp3_.c_str());
  }

  ~SpeakerNode() {
    stop_play();
  }

private:
  // ===== ROS IO =====
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;

  // ===== 파라미터 =====
  std::string alert_mp3_;
  std::string arrived_mp3_;

  // ===== 재생 상태 =====
  std::atomic<bool> playing_{false};
  std::atomic<bool> stop_flag_{false};
  std::thread play_thread_;
  std::mutex thread_mutex_;
  std::string current_mode_{"STANDBY"};

  // ===== 콜백 =====
  void mode_callback(const std_msgs::msg::String::SharedPtr msg) {
    std::string new_mode = msg->data;
    if (new_mode == current_mode_) return;

    current_mode_ = new_mode;
    RCLCPP_INFO(this->get_logger(), "robot_mode -> %s", current_mode_.c_str());

    if (current_mode_ == "DRIVING") {
      // 주행 중 경고음 반복 재생
      start_play(alert_mp3_, true);

    } else if (current_mode_ == "ARRIVED") {
      // 도착 알림음 1회 재생
      start_play(arrived_mp3_, false);

    } else if (current_mode_ == "STANDBY") {
      // 정지
      stop_play();
    }
  }

  // ===== 재생 시작 =====
  void start_play(const std::string &mp3_path, bool loop) {
    stop_play(); // 기존 재생 중지

    stop_flag_ = false;
    playing_   = true;

    play_thread_ = std::thread([this, mp3_path, loop]() {
      RCLCPP_INFO(this->get_logger(), "Start playing: %s (loop=%s)",
                  mp3_path.c_str(), loop ? "true" : "false");

      while (!stop_flag_) {
        // mpg123으로 mp3 재생
        std::string cmd = "mpg123 -q " + mp3_path;
        int ret = std::system(cmd.c_str());

        if (ret != 0) {
          RCLCPP_ERROR(this->get_logger(),
                       "mpg123 failed (ret=%d). Check file path: %s",
                       ret, mp3_path.c_str());
          break;
        }

        // 반복 재생이 아니면 한 번만 재생 후 종료
        if (!loop) break;

        // 중지 신호 확인
        if (stop_flag_) break;
      }

      playing_ = false;
      RCLCPP_INFO(this->get_logger(), "Playback stopped: %s", mp3_path.c_str());
    });
  }

  // ===== 재생 중지 =====
  void stop_play() {
    stop_flag_ = true;

    // 현재 재생 중인 mpg123 프로세스 강제 종료
    std::system("pkill -f mpg123");

    // 스레드 종료 대기
    if (play_thread_.joinable()) {
      play_thread_.join();
    }

    playing_ = false;
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SpeakerNode>());
  rclcpp::shutdown();
  return 0;
}