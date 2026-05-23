#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

using namespace std::chrono_literals;

class CameraDriverNode : public rclcpp::Node {
public:
  CameraDriverNode() : Node("camera_driver") {

    // ===== 파라미터 =====
    camera_index_  = this->declare_parameter<int>("camera_index", 0);       // /dev/video0
    frame_width_   = this->declare_parameter<int>("frame_width", 640);
    frame_height_  = this->declare_parameter<int>("frame_height", 480);
    fps_           = this->declare_parameter<int>("fps", 30);
    frame_id_      = this->declare_parameter<std::string>("frame_id", "camera_link");

    // ===== 카메라 오픈 =====
    cap_.open(camera_index_, cv::CAP_V4L2);
    if (!cap_.isOpened()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open camera index %d", camera_index_);
      rclcpp::shutdown();
      return;
    }

    cap_.set(cv::CAP_PROP_FRAME_WIDTH,  frame_width_);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, frame_height_);
    cap_.set(cv::CAP_PROP_FPS,          fps_);

    RCLCPP_INFO(this->get_logger(),
      "Camera opened: index=%d, resolution=%dx%d, fps=%d",
      camera_index_, frame_width_, frame_height_, fps_);

    // ===== 퍼블리셔 =====
    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/image_raw", 10);

    // ===== 타이머 (fps 주기로 프레임 캡처) =====
    int period_ms = static_cast<int>(1000.0 / fps_);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(period_ms),
      std::bind(&CameraDriverNode::timer_callback, this));
  }

  ~CameraDriverNode() {
    if (cap_.isOpened()) {
      cap_.release();
      RCLCPP_INFO(this->get_logger(), "Camera released");
    }
  }

private:
  // ===== ROS IO =====
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // ===== 카메라 =====
  cv::VideoCapture cap_;

  // ===== 파라미터 =====
  int camera_index_;
  int frame_width_;
  int frame_height_;
  int fps_;
  std::string frame_id_;

  // ===== 타이머 콜백 =====
  void timer_callback() {
    cv::Mat frame;
    if (!cap_.read(frame)) {
      RCLCPP_WARN(this->get_logger(), "Failed to capture frame");
      return;
    }

    if (frame.empty()) {
      RCLCPP_WARN(this->get_logger(), "Empty frame received");
      return;
    }

    // OpenCV BGR → ROS2 Image 메시지 변환
    auto msg = cv_bridge::CvImage(
      std_msgs::msg::Header(),
      "bgr8",
      frame
    ).toImageMsg();

    msg->header.stamp    = this->now();
    msg->header.frame_id = frame_id_;

    image_pub_->publish(*msg);
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraDriverNode>());
  rclcpp::shutdown();
  return 0;
}