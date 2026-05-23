#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <functional>

class ObstacleDetectorNode : public rclcpp::Node {
public:
  ObstacleDetectorNode()
  : Node("obstacle_detector_node"),
    obstacle_state_(false),
    release_counter_(0)
  {
    this->declare_parameter<float>("detect_threshold", 10.0f);
    this->declare_parameter<float>("release_threshold", 20.0f);
    this->declare_parameter<int>("release_required_count", 10);

    this->get_parameter("detect_threshold", detect_threshold_);
    this->get_parameter("release_threshold", release_threshold_);
    this->get_parameter("release_required_count", release_required_count_);

    distance_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "/ultrasonic_distance",
      rclcpp::QoS(rclcpp::KeepLast(10)),
      std::bind(&ObstacleDetectorNode::distance_callback, this, std::placeholders::_1)
    );

    obstacle_pub_ = this->create_publisher<std_msgs::msg::Bool>(
      "/obstacle_detected",
      rclcpp::QoS(rclcpp::KeepLast(10))
    );

    RCLCPP_INFO(this->get_logger(),
                "Obstacle Detector Node Started (detect=%.1fcm, release=%.1fcm, release_count=%d)",
                detect_threshold_, release_threshold_, release_required_count_);
  }

private:
  void distance_callback(const std_msgs::msg::Float32::SharedPtr msg) {
    float distance = msg->data;
    bool new_state = obstacle_state_;

    if (distance < detect_threshold_) {
      new_state = true;
      release_counter_ = 0;
    } else if (distance > release_threshold_) {
      if (obstacle_state_) {
        release_counter_++;
        if (release_counter_ >= release_required_count_) {
          new_state = false;
          release_counter_ = 0;
        }
      }
    }

    if (new_state != obstacle_state_) {
      obstacle_state_ = new_state;

      std_msgs::msg::Bool obs_msg;
      obs_msg.data = obstacle_state_;
      obstacle_pub_->publish(obs_msg);

      if (obstacle_state_) {
        RCLCPP_WARN(this->get_logger(), "Obstacle detected (distance=%.2f cm)", distance);
      } else {
        RCLCPP_INFO(this->get_logger(), "Obstacle cleared (consecutive %d)", release_required_count_);
      }
    }
  }

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr distance_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr obstacle_pub_;

  bool  obstacle_state_;
  float detect_threshold_;
  float release_threshold_;
  int   release_counter_;
  int   release_required_count_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObstacleDetectorNode>());
  rclcpp::shutdown();
  return 0;
}

