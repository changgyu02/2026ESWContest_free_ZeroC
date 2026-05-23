#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

#include <cmath>

class OdomPublisherNode : public rclcpp::Node {
public:
  OdomPublisherNode() : Node("odom_publisher") {

    // ===== 파라미터 =====
    wheel_radius_ = this->declare_parameter<double>("wheel_radius", 0.08);
    wheel_base_   = this->declare_parameter<double>("wheel_base",   0.19);
    publish_tf_   = this->declare_parameter<bool>("publish_tf", false);

    // ===== 구독 =====
    encoder_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        "/encoder", 10,
        std::bind(&OdomPublisherNode::encoder_cb, this, std::placeholders::_1));

    // ===== 발행 =====
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

    if (publish_tf_) {
      tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
      // slam_toolbox가 구동 직후 TF를 찾지 못해 실패하는 것을 방지하기 위해
      // 엔코더 데이터 유무와 무관하게 20Hz로 TF를 발행한다.
      tf_timer_ = this->create_wall_timer(
          std::chrono::milliseconds(50),
          std::bind(&OdomPublisherNode::tf_timer_cb, this));
    }

    // ===== 초기 상태 =====
    x_     = 0.0;
    y_     = 0.0;
    theta_ = 0.0;
    last_stamp_ = this->now();

    RCLCPP_INFO(this->get_logger(),
                "odom_publisher started (wheel_radius=%.3fm, wheel_base=%.3fm, publish_tf=%s)",
                wheel_radius_, wheel_base_, publish_tf_ ? "true" : "false");
  }

private:
  // ===== ROS IO =====
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr encoder_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr              odom_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster>                     tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr                                       tf_timer_;

  // ===== 파라미터 =====
  double wheel_radius_;
  double wheel_base_;
  bool   publish_tf_;

  // ===== 상태 =====
  double x_;
  double y_;
  double theta_;
  rclcpp::Time last_stamp_;

  // ===== 콜백 =====
  void encoder_cb(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
    rclcpp::Time now = msg->header.stamp;
    double dt = (now - last_stamp_).seconds();
    last_stamp_ = now;

    if (dt <= 0.0 || dt > 1.0) return;

    double linear_x  = msg->twist.linear.x;
    double angular_z = msg->twist.angular.z;

    double delta_x     = linear_x * std::cos(theta_) * dt;
    double delta_y     = linear_x * std::sin(theta_) * dt;
    double delta_theta = angular_z * dt;

    x_     += delta_x;
    y_     += delta_y;
    theta_ += delta_theta;

    while (theta_ >  M_PI) theta_ -= 2.0 * M_PI;
    while (theta_ < -M_PI) theta_ += 2.0 * M_PI;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, theta_);

    // ===== /odom 발행 =====
    nav_msgs::msg::Odometry odom;
    odom.header.stamp    = now;
    odom.header.frame_id = "odom";
    odom.child_frame_id  = "base_link";

    odom.pose.pose.position.x  = x_;
    odom.pose.pose.position.y  = y_;
    odom.pose.pose.position.z  = 0.0;
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    odom.twist.twist.linear.x  = linear_x;
    odom.twist.twist.angular.z = angular_z;

    odom.pose.covariance[0]  = 0.01;
    odom.pose.covariance[7]  = 0.01;
    odom.pose.covariance[35] = 0.01;
    odom.twist.covariance[0]  = 0.01;
    odom.twist.covariance[35] = 0.01;

    odom_pub_->publish(odom);

    RCLCPP_DEBUG(this->get_logger(),
                 "odom: x=%.3f, y=%.3f, theta=%.3f rad",
                 x_, y_, theta_);
  }

  // ===== TF 타이머 (20Hz) - 엔코더 데이터 유무와 무관하게 발행 =====
  void tf_timer_cb() {
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, theta_);

    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp            = this->now();
    tf_msg.header.frame_id         = "odom";
    tf_msg.child_frame_id          = "base_link";
    tf_msg.transform.translation.x = x_;
    tf_msg.transform.translation.y = y_;
    tf_msg.transform.translation.z = 0.0;
    tf_msg.transform.rotation.x    = q.x();
    tf_msg.transform.rotation.y    = q.y();
    tf_msg.transform.rotation.z    = q.z();
    tf_msg.transform.rotation.w    = q.w();
    tf_broadcaster_->sendTransform(tf_msg);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomPublisherNode>());
  rclcpp::shutdown();
  return 0;
}
