#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <cmath>
#include <vector>
#include <algorithm>
#include <limits>

class ControllerNode : public rclcpp::Node {
public:
    ControllerNode() : Node("controller_node") {

        // ===== 파라미터 =====
        lookahead_dist_  = this->declare_parameter("lookahead_dist",  0.7);   // [m] 복도 환경
        max_lin_speed_   = this->declare_parameter("max_lin_speed",   1.5);   // [m/s] 응급 상황
        max_ang_speed_   = this->declare_parameter("max_ang_speed",   1.0);   // [rad/s]
        control_rate_hz_ = this->declare_parameter("control_rate_hz", 20.0);  // [Hz]
        goal_tolerance_  = this->declare_parameter("goal_tolerance",  0.3);   // [m] 도착 판단 거리

        // 가감속 파라미터
        lin_accel_ = this->declare_parameter("lin_accel", 0.5);  // [m/s^2]
        lin_decel_ = this->declare_parameter("lin_decel", 0.8);  // [m/s^2]
        ang_accel_ = this->declare_parameter("ang_accel", 2.0);  // [rad/s^2]
        ang_decel_ = this->declare_parameter("ang_decel", 2.0);  // [rad/s^2]

        control_period_ = 1.0 / std::max(1.0, control_rate_hz_);

        // ===== 구독 =====
        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/planned_path", 10,
            std::bind(&ControllerNode::path_cb, this, std::placeholders::_1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 50,
            std::bind(&ControllerNode::odom_cb, this, std::placeholders::_1));

        obstacle_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/obstacle_detected", 10,
            std::bind(&ControllerNode::obstacle_cb, this, std::placeholders::_1));

        mode_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/robot_mode", 10,
            std::bind(&ControllerNode::mode_cb, this, std::placeholders::_1));

        // ===== 발행 =====
        cmd_pub_          = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        goal_reached_pub_ = this->create_publisher<std_msgs::msg::Bool>("/goal_reached", 10);

        // ===== 제어 루프 타이머 =====
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(control_period_),
            std::bind(&ControllerNode::control_loop, this));

        RCLCPP_INFO(this->get_logger(),
            "controller_node started | lookahead=%.2fm, max_v=%.2fm/s, max_w=%.2frad/s",
            lookahead_dist_, max_lin_speed_, max_ang_speed_);
    }

private:
    // ===== ROS IO =====
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr       path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr   odom_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr       obstacle_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr     mode_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr    cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr          goal_reached_pub_;
    rclcpp::TimerBase::SharedPtr                               timer_;

    // ===== 상태 =====
    nav_msgs::msg::Path      path_;
    nav_msgs::msg::Odometry  odom_;
    bool has_path_{false};
    bool has_odom_{false};
    bool obstacle_{false};

    // 모드
    std::string robot_mode_{"STANDBY"};

    // 램핑 상태
    double last_v_{0.0};
    double last_w_{0.0};

    // ===== 파라미터 =====
    double lookahead_dist_;
    double max_lin_speed_;
    double max_ang_speed_;
    double control_rate_hz_;
    double control_period_;
    double goal_tolerance_;
    double lin_accel_;
    double lin_decel_;
    double ang_accel_;
    double ang_decel_;

    // ===== 콜백 =====
    void path_cb(const nav_msgs::msg::Path::SharedPtr msg) {
        path_     = *msg;
        has_path_ = !path_.poses.empty();
        RCLCPP_INFO(this->get_logger(), "new path received: %zu poses", path_.poses.size());
    }

    void odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg) {
        odom_     = *msg;
        has_odom_ = true;
    }

    void obstacle_cb(const std_msgs::msg::Bool::SharedPtr msg) {
        obstacle_ = msg->data;
        if (obstacle_) {
            RCLCPP_WARN(this->get_logger(), "Obstacle detected -> STOP");
        } else {
            RCLCPP_INFO(this->get_logger(), "Obstacle cleared -> resume");
        }
    }

    void mode_cb(const std_msgs::msg::String::SharedPtr msg) {
        if (robot_mode_ != msg->data) {
            robot_mode_ = msg->data;
            RCLCPP_INFO(this->get_logger(), "robot_mode changed -> %s", robot_mode_.c_str());

            // 모드 전환 시 램핑 초기화
            if (robot_mode_ == "STANDBY" || robot_mode_ == "ARRIVED") {
                last_v_ = 0.0;
                last_w_ = 0.0;
            }
            // RETURNING은 램핑 유지 (정지 없이 계속 주행)
        }
    }

    // ===== 제어 루프 =====
    void control_loop() {
        if (!has_odom_) {
            publish_zero();
            return;
        }

        // STANDBY / ARRIVED → 정지
        if (robot_mode_ == "STANDBY" || robot_mode_ == "ARRIVED") {
            publish_zero();
            return;
        }

        // DRIVING / RETURNING 모드
        if (robot_mode_ == "DRIVING" || robot_mode_ == "RETURNING") {
            // 장애물 감지 시 정지
            if (obstacle_) {
                publish_zero();
                return;
            }

            if (!has_path_) {
                publish_zero();
                return;
            }

            // 도착 판단
            if (is_goal_reached()) {
                publish_zero();
                std_msgs::msg::Bool reached;
                reached.data = true;
                goal_reached_pub_->publish(reached);
                RCLCPP_INFO(this->get_logger(), "Goal reached -> /goal_reached");
                return;
            }

            // Pure Pursuit 실행
            pure_pursuit();
        }
    }

    // ===== Pure Pursuit =====
    void pure_pursuit() {
        const auto &pos = odom_.pose.pose.position;
        const auto &q   = odom_.pose.pose.orientation;
        double yaw      = yaw_from_quat(q);

        // 전방 목표점 탐색
        geometry_msgs::msg::Point target_pt;
        if (!get_lookahead_target(pos.x, pos.y, lookahead_dist_, target_pt)) {
            publish_zero();
            return;
        }

        // 목표점까지의 heading 오차 (alpha)
        double heading = std::atan2(target_pt.y - pos.y, target_pt.x - pos.x);
        double alpha   = normalize_angle(heading - yaw);

        // 곡률 계산 → 각속도 목표
        double curvature = (lookahead_dist_ > 1e-6)
                           ? (2.0 * std::sin(alpha) / lookahead_dist_)
                           : 0.0;

        double v_target = max_lin_speed_;
        double w_target = std::clamp(max_lin_speed_ * curvature,
                                     -max_ang_speed_, max_ang_speed_);

        // 가감속 램핑 적용
        last_v_ = slew_to(v_target, last_v_, lin_accel_, lin_decel_);
        last_w_ = slew_to(w_target, last_w_, ang_accel_, ang_decel_);

        geometry_msgs::msg::Twist cmd;
        cmd.linear.x  = last_v_;
        cmd.angular.z = last_w_;
        cmd_pub_->publish(cmd);

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
            "[PURE PURSUIT] alpha=%.1f deg, v=%.2f, w=%.2f",
            alpha * 180.0 / M_PI, last_v_, last_w_);
    }

    // ===== 도착 판단 =====
    bool is_goal_reached() {
        if (!has_path_ || path_.poses.empty()) return false;

        const auto &pos  = odom_.pose.pose.position;
        const auto &goal = path_.poses.back().pose.position;
        double dist = std::hypot(pos.x - goal.x, pos.y - goal.y);
        return dist < goal_tolerance_;
    }

    // ===== 전방 목표점 탐색 =====
    bool get_lookahead_target(double x, double y, double Ld,
                               geometry_msgs::msg::Point &target) {
        if (!has_path_ || path_.poses.size() < 2) return false;

        // 가장 가까운 경로점 탐색
        size_t nearest_idx  = 0;
        double nearest_dist = std::numeric_limits<double>::max();
        for (size_t i = 0; i < path_.poses.size(); ++i) {
            const auto &pp = path_.poses[i].pose.position;
            double d = std::hypot(pp.x - x, pp.y - y);
            if (d < nearest_dist) {
                nearest_dist = d;
                nearest_idx  = i;
            }
        }

        // nearest_idx 이후에서 lookahead 거리 이상인 점 탐색
        for (size_t i = nearest_idx; i < path_.poses.size(); ++i) {
            const auto &pp = path_.poses[i].pose.position;
            double d = std::hypot(pp.x - x, pp.y - y);
            if (d >= Ld) {
                target = pp;
                return true;
            }
        }

        // lookahead 거리 이상인 점 없으면 마지막 점 반환
        target = path_.poses.back().pose.position;
        return true;
    }

    // ===== 유틸 =====
    double slew_to(double target, double current,
                   double accel_limit, double decel_limit) {
        double delta = target - current;
        if (delta > 0.0) {
            double max_up = accel_limit * control_period_;
            delta = std::min(delta, max_up);
        } else {
            double max_down = decel_limit * control_period_;
            delta = std::max(delta, -max_down);
        }
        return current + delta;
    }

    static double normalize_angle(double a) {
        while (a >  M_PI) a -= 2.0 * M_PI;
        while (a < -M_PI) a += 2.0 * M_PI;
        return a;
    }

    static double yaw_from_quat(const geometry_msgs::msg::Quaternion &qmsg) {
        tf2::Quaternion q;
        tf2::fromMsg(qmsg, q);
        double r, p, y;
        tf2::Matrix3x3(q).getRPY(r, p, y);
        return y;
    }

    void publish_zero() {
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x  = 0.0;
        cmd.angular.z = 0.0;
        cmd_pub_->publish(cmd);
        last_v_ = 0.0;
        last_w_ = 0.0;
    }

};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControllerNode>());
    rclcpp::shutdown();
    return 0;
}