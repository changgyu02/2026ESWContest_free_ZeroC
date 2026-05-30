#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

#include <string>
#include <chrono>

// ===================================================
//  상태 정의
//  STANDBY       : 대기 (목표 없음)
//  DRIVING       : 신고지 이동 중 (경고음 + 경고등 ON)
//  OBSTACLE_WAIT : 장애물 감지 → 정지 대기
//  REPLANNING    : 장애물 해제 후 재계획 요청
//  ARRIVED       : 신고지 도착 (경고음 + 경고등 OFF)
//  RETURNING     : 원점 복귀 중 (경고음 + 경고등 OFF)
// ===================================================

enum class State {
    STANDBY,
    DRIVING,
    OBSTACLE_WAIT,
    REPLANNING,
    ARRIVED,
    RETURNING
};

static std::string state_str(State s) {
    switch (s) {
        case State::STANDBY:       return "STANDBY";
        case State::DRIVING:       return "DRIVING";
        case State::OBSTACLE_WAIT: return "OBSTACLE_WAIT";
        case State::REPLANNING:    return "REPLANNING";
        case State::ARRIVED:       return "ARRIVED";
        case State::RETURNING:     return "RETURNING";
    }
    return "UNKNOWN";
}

class MissionNode : public rclcpp::Node {
public:
    MissionNode() : Node("mission_node"), state_(State::STANDBY) {

        // ===== 파라미터 =====
        obstacle_wait_s_ = this->declare_parameter("obstacle_wait_s", 3.0);
        replan_delay_s_  = this->declare_parameter("replan_delay_s",  0.5);
        arrived_wait_s_  = this->declare_parameter("arrived_wait_s",  3.0);  // 신고지 도착 후 대기 시간
        home_x_          = this->declare_parameter("home_x", 0.0);           // 원점 x 좌표
        home_y_          = this->declare_parameter("home_y", 0.0);           // 원점 y 좌표

        // ===== 구독 =====
        goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10,
            std::bind(&MissionNode::on_goal, this, std::placeholders::_1));

        obstacle_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/obstacle_detected", 10,
            std::bind(&MissionNode::on_obstacle, this, std::placeholders::_1));

        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/planned_path", 10,
            std::bind(&MissionNode::on_path, this, std::placeholders::_1));

        arrived_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/goal_reached", 10,
            std::bind(&MissionNode::on_arrived, this, std::placeholders::_1));

        // ===== 발행 =====
        mode_pub_ = this->create_publisher<std_msgs::msg::String>("/robot_mode", 10);
        goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);

        // ===== 감시 타이머 (10Hz) =====
        monitor_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&MissionNode::monitor_loop, this));

        publish_mode(state_);
        RCLCPP_INFO(this->get_logger(),
            "mission_node started | state=STANDBY | obstacle_wait=%.1fs | home=(%.2f, %.2f)",
            obstacle_wait_s_, home_x_, home_y_);
    }

private:
    // ===== ROS IO =====
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr            goal_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr                        obstacle_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr                        path_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr                        arrived_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr                         mode_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr               goal_pub_;
    rclcpp::TimerBase::SharedPtr                                                monitor_timer_;

    // ===== 상태 =====
    State state_;
    bool obstacle_{false};
    bool has_goal_{false};
    bool path_empty_{true};
    State pre_obstacle_state_{State::DRIVING};  // 장애물 감지 전 상태 기억 (복귀/이동 구분)

    // ===== 타이머용 시각 =====
    rclcpp::Time obstacle_detected_time_;
    rclcpp::Time arrived_time_;

    // ===== 파라미터 =====
    double obstacle_wait_s_;
    double replan_delay_s_;
    double arrived_wait_s_;
    double home_x_;
    double home_y_;

    // ===== 콜백 =====
    void on_goal(const geometry_msgs::msg::PoseStamped::SharedPtr /*msg*/) {
        has_goal_ = true;
        // 외부 목표는 대기/도착 상태에서만 수락
        if (state_ == State::STANDBY || state_ == State::ARRIVED) {
            transition(State::DRIVING);
        }
    }

    void on_obstacle(const std_msgs::msg::Bool::SharedPtr msg) {
        bool prev = obstacle_;
        obstacle_ = msg->data;

        if (!prev && obstacle_) {
            if (state_ == State::DRIVING || state_ == State::RETURNING) {
                pre_obstacle_state_     = state_;
                obstacle_detected_time_ = this->now();
                transition(State::OBSTACLE_WAIT);
            }
        } else if (prev && !obstacle_) {
            if (state_ == State::OBSTACLE_WAIT) {
                transition(State::REPLANNING);
            }
        }
    }

    void on_path(const nav_msgs::msg::Path::SharedPtr msg) {
        path_empty_ = msg->poses.empty();

        // 이동/복귀 중 경로 계획 실패 시 STANDBY
        if ((state_ == State::DRIVING || state_ == State::RETURNING)
            && path_empty_ && has_goal_) {
            RCLCPP_WARN(this->get_logger(), "Path empty in %s → back to STANDBY",
                        state_str(state_).c_str());
            has_goal_ = false;
            transition(State::STANDBY);
        }
    }

    void on_arrived(const std_msgs::msg::Bool::SharedPtr msg) {
        if (!msg->data) return;

        if (state_ == State::DRIVING) {
            // 신고지 도착 → ARRIVED (경고음/경고등 OFF)
            arrived_time_ = this->now();
            transition(State::ARRIVED);

        } else if (state_ == State::RETURNING) {
            // 원점 복귀 완료 → STANDBY
            has_goal_ = false;
            transition(State::STANDBY);
        }
    }

    // ===== 감시 루프 =====
    void monitor_loop() {
        auto now = this->now();

        switch (state_) {
        case State::OBSTACLE_WAIT: {
            double wait = (now - obstacle_detected_time_).seconds();
            if (static_cast<int>(wait) % 5 == 0 && wait > 1.0) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                    "Obstacle persists for %.0fs...", wait);
            }
            break;
        }

        case State::REPLANNING: {
            // 장애물 해제 후 잠깐 대기 → 이동 전 상태로 복귀
            double delay = (now - obstacle_detected_time_).seconds();
            if (delay >= obstacle_wait_s_ + replan_delay_s_) {
                RCLCPP_INFO(this->get_logger(), "Obstacle cleared → %s (replan)",
                            state_str(pre_obstacle_state_).c_str());
                transition(pre_obstacle_state_);
            }
            break;
        }

        case State::ARRIVED: {
            // 신고지에서 arrived_wait_s_ 대기 후 원점으로 복귀
            if ((now - arrived_time_).seconds() > arrived_wait_s_) {
                publish_home_goal();
                transition(State::RETURNING);
            }
            break;
        }

        default:
            break;
        }
    }

    // ===== 원점 복귀 목표 발행 =====
    void publish_home_goal() {
        geometry_msgs::msg::PoseStamped goal;
        goal.header.stamp    = this->now();
        goal.header.frame_id = "map";
        goal.pose.position.x    = home_x_;
        goal.pose.position.y    = home_y_;
        goal.pose.orientation.w = 1.0;
        goal_pub_->publish(goal);
        RCLCPP_INFO(this->get_logger(), "Home goal published: (%.2f, %.2f)", home_x_, home_y_);
    }

    // ===== 상태 전환 =====
    void transition(State next) {
        if (state_ == next) return;
        RCLCPP_INFO(this->get_logger(), "State: %s → %s",
            state_str(state_).c_str(), state_str(next).c_str());
        state_ = next;
        publish_mode(next);
    }

    void publish_mode(State s) {
        std_msgs::msg::String msg;
        // REPLANNING은 이전 이동 상태(DRIVING or RETURNING)를 그대로 전달
        if (s == State::REPLANNING) {
            msg.data = state_str(pre_obstacle_state_);
        } else {
            msg.data = state_str(s);
        }
        mode_pub_->publish(msg);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MissionNode>());
    rclcpp::shutdown();
    return 0;
}
