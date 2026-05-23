#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/string.hpp>
#include <yaml-cpp/yaml.h>
#include <opencv2/opencv.hpp>

#include <queue>
#include <cmath>
#include <limits>
#include <map>
#include <vector>

using namespace std::chrono_literals;

class PlannerNode : public rclcpp::Node {
public:
  PlannerNode() : Node("planner_node") {

    // ===== 파라미터 =====
    map_yaml_path_         = this->declare_parameter<std::string>("map_yaml", "/home/lee/map.yaml");
    robot_radius_m_        = this->declare_parameter<double>("robot_radius_m", 0.25);
    max_snap_radius_m_     = this->declare_parameter<double>("max_snap_radius_m", 1.0);
    retry_scales_          = this->declare_parameter<std::vector<double>>(
                               "retry_scales", std::vector<double>{1.0, 0.85, 0.7});

    // 다운샘플 파라미터
    downsample_interval_m_ = this->declare_parameter<double>("downsample_interval_m", 0.1);
    angle_keep_deg_        = this->declare_parameter<double>("angle_keep_deg", 10.0);
    goal_dense_radius_m_   = this->declare_parameter<double>("goal_dense_radius_m", 0.5);
    goal_interval_m_       = this->declare_parameter<double>("goal_interval_m", 0.05);
    danger_distance_m_     = this->declare_parameter<double>("danger_distance_m", 0.2);
    danger_interval_m_     = this->declare_parameter<double>("danger_interval_m", 0.05);

    load_map(map_yaml_path_);

    // ===== 구독 =====
    amcl_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/amcl_pose", 10,
        std::bind(&PlannerNode::amcl_callback, this, std::placeholders::_1));

    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/goal_pose", 10,
        std::bind(&PlannerNode::goal_callback, this, std::placeholders::_1));

    mode_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/robot_mode", 10,
        std::bind(&PlannerNode::mode_callback, this, std::placeholders::_1));

    // ===== 발행 =====
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/planned_path", 10);
    mode_pub_ = this->create_publisher<std_msgs::msg::String>("/robot_mode", 10);

    RCLCPP_INFO(this->get_logger(), "planner_node initialized");
  }

private:
  // ===== ROS IO =====
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr               goal_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr                         mode_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr                              path_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr                            mode_pub_;

  // ===== 상태 =====
  geometry_msgs::msg::Pose        current_pose_;
  geometry_msgs::msg::PoseStamped goal_pose_;
  bool has_pose_{false};
  bool has_goal_{false};
  std::string robot_mode_{"STANDBY"};

  // ===== 맵 / 파라미터 =====
  std::string map_yaml_path_;
  cv::Mat base_map_gray_;
  cv::Mat inflated_map_;
  cv::Mat dist_to_obs_m_;
  double resolution_{0.05};
  std::vector<double> origin_;
  double robot_radius_m_;
  double max_snap_radius_m_;
  std::vector<double> retry_scales_;

  // 다운샘플 파라미터
  double downsample_interval_m_;
  double angle_keep_deg_;
  double goal_dense_radius_m_;
  double goal_interval_m_;
  double danger_distance_m_;
  double danger_interval_m_;

  // ===== 콜백 =====
  void amcl_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    current_pose_ = msg->pose.pose;
    has_pose_     = true;
    try_plan();
  }

  void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    if (std::isnan(msg->pose.position.x) || std::isnan(msg->pose.position.y)) {
      RCLCPP_WARN(this->get_logger(), "Invalid goal_pose: NaN -> skip");
      return;
    }
    goal_pose_ = *msg;
    has_goal_  = true;
    RCLCPP_INFO(this->get_logger(), "New goal: (%.2f, %.2f)",
                goal_pose_.pose.position.x, goal_pose_.pose.position.y);

    // goal 수신 시 자동으로 DRIVING 모드 전환
    if (robot_mode_ != "DRIVING") {
      robot_mode_ = "DRIVING";
      publish_mode("DRIVING");
      RCLCPP_INFO(this->get_logger(), "goal received -> DRIVING");
    }

    try_plan();
  }

  void mode_callback(const std_msgs::msg::String::SharedPtr msg) {
    // 자기 자신이 발행한 모드는 무시
    if (robot_mode_ == msg->data) return;

    robot_mode_ = msg->data;
    RCLCPP_INFO(this->get_logger(), "robot_mode -> %s", robot_mode_.c_str());

    // STANDBY / ARRIVED 시 빈 경로 발행
    if (robot_mode_ == "STANDBY" || robot_mode_ == "ARRIVED") {
      publish_empty_path();
    }
  }

  // ===== 맵 로드 =====
  void load_map(const std::string &yaml_path) {
    YAML::Node map_yaml = YAML::LoadFile(yaml_path);
    std::string pgm_path = map_yaml["image"].as<std::string>();
    if (pgm_path.find('/') != 0)
      pgm_path = yaml_path.substr(0, yaml_path.find_last_of('/')) + "/" + pgm_path;

    resolution_    = map_yaml["resolution"].as<double>();
    origin_        = map_yaml["origin"].as<std::vector<double>>();
    base_map_gray_ = cv::imread(pgm_path, cv::IMREAD_GRAYSCALE);

    if (base_map_gray_.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load map: %s", pgm_path.c_str());
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Map loaded: %s (%dx%d), resolution=%.4f m/px",
                pgm_path.c_str(), base_map_gray_.cols, base_map_gray_.rows, resolution_);

    make_inflated_map(robot_radius_m_);
  }

  void make_inflated_map(double robot_radius_m) {
    cv::Mat bin;
    cv::threshold(base_map_gray_, bin, 100, 255, cv::THRESH_BINARY);

    int r_px = std::max(0, static_cast<int>(std::round(robot_radius_m / resolution_)));
    if (r_px > 0) {
      int k          = 2 * r_px + 1;
      cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(k, k));
      cv::erode(bin, inflated_map_, kernel);
    } else {
      inflated_map_ = bin.clone();
    }

    cv::Mat dist_px;
    cv::distanceTransform(inflated_map_, dist_px, cv::DIST_L2, 3);
    dist_to_obs_m_ = dist_px * resolution_;

    RCLCPP_INFO(this->get_logger(),
                "Inflation done (radius=%.3fm, %dpx)", robot_radius_m, r_px);
  }

  // ===== 좌표 변환 =====
  inline bool map_ready() const {
    return !base_map_gray_.empty() && !inflated_map_.empty();
  }

  inline bool in_bounds(const cv::Point &pt) const {
    return pt.x >= 0 && pt.x < inflated_map_.cols &&
           pt.y >= 0 && pt.y < inflated_map_.rows;
  }

  inline bool is_free(const cv::Point &pt) const {
    return inflated_map_.at<uchar>(pt) > 200;
  }

  inline double obs_distance_m_at(const cv::Point &pt) const {
    if (!in_bounds(pt)) return 0.0;
    return dist_to_obs_m_.at<float>(pt);
  }

  cv::Point world_to_pixel(double x, double y) const {
    int px = static_cast<int>((x - origin_[0]) / resolution_);
    int py = static_cast<int>(inflated_map_.rows - (y - origin_[1]) / resolution_);
    return cv::Point(px, py);
  }

  geometry_msgs::msg::PoseStamped pixel_to_world_pose(int px, int py) const {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id    = "map";
    pose.header.stamp       = this->now();
    pose.pose.position.x    = px * resolution_ + origin_[0];
    pose.pose.position.y    = (inflated_map_.rows - py) * resolution_ + origin_[1];
    pose.pose.orientation.w = 1.0;
    return pose;
  }

  bool find_nearest_free_pixel(const cv::Point &from, int max_r_px, cv::Point &out,
                               bool use_base_map = false) const {
    auto chk = [&](const cv::Point &pt) -> bool {
      if (!in_bounds(pt)) return false;
      if (use_base_map) return base_map_gray_.at<uchar>(pt) > 50;
      return inflated_map_.at<uchar>(pt) > 200;
    };
    if (chk(from)) { out = from; return true; }
    for (int r = 1; r <= max_r_px; ++r) {
      for (int dy = -r; dy <= r; ++dy) {
        int dx = r - std::abs(dy);
        cv::Point p1(from.x - dx, from.y + dy);
        cv::Point p2(from.x + dx, from.y + dy);
        if (chk(p1)) { out = p1; return true; }
        if (chk(p2)) { out = p2; return true; }
      }
    }
    return false;
  }

  // ===== 경로 생성 =====
  void try_plan() {
    if (!has_pose_ || !has_goal_ || !map_ready()) return;
    if (robot_mode_ != "DRIVING") return;

    cv::Point start_pix = world_to_pixel(current_pose_.position.x,
                                          current_pose_.position.y);
    cv::Point goal_pix  = world_to_pixel(goal_pose_.pose.position.x,
                                          goal_pose_.pose.position.y);

    int snap_r_px = std::max(1, (int)std::round(max_snap_radius_m_ / resolution_));
    cv::Point snapped_start, snapped_goal;

    if (!find_nearest_free_pixel(start_pix, snap_r_px, snapped_start, true)) {
      RCLCPP_ERROR(this->get_logger(), "Start position snap failed");
      publish_empty_path();
      return;
    }
    if (!find_nearest_free_pixel(goal_pix, snap_r_px, snapped_goal, true)) {
      RCLCPP_ERROR(this->get_logger(), "Goal position snap failed");
      publish_empty_path();
      return;
    }

    for (double scale : retry_scales_) {
      make_inflated_map(robot_radius_m_ * scale);

      std::vector<cv::Point> px_path = a_star(snapped_start, snapped_goal);
      if (px_path.empty()) {
        RCLCPP_WARN(this->get_logger(), "A* failed with scale=%.2f, retrying...", scale);
        continue;
      }

      px_path = simplify_path(px_path);

      std::vector<geometry_msgs::msg::PoseStamped> raw_path;
      for (const auto &pt : px_path)
        raw_path.push_back(pixel_to_world_pose(pt.x, pt.y));

      std::vector<geometry_msgs::msg::PoseStamped> final_path =
          adaptive_downsample(raw_path);

      nav_msgs::msg::Path path_msg;
      path_msg.header.frame_id = "map";
      path_msg.header.stamp    = this->now();
      path_msg.poses           = final_path;

      path_pub_->publish(path_msg);
      RCLCPP_INFO(this->get_logger(),
                  "Path published: %zu poses (scale=%.2f)", final_path.size(), scale);
      return;
    }

    RCLCPP_ERROR(this->get_logger(), "Planning failed for all scales");
    publish_empty_path();
  }

  // ===== A* (8방향) =====
  std::vector<cv::Point> a_star(cv::Point start, cv::Point goal) {
    int rows = inflated_map_.rows, cols = inflated_map_.cols;
    auto inb  = [&](cv::Point p){ return p.x>=0 && p.x<cols && p.y>=0 && p.y<rows; };
    auto free = [&](cv::Point p){ return inflated_map_.at<uchar>(p) > 200; };

    if (!inb(start) || !inb(goal) || !free(start) || !free(goal)) return {};

    std::vector<cv::Point> dirs = {
      {1,0},{0,1},{-1,0},{0,-1},
      {1,1},{1,-1},{-1,1},{-1,-1}
    };

    auto cmp = [](const std::pair<double, cv::Point> &a,
                  const std::pair<double, cv::Point> &b){
      return a.first > b.first;
    };
    std::priority_queue<
      std::pair<double, cv::Point>,
      std::vector<std::pair<double, cv::Point>>,
      decltype(cmp)> open(cmp);

    auto lessPt = [](const cv::Point &a, const cv::Point &b){
      return (a.y == b.y) ? (a.x < b.x) : (a.y < b.y);
    };
    std::map<cv::Point, cv::Point, decltype(lessPt)> parent(lessPt);
    std::map<cv::Point, double,    decltype(lessPt)> g(lessPt);

    open.push({0.0, start});
    parent[start] = start;
    g[start]      = 0.0;

    while (!open.empty()) {
      cv::Point cur = open.top().second;
      open.pop();

      if (cur == goal) break;

      for (auto d : dirs) {
        cv::Point nxt = cur + d;
        if (!inb(nxt) || !free(nxt)) continue;
        double cost = g[cur] + cv::norm(d);
        if (!g.count(nxt) || cost < g[nxt]) {
          g[nxt]   = cost;
          double h = cv::norm(nxt - goal);
          open.push({cost + h, nxt});
          parent[nxt] = cur;
        }
      }
    }

    if (!parent.count(goal)) return {};

    std::vector<cv::Point> path;
    for (cv::Point c = goal; c != parent[c]; c = parent[c])
      path.push_back(c);
    path.push_back(start);
    std::reverse(path.begin(), path.end());
    return path;
  }

  // 직선 병합
  std::vector<cv::Point> simplify_path(const std::vector<cv::Point> &raw) {
    if (raw.size() <= 2) return raw;
    std::vector<cv::Point> out;
    out.push_back(raw.front());
    cv::Point prev_dir = raw[1] - raw[0];
    for (size_t i = 2; i < raw.size(); ++i) {
      cv::Point dir = raw[i] - raw[i-1];
      if (dir != prev_dir) {
        out.push_back(raw[i-1]);
        prev_dir = dir;
      }
    }
    out.push_back(raw.back());
    return out;
  }

  // 적응형 다운샘플
  std::vector<geometry_msgs::msg::PoseStamped> adaptive_downsample(
      const std::vector<geometry_msgs::msg::PoseStamped> &raw)
  {
    std::vector<geometry_msgs::msg::PoseStamped> out;
    if (raw.empty()) return out;
    if (raw.size() == 1) { out.push_back(raw.front()); return out; }

    auto dist2d = [](const geometry_msgs::msg::Point &a,
                     const geometry_msgs::msg::Point &b){
      return std::hypot(a.x - b.x, a.y - b.y);
    };

    auto angle_between = [](const cv::Point2d &a, const cv::Point2d &b) {
      double na = std::hypot(a.x, a.y), nb = std::hypot(b.x, b.y);
      if (na < 1e-9 || nb < 1e-9) return 0.0;
      double cosv = std::clamp((a.x*b.x + a.y*b.y) / (na*nb), -1.0, 1.0);
      return std::acos(cosv) * 180.0 / M_PI;
    };

    const auto &goal_pt = raw.back().pose.position;
    out.push_back(raw.front());
    auto last_kept = raw.front().pose.position;

    for (size_t i = 1; i + 1 < raw.size(); ++i) {
      const auto &prev = raw[i-1].pose.position;
      const auto &curr = raw[i].pose.position;
      const auto &next = raw[i+1].pose.position;

      cv::Point2d v1(curr.x - prev.x, curr.y - prev.y);
      cv::Point2d v2(next.x - curr.x, next.y - curr.y);
      double ang      = angle_between(v1, v2);
      bool keep_angle = (ang >= angle_keep_deg_);

      double local_interval = downsample_interval_m_;

      double dist_goal = dist2d(curr, goal_pt);
      if (dist_goal <= goal_dense_radius_m_)
        local_interval = std::min(local_interval, goal_interval_m_);

      cv::Point curr_px = world_to_pixel(curr.x, curr.y);
      if (obs_distance_m_at(curr_px) <= danger_distance_m_)
        local_interval = std::min(local_interval, danger_interval_m_);

      double seg = dist2d(last_kept, curr);
      if (keep_angle || seg >= local_interval) {
        out.push_back(raw[i]);
        last_kept = curr;
      }
    }

    out.push_back(raw.back());
    return out;
  }

  // ===== 유틸 =====
  void publish_empty_path() {
    nav_msgs::msg::Path p;
    p.header.frame_id = "map";
    p.header.stamp    = this->now();
    path_pub_->publish(p);
  }

  void publish_mode(const std::string &mode) {
    std_msgs::msg::String msg;
    msg.data = mode;
    mode_pub_->publish(msg);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}