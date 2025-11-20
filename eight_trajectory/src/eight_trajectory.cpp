#include "rclcpp/logging.hpp"
#include <chrono>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <thread>
#include <vector>

struct Waypoint {
  double dphi; // Delta yaw in radians (world frame)
  double dx;   // Delta x in meters (world frame)
  double dy;   // Delta y in meters (world frame)
};

const std::vector<Waypoint> figure_waypoints = {
    {0.0, 1.0, -1.0},     {0.0, 1.0, 1.0},       {0.0, 1.0, 1.0},
    {-1.5708, 1.0, -1.0}, {-1.5708, -1.0, -1.0}, {0.0, -1.0, 1.0},
    {0.0, -1.0, 1.0},     {0.0, -1.0, -1.0}};

class EightTrajectory : public rclcpp::Node {
public:
  EightTrajectory() : Node("eight_trajectory") {
    // Declare parameters with defaults and descriptions
    rcl_interfaces::msg::ParameterDescriptor desc;
    desc.description = "Wheel radius in meters";
    this->declare_parameter<double>("wheel_radius", 0.05, desc);

    desc.description = "Full wheel base (longitudinal distance between "
                       "front/rear axles) in meters";
    this->declare_parameter<double>("wheel_base", 0.17, desc);

    desc.description = "Full track width (lateral distance between "
                       "left/right wheels) in meters";
    this->declare_parameter<double>("track_width", 0.26969, desc);

    desc.description = "Desired maximum linear velocity in m/s";
    this->declare_parameter<double>("max_linear_vel", 0.5, desc);

    desc.description = "Desired maximum angular velocity in rad/s";
    this->declare_parameter<double>("max_angular_vel", 0.5, desc);

    desc.description = "Proportional gain for linear control";
    this->declare_parameter<double>("linear_kp", 2.0, desc);

    desc.description = "Proportional gain for angular control";
    this->declare_parameter<double>("angular_kp", 2.0, desc);

    desc.description = "Linear tolerance in meters";
    this->declare_parameter<double>("linear_tolerance", 0.05, desc);

    desc.description = "Angular tolerance in radians";
    this->declare_parameter<double>("angular_tolerance", 0.05, desc);

    // Load parameters
    wheel_radius_ = this->get_parameter("wheel_radius").as_double();
    half_wheel_base_ = this->get_parameter("wheel_base").as_double() / 2.0;
    half_track_width_ = this->get_parameter("track_width").as_double() / 2.0;
    max_linear_vel_ = this->get_parameter("max_linear_vel").as_double();
    max_angular_vel_ = this->get_parameter("max_angular_vel").as_double();
    linear_kp_ = this->get_parameter("linear_kp").as_double();
    angular_kp_ = this->get_parameter("angular_kp").as_double();
    linear_tolerance_ = this->get_parameter("linear_tolerance").as_double();
    angular_tolerance_ = this->get_parameter("angular_tolerance").as_double();

    // Create Reentrant Callback Group
    callback_group_ =
        this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::SubscriptionOptions options;
    options.callback_group = callback_group_;

    // Subscriber for odometry information
    odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odometry/filtered", 10,
        std::bind(&EightTrajectory::odom_callback, this, std::placeholders::_1),
        options);

    // Publisher for wheel speeds
    wheel_speed_publisher_ =
        this->create_publisher<std_msgs::msg::Float32MultiArray>("wheel_speed",
                                                                 10);

    // Timer for control loop
    startup_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000),
        std::bind(&EightTrajectory::start_once, this), callback_group_);

    // Rates for control and logging
    control_rate_ =
        std::make_shared<rclcpp::Rate>(50);        // 50 Hz for control loop
    log_rate_ = std::make_shared<rclcpp::Rate>(2); // 2 Hz for debug logging

    // Precompute inverse kinematics matrix (4x3 for wheels x twist)
    const double L = half_wheel_base_ + half_track_width_;
    inv_kin_mat_ << -L, 1.0, -1.0, L, 1.0, 1.0, L, 1.0, -1.0, -L, 1.0, 1.0;
    inv_kin_mat_ /= wheel_radius_; // Scale by 1/R

    RCLCPP_INFO(this->get_logger(), "Eight Trajectory Node ready!");
  }

private:
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr
      wheel_speed_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::TimerBase::SharedPtr startup_timer_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;

  std::shared_ptr<rclcpp::Rate> control_rate_;
  std::shared_ptr<rclcpp::Rate> log_rate_;

  double current_yaw_{0.0};
  double current_x_{0.0};
  double current_y_{0.0};
  bool odom_received_{false};

  double wheel_radius_;
  double half_wheel_base_;
  double half_track_width_;
  double linear_tolerance_;
  double angular_tolerance_;
  double linear_kp_;
  double angular_kp_;
  double max_angular_vel_;
  double max_linear_vel_;

  Eigen::Matrix<double, 4, 3>
      inv_kin_mat_; // Precomputed inverse kinematics matrix

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_x_ = msg->pose.pose.position.x;
    current_y_ = msg->pose.pose.position.y;
    tf2::Quaternion q(
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    current_yaw_ = tf2::getYaw(q);
    odom_received_ = true;
  }

  void start_once() {
    if (!odom_received_) {
      RCLCPP_WARN(this->get_logger(), "Waiting for first odometry message...");
      return;
    }
    startup_timer_->cancel(); // cancel timer
    execute_trajectory();
  }

  void execute_trajectory() {
    // Wait for publisher connections
    while (rclcpp::ok() &&
           wheel_speed_publisher_->get_subscription_count() == 0) {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "Waiting for subscribers on wheel_speed...");
      log_rate_->sleep(); // 2Hz sleep
    }

    size_t waypoint_index = 0;
    for (const Waypoint &wp : figure_waypoints) {
      RCLCPP_INFO(this->get_logger(),
                  "Moving to waypoint %zu (dx=%.3f, dy=%.3f, dphi=%.3f)",
                  waypoint_index, wp.dx, wp.dy, wp.dphi);
      go_to_waypoint(wp);
      RCLCPP_INFO(this->get_logger(), "Reached waypoint %zu.", waypoint_index);
      waypoint_index++;
    }

    // Final stop
    publish_wheel_speeds({0.0, 0.0, 0.0, 0.0});
    RCLCPP_INFO(this->get_logger(), "Trajectory completed.");
    rclcpp::shutdown();
  }

  void go_to_waypoint(const Waypoint &wp) {
    // Compute target for current waypoint
    double target_x = current_x_ + wp.dx;
    double target_y = current_y_ + wp.dy;
    double target_yaw = normalize_angle(current_yaw_ + wp.dphi);

    // Compute errors
    double error_x = target_x - current_x_;
    double error_y = target_y - current_y_;
    double error_dist = std::hypot(error_x, error_y);
    double error_yaw = normalize_angle(target_yaw - current_yaw_);

    while (rclcpp::ok() && (error_dist > linear_tolerance_ ||
                            std::abs(error_yaw) > angular_tolerance_)) {
      error_x = target_x - current_x_;
      error_y = target_y - current_y_;
      error_dist = std::hypot(error_x, error_y);
      error_yaw = normalize_angle(target_yaw - current_yaw_);

      // Linear velocity: proportional to distance, capped
      double v_lin = std::min(
          max_linear_vel_, linear_kp_ * error_dist); // Slows down as approaches
      double heading_to_target = std::atan2(error_y, error_x);
      double v_world_x = v_lin * std::cos(heading_to_target);
      double v_world_y = v_lin * std::sin(heading_to_target);
      // Angular velocity: proportional to yaw error, capped
      double w_world_z = std::copysign(
          std::min(max_angular_vel_, std::abs(angular_kp_ * error_yaw)),
          error_yaw);

      // Compute Body Frame twist
      double w_z, v_x, v_y;
      std::tie(w_z, v_x, v_y) =
          world_to_body_twist_eigen(w_world_z, v_world_x, v_world_y);
      //    std::tie(w_z, v_x, v_y) = world_to_body_twist(w_world_z, v_world_x,
      //     v_world_y);

      // Compute and publish wheel speed
      //   std::vector<float> wheel_speeds = twist_to_wheels(w_z, v_x, v_y);
      std::vector<float> wheel_speeds = twist_to_wheels_eigen(w_z, v_x, v_y);
      publish_wheel_speeds(wheel_speeds);

      RCLCPP_DEBUG(this->get_logger(),
                   "Debug: Error (x=%.3f, y=%.3f, yaw=%.3f) | "
                   "World Frame Speed: (v_x=%.3f, v_y=%.3f, w_z=%.3f)",
                   error_x, error_y, error_yaw, v_world_x, v_world_y,
                   w_world_z);

      control_rate_->sleep(); // 50 Hz loop
    }

    publish_wheel_speeds({0.0, 0.0, 0.0, 0.0});
    log_rate_->sleep(); // 2Hz sleep -  Brief pause before next
  }

  std::tuple<double, double, double> world_to_body_twist(double dphi, double dx,
                                                         double dy) {
    double w_z = dphi; // Angular velocity is invariant in 2D
    double v_x = std::cos(current_yaw_) * dx + std::sin(current_yaw_) * dy;
    double v_y = -std::sin(current_yaw_) * dx + std::cos(current_yaw_) * dy;

    return {w_z, v_x, v_y};
  }

  std::vector<float> twist_to_wheels(double w_z, double v_x, double v_y) {
    // Lever arm (distance from center to wheel along diagonal)
    const double L = half_wheel_base_ + half_track_width_;
    const double R = wheel_radius_;
    // Inverse kinematics: Body twist to wheel speeds (FL, FR, RL, RR)
    float u1 = (1.0 / R) * (-L * w_z + v_x - v_y);
    float u2 = (1.0 / R) * (L * w_z + v_x + v_y);
    float u3 = (1.0 / R) * (L * w_z + v_x - v_y);
    float u4 = (1.0 / R) * (-L * w_z + v_x + v_y);
    return {u1, u2, u3, u4};
  }

  std::tuple<double, double, double>
  world_to_body_twist_eigen(double dphi, double dx, double dy) {

    // Use Eigen for rotation matrix
    Eigen::Matrix3d rot;
    rot << 1, 0, 0, 0, std::cos(current_yaw_), std::sin(current_yaw_), 0,
        -std::sin(current_yaw_), std::cos(current_yaw_);
    Eigen::Vector3d v_world(dphi, dx, dy);
    Eigen::Vector3d v_body = rot * v_world;

    double w_z = v_body(0);
    double v_x = v_body(1);
    double v_y = v_body(2);

    return {w_z, v_x, v_y};
  }

  std::vector<float> twist_to_wheels_eigen(double w_z, double v_x, double v_y) {
    Eigen::Vector3d twist(w_z, v_x, v_y);
    Eigen::Vector4d u = inv_kin_mat_ * twist;

    return {static_cast<float>(u(0)), static_cast<float>(u(1)),
            static_cast<float>(u(2)), static_cast<float>(u(3))};
  }

  void publish_wheel_speeds(const std::vector<float> &wheel_speeds) {
    std_msgs::msg::Float32MultiArray msg;
    msg.data = wheel_speeds;
    wheel_speed_publisher_->publish(msg);
  }

  double normalize_angle(double theta) {
    theta = std::fmod(theta + M_PI, 2.0 * M_PI) - M_PI;
    if (theta < -M_PI)
      theta += 2 * M_PI;
    return theta;
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<EightTrajectory>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),
                                                    2);
  executor.add_node(node);
  try {
    executor.spin();
  } catch (const std::exception &e) {
    RCLCPP_ERROR(node->get_logger(), "Exception: %s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}