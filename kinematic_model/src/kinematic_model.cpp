#include <chrono>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <vector>

class KinematicModel : public rclcpp::Node {
public:
  KinematicModel() : Node("kinematic_model") {
    // Declare parameters with defaults and descriptions
    rcl_interfaces::msg::ParameterDescriptor radius_desc;
    radius_desc.description = "Wheel radius in meters";
    this->declare_parameter<double>("wheel_radius", 0.05, radius_desc);

    rcl_interfaces::msg::ParameterDescriptor base_desc;
    base_desc.description = "Full wheel base (longitudinal distance between "
                            "front/rear axles) in meters";
    this->declare_parameter<double>("wheel_base", 0.17, base_desc);

    rcl_interfaces::msg::ParameterDescriptor track_desc;
    track_desc.description = "Full track width (lateral distance between "
                             "left/right wheels) in meters";
    this->declare_parameter<double>("track_width", 0.26969, track_desc);

    // Load parameters
    wheel_radius_ = this->get_parameter("wheel_radius").as_double();
    half_wheel_base_ = this->get_parameter("wheel_base").as_double() / 2.0;
    half_track_width_ = this->get_parameter("track_width").as_double() / 2.0;

    // Subscriber for wheel speeds
    wheel_speed_subscription_ =
        this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "wheel_speed", 10,
            std::bind(&KinematicModel::wheel_speed_callback, this,
                      std::placeholders::_1));
    // Publisher for computed twist
    cmd_vel_publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    RCLCPP_INFO(this->get_logger(), "Kinematic Model Node ready!");
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr
      wheel_speed_subscription_;

  double wheel_radius_;
  double half_wheel_base_;
  double half_track_width_;

  void
  wheel_speed_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    const std::vector<float> &speeds = msg->data;

    if (speeds.size() != 4) {
      RCLCPP_ERROR(this->get_logger(),
                   "Invalid wheel speeds: Must provide exactly 4 values.");
      return;
    }

    // for simplicity - lever arm (distance from center to wheel along diagonal)
    const double L = half_wheel_base_ + half_track_width_;
    const double R = wheel_radius_;

    // Forward Kinematics for Holonomic Robot:
    // Wheel speeds to body twist (assuming ordering: FL[0], FR[1], RL[2],
    // RR[3])
    double v_x = (R / 4) * (speeds[0] + speeds[1] + speeds[2] + speeds[3]);
    double v_y = (R / 4) * (-speeds[0] + speeds[1] - speeds[2] + speeds[3]);
    double w_z =
        (R / (4 * L)) * (-speeds[0] + speeds[1] + speeds[2] - speeds[3]);

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = v_x;
    cmd.linear.y = v_y;
    cmd.angular.z = w_z;

    cmd_vel_publisher_->publish(cmd);

    RCLCPP_DEBUG(
        this->get_logger(),
        "Published chassis planar twist (v_x, v_y, w_z) = (%.3f, %.3f, %.3f)",
        v_x, v_y, w_z);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<KinematicModel>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}