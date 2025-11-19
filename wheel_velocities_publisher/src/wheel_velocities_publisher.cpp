#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <string>

class WheelVelocitiesPublisher : public rclcpp::Node {
public:
  WheelVelocitiesPublisher() : Node("wheel_velocities_publisher") {
    // Publishers
    wheel_speed_publisher_ =
        this->create_publisher<std_msgs::msg::Float32MultiArray>("wheel_speed",
                                                                 10);
    RCLCPP_INFO(this->get_logger(), "Wheel Velocities Publisher Node ready!");
  }

  void execute_commands() {
    const float speed = 4.0f;
    const double delay_sec = 3.0;
    rclcpp::Rate rate(1.0 / delay_sec); // Frequency = 1 / delay sec

    // front left and rare right (1,-1) vector | wheel 1 and wheel 3
    // front right and rare left (1, 1) vector | wheel 2 and wheel 4

    // Forward
    publish_command({speed, speed, speed, speed}, "Move Forward");
    rate.sleep();

    // Backward
    publish_command({-speed, -speed, -speed, -speed}, "Move Backward");
    rate.sleep();

    // Left
    publish_command({-speed, speed, -speed, speed}, "Move Left");
    rate.sleep();

    // Right
    publish_command({speed, -speed, speed, -speed}, "Move Right");
    rate.sleep();

    // Turn Clockwise
    publish_command({speed, -speed, -speed, speed}, "Turn Clockwise");
    rate.sleep();

    // Turn Counter Clockwise
    publish_command({-speed, speed, speed, -speed}, "Turn Counter-Clockwise");
    rate.sleep();

    // Stop
    publish_command({0.0f, 0.0f, 0.0f, 0.0f}, "Stop");
  }

private:
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr
      wheel_speed_publisher_;

  void publish_command(const std::vector<float> &wheel_speeds,
                       const std::string &message = "") {
    if (wheel_speeds.size() != 4) {
      RCLCPP_ERROR(this->get_logger(),
                   "Invalid wheel speeds: Must provide exactly 4 values.");
      return;
    }

    if (!message.empty()) {
      RCLCPP_INFO(this->get_logger(), "%s", message.c_str());
    }

    std_msgs::msg::Float32MultiArray msg;
    msg.data = wheel_speeds;
    wheel_speed_publisher_->publish(msg);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto wheel_velocities_node = std::make_shared<WheelVelocitiesPublisher>();
  wheel_velocities_node->execute_commands();
  rclcpp::shutdown();
  return 0;
}