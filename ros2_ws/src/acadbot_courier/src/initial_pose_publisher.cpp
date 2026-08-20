#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"

namespace acadbot_courier
{

class InitialPosePublisher : public rclcpp::Node
{
public:
  explicit InitialPosePublisher(const rclcpp::NodeOptions & options)
  : Node("initial_pose_publisher", options)
  {
    x_ = required_parameter<double>("initial_pose.x");
    y_ = required_parameter<double>("initial_pose.y");
    yaw_ = required_parameter<double>("initial_pose.yaw");
    frame_ = required_parameter<std::string>("global_frame");
    covariance_x_ = required_parameter<double>("initial_pose.covariance_x");
    covariance_y_ = required_parameter<double>("initial_pose.covariance_y");
    covariance_yaw_ = required_parameter<double>("initial_pose.covariance_yaw");
    const double delay_sec = required_parameter<double>("delay_sec");
    if (delay_sec < 0.0) {
      throw std::runtime_error("delay_sec must be non-negative");
    }

    publisher_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose", rclcpp::QoS(1).reliable().transient_local());
    timer_ = create_wall_timer(
      std::chrono::duration<double>(delay_sec),
      std::bind(&InitialPosePublisher::publish_once, this));
  }

private:
  template<typename T>
  T required_parameter(const std::string & name) const
  {
    T value{};
    if (!get_parameter(name, value)) {
      throw std::runtime_error("required parameter is missing: " + name);
    }
    return value;
  }

  void publish_once()
  {
    geometry_msgs::msg::PoseWithCovarianceStamped message;
    message.header.stamp = now();
    message.header.frame_id = frame_;
    message.pose.pose.position.x = x_;
    message.pose.pose.position.y = y_;

    tf2::Quaternion orientation;
    orientation.setRPY(0.0, 0.0, yaw_);
    message.pose.pose.orientation.x = orientation.x();
    message.pose.pose.orientation.y = orientation.y();
    message.pose.pose.orientation.z = orientation.z();
    message.pose.pose.orientation.w = orientation.w();
    message.pose.covariance[0] = covariance_x_;
    message.pose.covariance[7] = covariance_y_;
    message.pose.covariance[35] = covariance_yaw_;

    publisher_->publish(message);
    RCLCPP_INFO(
      get_logger(), "Published initial pose (%.3f, %.3f, yaw=%.3f) in %s",
      x_, y_, yaw_, frame_.c_str());
    timer_->cancel();
  }

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  double x_;
  double y_;
  double yaw_;
  double covariance_x_;
  double covariance_y_;
  double covariance_yaw_;
  std::string frame_;
};

}  // namespace acadbot_courier

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);
    rclcpp::spin(std::make_shared<acadbot_courier::InitialPosePublisher>(options));
  } catch (const std::exception & error) {
    RCLCPP_FATAL(
      rclcpp::get_logger("initial_pose_publisher"), "Startup failed: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
