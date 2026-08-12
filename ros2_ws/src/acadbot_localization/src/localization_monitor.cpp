#include <chrono>
#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

using namespace std::chrono_literals;

class LocalizationMonitor : public rclcpp::Node
{
public:
  LocalizationMonitor()
  : Node("localization_monitor")
  {
    this->declare_parameter<double>("report_period", 1.0);
    this->declare_parameter<double>("converged_sigma", 0.25);

    report_period_ = this->get_parameter("report_period").as_double();
    converged_sigma_ = this->get_parameter("converged_sigma").as_double();

    subscription_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/amcl_pose", 10,
      std::bind(&LocalizationMonitor::pose_callback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(report_period_),
      std::bind(&LocalizationMonitor::report, this));

    RCLCPP_INFO(this->get_logger(),
      "localization_monitor started. Waiting for /amcl_pose (report every %.2fs, converged below sigma=%.2fm)...",
      report_period_, converged_sigma_);
  }

private:
  void pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    latest_pose_ = msg;
    have_pose_ = true;
  }

  void report()
  {
    if (!have_pose_) {
      RCLCPP_WARN(this->get_logger(),
        "No /amcl_pose received yet — AMCL has not been given an initial pose. "
        "Use '2D Pose Estimate' in RViz.");
      return;
    }

    const auto & pose = latest_pose_->pose.pose;
    const double x = pose.position.x;
    const double y = pose.position.y;

    const double siny_cosp = 2.0 * (pose.orientation.w * pose.orientation.z +
                                     pose.orientation.x * pose.orientation.y);
    const double cosy_cosp = 1.0 - 2.0 * (pose.orientation.y * pose.orientation.y +
                                           pose.orientation.z * pose.orientation.z);
    const double yaw = std::atan2(siny_cosp, cosy_cosp);

    const auto & cov = latest_pose_->pose.covariance;
    const double var_x = cov[0];
    const double var_y = cov[7];
    const double sigma = std::sqrt(std::max(var_x, var_y));

    const std::string verdict = (sigma < converged_sigma_) ? "CONVERGED" : "SEARCHING";

    RCLCPP_INFO(this->get_logger(),
      "x=%.3f y=%.3f yaw=%.3f rad | sigma=%.3f m | %s",
      x, y, yaw, sigma, verdict.c_str());
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr latest_pose_;
  bool have_pose_{false};
  double report_period_{1.0};
  double converged_sigma_{0.25};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalizationMonitor>());
  rclcpp::shutdown();
  return 0;
}
