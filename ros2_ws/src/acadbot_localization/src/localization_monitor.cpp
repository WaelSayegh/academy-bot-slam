// localization_monitor.cpp
// ---------------------------------------------------------------------------
// Session 2 — "is AMCL actually localised, or is it just running?"
//
// RViz answers that only by eye. This node answers it with a number: it
// subscribes to /amcl_pose and, on a timer, logs the robot's x, y and yaw
// alongside AMCL's own estimate of how uncertain that position is.
//
// The uncertainty comes out of the message's row-major 6x6 covariance, where
// index 0 is the variance in x and index 7 the variance in y. We report the
// standard deviation — the square root of the larger of the two — in metres,
// and label it CONVERGED or SEARCHING against the converged_sigma threshold.
//
// Parameters (config/localization_monitor.yaml):
//   report_period    seconds between report lines           (default 1.0)
//   converged_sigma  metres; below it, report CONVERGED      (default 0.25)
//
// Before an initial pose is set in RViz, AMCL publishes nothing at all, so
// there is no pose to report. The node warns about that rather than printing
// zeros, which would look like a confident estimate at the origin.
// ---------------------------------------------------------------------------
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

class LocalizationMonitor : public rclcpp::Node
{
public:
  LocalizationMonitor() : Node("localization_monitor"), has_received_pose_(false)
  {
    this->declare_parameter<double>("report_period", 1.0);
    this->declare_parameter<double>("converged_sigma", 0.25);

    double report_period = this->get_parameter("report_period").as_double();
    converged_sigma_ = this->get_parameter("converged_sigma").as_double();

    pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/amcl_pose", 10,
      std::bind(&LocalizationMonitor::pose_callback, this, std::placeholders::_1));

    auto timer_period = std::chrono::duration<double>(report_period);
    timer_ = this->create_wall_timer(
      timer_period, std::bind(&LocalizationMonitor::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Localization Monitor node started.");
  }
private:
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_subscriber_;
  rclcpp::TimerBase::SharedPtr timer_;
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr last_pose_;
  bool has_received_pose_;
  double converged_sigma_;

  void pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    last_pose_ = msg;
    has_received_pose_ = true;
  }

  void timer_callback()
  {
    if (!has_received_pose_)
    {
      RCLCPP_WARN(this->get_logger(), "No AMCL pose received yet.");
      return;
    }

    double x = last_pose_->pose.pose.position.x;
    double y = last_pose_->pose.pose.position.y;

    tf2::Quaternion q(
      last_pose_->pose.pose.orientation.x,
      last_pose_->pose.pose.orientation.y,
      last_pose_->pose.pose.orientation.z,
      last_pose_->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    // Row-major 6x6 covariance matrix: Index 0 is var(x), Index 7 is var(y)
    double var_x = last_pose_->pose.covariance[0];
    double var_y = last_pose_->pose.covariance[7];
    double max_var = std::max(var_x, var_y);
    double sigma = (max_var > 0.0) ? std::sqrt(max_var) : 0.0;

    // Determine convergence status
    std::string status = (sigma <= converged_sigma_) ? "CONVERGED" : "SEARCHING";

    RCLCPP_INFO(this->get_logger(), "x: %.3f, y: %.3f, yaw: %.3f, uncertainty: %.3f m (%s)",
                x, y, yaw, sigma, status.c_str());
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalizationMonitor>());
  rclcpp::shutdown();
  return 0;
}
