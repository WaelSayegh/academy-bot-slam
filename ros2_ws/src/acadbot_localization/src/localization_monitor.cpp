#include <chrono>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

using geometry_msgs::msg::PoseWithCovarianceStamped;
using namespace std::chrono_literals;

class LocalizationMonitor : public rclcpp::Node
{
public:
  LocalizationMonitor() : Node("localization_monitor")
  {
    // Read the report period from the params yaml (falls back to 1.0s if
    // no params file is passed in).
    declare_parameter("report_period", 1.0);
    double report_period = get_parameter("report_period").as_double();

    // Creating a subscriber: listens to AMCL's pose estimate and just
    // stashes the latest message, no logging happens here.
    subscription_ = create_subscription<PoseWithCovarianceStamped>(
      "/amcl_pose", 10,
      [this](PoseWithCovarianceStamped::SharedPtr msg) {
        latest_pose_ = msg;
        has_pose_ = true;
      });

    // Creating a timer: fires every report_period seconds and calls
    // report(), which does the actual logging.
    timer_ = create_wall_timer(
      std::chrono::duration<double>(report_period),
      std::bind(&LocalizationMonitor::report, this));
  }

private:
  void report()
  {
    // Guard clause: AMCL publishes nothing at all until an initial pose is
    // set in RViz, so has_pose_ is the only way to tell "no message yet"
    // apart from "a real pose that happens to be zero."
    if (!has_pose_) {
      RCLCPP_WARN(get_logger(),
        "No AMCL pose yet - set an initial pose with 2D Pose Estimate in RViz");
      return;
    }

    // Pulling x, y straight out of the message.
    double x = latest_pose_->pose.pose.position.x;
    double y = latest_pose_->pose.pose.position.y;

    // Orientation arrives as a quaternion (qx, qy, qz, qw), not an angle.
    // The general formula for the yaw it encodes is
    //   yaw = atan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz))
    // which is exactly what tf2::getYaw() computes, and it is correct for
    // any orientation, including one that is also rolled or pitched.
    //
    // AcadBot is a ground differential-drive robot: it only ever rotates
    // about the vertical (z) axis, so its quaternion always has qx = 0 and
    // qy = 0. Substituting that into the general formula:
    //   yaw = atan2(2*qw*qz, 1 - 2*qz*qz)
    //
    // A quaternion for "rotate by angle phi about z" is built as
    // qz = sin(phi/2), qw = cos(phi/2) - it always stores HALF the angle.
    // With qx = qy = 0, those are the only two nonzero components, so
    //   atan2(qz, qw) = atan2(sin(phi/2), cos(phi/2)) = phi/2,
    // and doubling that back out gives the real yaw:
    //   yaw = 2 * atan2(qz, qw)
    // (this is the same value as the atan2(2*qw*qz, 1 - 2*qz*qz) form
    // above, just reached through a simpler identity). It is an exact
    // simplification for a robot that never tilts, not an approximation,
    // and it means the node needs no tf2/tf2_geometry_msgs dependency for
    // one angle.
    double qz = latest_pose_->pose.pose.orientation.z;
    double qw = latest_pose_->pose.pose.orientation.w;
    double yaw = 2.0 * std::atan2(qz, qw);

    // Logging the report line.
    RCLCPP_INFO(get_logger(), "x=%.3f y=%.3f yaw=%.3f", x, y, yaw);
  }

  // Subscriber handle, the timer handle, and the last pose we received.
  rclcpp::Subscription<PoseWithCovarianceStamped>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  PoseWithCovarianceStamped::SharedPtr latest_pose_;
  bool has_pose_ = false;
};

int main(int argc, char ** argv)
{
  
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalizationMonitor>());
  rclcpp::shutdown();
  return 0;
}
