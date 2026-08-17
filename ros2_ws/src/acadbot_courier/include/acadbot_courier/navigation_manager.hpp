#ifndef ACADBOT_COURIER__NAVIGATION_MANAGER_HPP_
#define ACADBOT_COURIER__NAVIGATION_MANAGER_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <cstdint>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "acadbot_courier_interfaces/action/execute_delivery.hpp"
#include "acadbot_courier/types.hpp"

using ExecuteDelivery =
  acadbot_courier_interfaces::action::ExecuteDelivery;

using CourierGoalHandle =
  rclcpp_action::ServerGoalHandle<ExecuteDelivery>;

using NavigateToPose =
  nav2_msgs::action::NavigateToPose;

using NavGoalHandle =
  rclcpp_action::ClientGoalHandle<NavigateToPose>;

class NavigationManager
{
public:
  NavigationManager(
    rclcpp::Node * node,
    const std::string & frame_id,
    int max_retries,
    double feedback_rate_hz,
    double nav2_server_timeout_sec,
    double nav_goal_timeout_sec,
    int nav_result_poll_period_ms,
    const std::unordered_map<std::string, Location> & locations);

  bool action_server_is_ready() const;

  // ---------------------------------------------------------------------------
  // Execute one courier navigation leg with configurable retry handling.
  //
  // A leg is attempted once initially, then retried up to max_retries_ times if
  // Nav2 aborts or the courier navigation timeout is reached.
  //
  // Cancellation is never retried because it represents an explicit request to
  // stop the delivery.
  //
  // The current attempt number is passed to navigate_to_location() so action
  // feedback accurately reports progress such as attempt 1 of 2.
  // ---------------------------------------------------------------------------
  NavigationOutcome navigate_with_retries(
    const std::string & leg,
    const std::string & target,
    const std::shared_ptr<CourierGoalHandle> & goal_handle);

private:
  // ---------------------------------------------------------------------------
  // Convert a configured courier location into a Nav2 goal pose.
  //
  // Courier locations are stored in YAML as [x, y, yaw]. Nav2 expects a
  // geometry_msgs::msg::PoseStamped, so this helper adds the configured map
  // frame and current timestamp and converts the planar yaw angle into a
  // quaternion orientation.
  // ---------------------------------------------------------------------------
  geometry_msgs::msg::PoseStamped make_pose(const Location & location);

  // ---------------------------------------------------------------------------
  // Publish human-readable progress for the active courier action.
  //
  // Feedback identifies the current delivery leg, named target, latest remaining
  // distance reported by Nav2, and the current navigation attempt. The maximum
  // number of attempts comes from the configured retry limit.
  // ---------------------------------------------------------------------------
  void publish_delivery_feedback(
    const std::shared_ptr<CourierGoalHandle> & goal_handle,
    const std::string & leg,
    const std::string & target,
    double distance_remaining,
    uint32_t attempt);

  // ---------------------------------------------------------------------------
  // Navigate to one configured named location using Nav2.
  //
  // The location name is resolved through the YAML-loaded location map and
  // converted into a PoseStamped before being sent to Nav2's navigate_to_pose
  // action server.
  //
  // This helper waits for Nav2's result from the delivery worker thread while
  // the ROS executor remains free to process action callbacks. It returns a
  // NavigationOutcome so the courier mission can distinguish success, abort,
  // cancellation, and goal rejection.
  // ---------------------------------------------------------------------------
  NavigationOutcome navigate_to_location(
    const std::string & leg,
    const std::string & location_name,
    uint32_t attempt,
    const std::shared_ptr<CourierGoalHandle> & courier_goal_handle);

  rclcpp::Node * node_;

  std::string frame_id_;

  int max_retries_;
  int nav_result_poll_period_ms_;

  double feedback_rate_hz_;
  double nav2_server_timeout_sec_;
  double nav_goal_timeout_sec_;
  double latest_distance_remaining_{0.0};

  bool nav_feedback_received_{false};

  const std::unordered_map<std::string, Location> & locations_;

  rclcpp_action::Client<NavigateToPose>::SharedPtr nav2_client_;

  NavGoalHandle::SharedPtr active_nav_goal_;

  mutable std::mutex state_mutex_;
};

#endif  // ACADBOT_COURIER__NAVIGATION_MANAGER_HPP_
