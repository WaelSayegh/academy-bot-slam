// navigation_manager.cpp
// ---------------------------------------------------------------------------
// Final Project — AcadBot Courier
//
// Focused Nav2 execution support for the AcadBot courier system.
//
// This class owns the NavigateToPose action client and the low-level behavior
// required to drive one navigation leg while preserving the current courier
// semantics for retries, cancellation, timeout handling, and action feedback.
// ---------------------------------------------------------------------------

#include "acadbot_courier/navigation_manager.hpp"

#include <chrono>
#include <future>

#include "tf2/LinearMath/Quaternion.h"

NavigationManager::NavigationManager(
  rclcpp::Node * node,
  const std::string & frame_id,
  int max_retries,
  double feedback_rate_hz,
  double nav2_server_timeout_sec,
  double nav_goal_timeout_sec,
  int nav_result_poll_period_ms,
  const std::unordered_map<std::string, Location> & locations)
: node_(node),
  frame_id_(frame_id),
  max_retries_(max_retries),
  nav_result_poll_period_ms_(nav_result_poll_period_ms),
  feedback_rate_hz_(feedback_rate_hz),
  nav2_server_timeout_sec_(nav2_server_timeout_sec),
  nav_goal_timeout_sec_(nav_goal_timeout_sec),
  locations_(locations)
{
  nav2_client_ =
    rclcpp_action::create_client<NavigateToPose>(
    node_, "navigate_to_pose");
}

bool NavigationManager::action_server_is_ready() const
{
  return nav2_client_->action_server_is_ready();
}

NavigationOutcome NavigationManager::navigate_with_retries(
  const std::string & leg,
  const std::string & target,
  const std::shared_ptr<CourierGoalHandle> & goal_handle)
{
  const uint32_t max_attempts =
    static_cast<uint32_t>(max_retries_ + 1);

  NavigationOutcome last_outcome = NavigationOutcome::ABORTED;

  for (uint32_t attempt = 1; attempt <= max_attempts; ++attempt) {

    RCLCPP_INFO(node_->get_logger(),
      "Starting %s navigation attempt %u of %u to '%s'.",
      leg.c_str(),
      attempt,
      max_attempts,
      target.c_str());

    last_outcome =
      navigate_to_location(
        leg,
        target,
        attempt,
        goal_handle);

    if (last_outcome == NavigationOutcome::SUCCEEDED) {
      return NavigationOutcome::SUCCEEDED;
    }

    if (last_outcome == NavigationOutcome::CANCELED) {
      return NavigationOutcome::CANCELED;
    }

    if (attempt < max_attempts) {
      RCLCPP_WARN(node_->get_logger(),
        "%s navigation to '%s' failed on attempt %u of %u. Retrying.",
        leg.c_str(),
        target.c_str(),
        attempt,
        max_attempts);
    } 
    else {
      RCLCPP_ERROR(node_->get_logger(),
        "%s navigation to '%s' failed after %u attempt(s).",
        leg.c_str(),
        target.c_str(),
        max_attempts);
    }
  }

  return last_outcome;
}

geometry_msgs::msg::PoseStamped NavigationManager::make_pose(
  const Location & location)
{
  geometry_msgs::msg::PoseStamped pose;

  pose.header.frame_id = frame_id_;
  pose.header.stamp = node_->now();

  pose.pose.position.x = location.x;
  pose.pose.position.y = location.y;
  pose.pose.position.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, location.yaw);

  pose.pose.orientation.x = q.x();
  pose.pose.orientation.y = q.y();
  pose.pose.orientation.z = q.z();
  pose.pose.orientation.w = q.w();

  return pose;
}

void NavigationManager::publish_delivery_feedback(
  const std::shared_ptr<CourierGoalHandle> & goal_handle,
  const std::string & leg,
  const std::string & target,
  double distance_remaining,
  uint32_t attempt)
{
  auto feedback =
    std::make_shared<ExecuteDelivery::Feedback>();

  feedback->leg = leg;
  feedback->target = target;
  feedback->distance_remaining =
    static_cast<float>(distance_remaining);
  feedback->attempt = attempt;
  feedback->max_attempts =
    static_cast<uint32_t>(max_retries_ + 1);

  goal_handle->publish_feedback(feedback);
}

NavigationOutcome NavigationManager::navigate_to_location(
  const std::string & leg,
  const std::string & location_name,
  uint32_t attempt,
  const std::shared_ptr<CourierGoalHandle> & courier_goal_handle)
{
  const auto location_it = locations_.find(location_name);

  // Verify that the requested location exists in the configured map.
  if (location_it == locations_.end()) {
    RCLCPP_ERROR(node_->get_logger(),
      "Cannot navigate to unknown location '%s'.",
      location_name.c_str());

    return NavigationOutcome::REJECTED;
  }

  // Reset the latest distance remaining and feedback flag before sending a new goal.
  {
    std::lock_guard<std::mutex> lock(state_mutex_);

    latest_distance_remaining_ = 0.0;
    nav_feedback_received_ = false;
  }

  NavigateToPose::Goal nav_goal;
  nav_goal.pose = make_pose(location_it->second);

  RCLCPP_INFO(node_->get_logger(),
    "Sending Nav2 goal to '%s' at (%.4f, %.4f, yaw=%.4f).",
    location_name.c_str(),
    location_it->second.x,
    location_it->second.y,
    location_it->second.yaw);

  rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;

  options.feedback_callback =
    [this, location_name](NavGoalHandle::SharedPtr,
      const std::shared_ptr<const NavigateToPose::Feedback> feedback)
      {
        {
          std::lock_guard<std::mutex> lock(state_mutex_);

          latest_distance_remaining_ = feedback->distance_remaining;

          nav_feedback_received_ = true;
        }

        RCLCPP_DEBUG(node_->get_logger(),
          "Nav2 feedback for '%s': %.2f m remaining",
          location_name.c_str(),
          feedback->distance_remaining);
      };

  auto goal_future =
    nav2_client_->async_send_goal(nav_goal, options);

  // Wait for Nav2 to accept or reject the goal.
  if (goal_future.wait_for(std::chrono::seconds(
    static_cast<int>(nav2_server_timeout_sec_))) !=
      std::future_status::ready)
  {
    RCLCPP_ERROR(node_->get_logger(),
      "Timed out waiting for Nav2 to accept goal '%s'.",
      location_name.c_str());

    return NavigationOutcome::REJECTED;
  }

  auto nav_goal_handle = goal_future.get();

  if (!nav_goal_handle) {
    RCLCPP_ERROR(node_->get_logger(),
      "Nav2 rejected goal for '%s'.",
      location_name.c_str());

    return NavigationOutcome::REJECTED;
  }

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    active_nav_goal_ = nav_goal_handle;
  }

  RCLCPP_INFO(node_->get_logger(),
    "Nav2 accepted goal for '%s'.",
    location_name.c_str());

  auto result_future =
    nav2_client_->async_get_result(nav_goal_handle);

  const auto start_time = node_->now();

  auto last_feedback_publish = std::chrono::steady_clock::now();

  const double feedback_period_sec = 1.0 / feedback_rate_hz_;

  bool nav_cancel_requested = false;
  bool navigation_timed_out = false;

  while (result_future.wait_for(
    std::chrono::milliseconds(nav_result_poll_period_ms_))
      != std::future_status::ready) 
  {
    // ---------------------------------------------------------
    // Courier action cancellation
    // ---------------------------------------------------------

    if (courier_goal_handle->is_canceling() &&
      !nav_cancel_requested)
    {
      RCLCPP_WARN(node_->get_logger(),
        "Courier action canceled while navigating to '%s'. "
        "Canceling active Nav2 goal.",
        location_name.c_str());

      nav2_client_->async_cancel_goal(nav_goal_handle);

      nav_cancel_requested = true;
    }


    // ---------------------------------------------------------
    // Courier action feedback
    // ---------------------------------------------------------

    const auto wall_now = std::chrono::steady_clock::now();

    const double time_since_feedback =
      std::chrono::duration<double>(
        wall_now - last_feedback_publish).count();

    if (time_since_feedback >= feedback_period_sec) {

      double distance = 0.0;
      bool have_distance = false;

      {
        std::lock_guard<std::mutex> lock(state_mutex_);

        distance = latest_distance_remaining_;
        have_distance = nav_feedback_received_;
      }

      if (have_distance && !courier_goal_handle->is_canceling()) {
        publish_delivery_feedback(
          courier_goal_handle,
          leg,
          location_name,
          distance,
          attempt);
      }

      last_feedback_publish = wall_now;
    }


    // ---------------------------------------------------------
    // Courier navigation timeout
    // ---------------------------------------------------------

    const double elapsed = (node_->now() - start_time).seconds();

    if (elapsed >= nav_goal_timeout_sec_ && !nav_cancel_requested) 
    {
      RCLCPP_ERROR(node_->get_logger(),
        "Navigation to '%s' exceeded timeout of %.1f seconds.",
        location_name.c_str(),
        nav_goal_timeout_sec_);

      nav2_client_->async_cancel_goal(nav_goal_handle);

      nav_cancel_requested = true;

      // Remember that this was caused by timeout rather than user cancellation.
      navigation_timed_out = true;
    }
  }


  // ---------------------------------------------------------
  // Process the final Nav2 result
  // ---------------------------------------------------------

  const auto result = result_future.get();

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    active_nav_goal_.reset();
  }

  if (navigation_timed_out) {
    RCLCPP_ERROR(node_->get_logger(),
      "Navigation to '%s' stopped after courier timeout.",
      location_name.c_str());

    return NavigationOutcome::TIMED_OUT;
  }

  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(node_->get_logger(),
        "Reached location '%s'.",
        location_name.c_str());

      return NavigationOutcome::SUCCEEDED;

    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(node_->get_logger(),
        "Nav2 aborted navigation to '%s'.",
        location_name.c_str());

      return NavigationOutcome::ABORTED;

    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_WARN(node_->get_logger(),
        "Navigation to '%s' was canceled.",
        location_name.c_str());

      return NavigationOutcome::CANCELED;

    default:
      RCLCPP_ERROR(node_->get_logger(),
        "Navigation to '%s' ended with an unknown result.",
        location_name.c_str());

      return NavigationOutcome::ABORTED;
  }
}
