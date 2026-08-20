#include "acadbot_courier/nav_leg_client.hpp"

#include <chrono>
#include <cmath>
#include <thread>
#include <utility>

#include "tf2/exceptions.hpp"
#include "tf2/time.hpp"

namespace acadbot_courier
{

using namespace std::chrono_literals;

NavLegClient::NavLegClient(
  rclcpp::Node * node,
  const rclcpp::CallbackGroup::SharedPtr & callback_group,
  std::string action_name,
  std::string global_frame,
  std::string robot_base_frame,
  std::string global_costmap_clear_service,
  std::string local_costmap_clear_service,
  double goal_response_timeout_sec,
  double cancel_confirm_timeout_sec,
  double costmap_service_timeout_sec)
: node_(node),
  logger_(node->get_logger()),
  callback_group_(callback_group),
  global_frame_(std::move(global_frame)),
  robot_base_frame_(std::move(robot_base_frame)),
  global_costmap_clear_service_(std::move(global_costmap_clear_service)),
  local_costmap_clear_service_(std::move(local_costmap_clear_service)),
  goal_response_timeout_sec_(goal_response_timeout_sec),
  cancel_confirm_timeout_sec_(cancel_confirm_timeout_sec),
  costmap_service_timeout_sec_(costmap_service_timeout_sec)
{
  action_client_ = rclcpp_action::create_client<NavigateToPose>(
    node_, action_name, callback_group_);
  global_costmap_client_ = node_->create_client<ClearEntireCostmap>(
    global_costmap_clear_service_, rclcpp::ServicesQoS(), callback_group_);
  local_costmap_client_ = node_->create_client<ClearEntireCostmap>(
    local_costmap_clear_service_, rclcpp::ServicesQoS(), callback_group_);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(
    *tf_buffer_,
    node_->get_node_base_interface(),
    node_->get_node_logging_interface(),
    node_->get_node_parameters_interface(),
    node_->get_node_topics_interface(),
    false);
}

bool NavLegClient::send(const geometry_msgs::msg::PoseStamped & goal)
{
  if (!action_client_->wait_for_action_server(
      std::chrono::duration<double>(goal_response_timeout_sec_)))
  {
    RCLCPP_WARN(logger_, "Nav2 action server is not available");
    return false;
  }

  NavigateToPose::Goal nav_goal;
  nav_goal.pose = goal;
  nav_goal.pose.header.stamp = node_->now();

  {
    std::lock_guard<std::mutex> lock(mutex_);
    goal_handle_.reset();
    result_future_.reset();
  }
  set_target(nav_goal.pose);

  rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
  options.feedback_callback =
    [this](GoalHandle::SharedPtr, const std::shared_ptr<const NavigateToPose::Feedback> feedback) {
      std::lock_guard<std::mutex> lock(mutex_);
      nav_distance_remaining_ = static_cast<double>(feedback->distance_remaining);
    };

  const auto goal_future = action_client_->async_send_goal(nav_goal, options);
  if (goal_future.wait_for(std::chrono::duration<double>(goal_response_timeout_sec_)) !=
    std::future_status::ready)
  {
    RCLCPP_WARN(logger_, "Timed out waiting for Nav2 to accept the goal");
    return false;
  }

  const auto goal_handle = goal_future.get();
  if (!goal_handle) {
    return false;
  }

  const auto result_future = action_client_->async_get_result(goal_handle);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    goal_handle_ = goal_handle;
    result_future_ = result_future;
  }
  return true;
}

void NavLegClient::set_target(const geometry_msgs::msg::PoseStamped & goal)
{
  std::lock_guard<std::mutex> lock(mutex_);
  target_pose_ = goal;
  nav_distance_remaining_.reset();
}

double NavLegClient::distance_remaining() const
{
  geometry_msgs::msg::PoseStamped target;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (nav_distance_remaining_.has_value() && nav_distance_remaining_.value() > 0.0) {
      return nav_distance_remaining_.value();
    }
    target = target_pose_;
  }

  try {
    const auto transform = tf_buffer_->lookupTransform(
      global_frame_, robot_base_frame_, tf2::TimePointZero);
    const double dx = target.pose.position.x - transform.transform.translation.x;
    const double dy = target.pose.position.y - transform.transform.translation.y;
    return std::hypot(dx, dy);
  } catch (const tf2::TransformException & error) {
    RCLCPP_DEBUG(logger_, "Distance fallback unavailable: %s", error.what());
    return -1.0;
  }
}

NavResult NavLegClient::wait_for_result(
  double timeout_sec, const std::function<bool()> & cancel_requested)
{
  std::shared_future<GoalHandle::WrappedResult> result_future;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!result_future_.has_value()) {
      return NavResult::ABORTED;
    }
    result_future = result_future_.value();
  }

  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::duration<double>(timeout_sec);
  while (rclcpp::ok()) {
    if (cancel_requested()) {
      return cancel() ? NavResult::CANCELED : NavResult::TIMEOUT;
    }
    if (std::chrono::steady_clock::now() >= deadline) {
      cancel();
      return NavResult::TIMEOUT;
    }
    if (result_future.wait_for(100ms) == std::future_status::ready) {
      const auto wrapped_result = result_future.get();
      {
        std::lock_guard<std::mutex> lock(mutex_);
        goal_handle_.reset();
        result_future_.reset();
      }
      switch (wrapped_result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          return NavResult::SUCCEEDED;
        case rclcpp_action::ResultCode::CANCELED:
          return NavResult::CANCELED;
        case rclcpp_action::ResultCode::ABORTED:
        default:
          return NavResult::ABORTED;
      }
    }
  }
  cancel();
  return NavResult::CANCELED;
}

bool NavLegClient::cancel()
{
  GoalHandle::SharedPtr goal_handle;
  std::optional<std::shared_future<GoalHandle::WrappedResult>> result_future;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    goal_handle = goal_handle_;
    result_future = result_future_;
  }
  if (!goal_handle || !result_future.has_value()) {
    return true;
  }
  if (result_future->wait_for(0ms) == std::future_status::ready) {
    std::lock_guard<std::mutex> lock(mutex_);
    goal_handle_.reset();
    result_future_.reset();
    return true;
  }

  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::duration<double>(cancel_confirm_timeout_sec_);
  const auto cancel_future = action_client_->async_cancel_goal(goal_handle);
  if (cancel_future.wait_until(deadline) != std::future_status::ready) {
    RCLCPP_WARN(logger_, "Timed out waiting for Nav2 to acknowledge cancellation");
    return false;
  }
  const auto cancel_response = cancel_future.get();
  if (cancel_response->goals_canceling.empty()) {
    RCLCPP_WARN(logger_, "Nav2 did not accept the cancellation request");
    return false;
  }

  // A cancel-service reply is only an acknowledgement. The robot-facing goal is
  // not finished until its result reaches a terminal state.
  if (result_future->wait_until(deadline) != std::future_status::ready) {
    RCLCPP_WARN(logger_, "Timed out waiting for the Nav2 goal to terminate after cancellation");
    return false;
  }

  const auto wrapped_result = result_future->get();
  if (wrapped_result.code != rclcpp_action::ResultCode::CANCELED) {
    RCLCPP_WARN(
      logger_, "Nav2 cancellation ended with result code %d",
      static_cast<int>(wrapped_result.code));
  }
  std::lock_guard<std::mutex> lock(mutex_);
  goal_handle_.reset();
  result_future_.reset();
  return true;
}

void NavLegClient::clear_costmaps()
{
  clear_costmap(global_costmap_client_, global_costmap_clear_service_);
  clear_costmap(local_costmap_client_, local_costmap_clear_service_);
}

void NavLegClient::clear_costmap(
  const rclcpp::Client<ClearEntireCostmap>::SharedPtr & client,
  const std::string & service_name)
{
  const auto timeout = std::chrono::duration<double>(costmap_service_timeout_sec_);
  if (!client->wait_for_service(timeout)) {
    RCLCPP_WARN(logger_, "Costmap clear service unavailable: %s", service_name.c_str());
    return;
  }
  const auto request = std::make_shared<ClearEntireCostmap::Request>();
  const auto future = client->async_send_request(request);
  if (future.wait_for(timeout) != std::future_status::ready) {
    RCLCPP_WARN(logger_, "Costmap clear timed out: %s", service_name.c_str());
  }
}

}  // namespace acadbot_courier
