#include "acadbot_courier/bt_nodes.hpp"

#include <memory>
#include <string>
#include <utility>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace acadbot_courier
{

namespace
{
geometry_msgs::msg::PoseStamped to_pose_stamped(
  const Pose2D & pose, const std::string & frame_id, const rclcpp::Time & stamp)
{
  geometry_msgs::msg::PoseStamped out;
  out.header.frame_id = frame_id;
  out.header.stamp = stamp;
  out.pose.position.x = pose.x;
  out.pose.position.y = pose.y;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, pose.yaw);
  out.pose.orientation = tf2::toMsg(q);
  return out;
}
}  // namespace

GoToLocation::GoToLocation(
  const std::string & name, const BT::NodeConfig & config, const NavContext & context)
: BT::StatefulActionNode(name, config), context_(context)
{
}

BT::PortsList GoToLocation::providedPorts()
{
  return {
    BT::InputPort<Pose2D>("pose", "Target coordinates in the goal frame"),
    BT::InputPort<std::string>("location_name", "Location name, for feedback and logging"),
    BT::InputPort<std::string>("leg", "pickup or dropoff"),
  };
}

BT::NodeStatus GoToLocation::onStart()
{
  Pose2D target_pose;
  if (!getInput("pose", target_pose)) {
    RCLCPP_ERROR(context_.node->get_logger(), "GoToLocation: missing 'pose' port");
    return BT::NodeStatus::FAILURE;
  }

  std::string location_name;
  if (!getInput("location_name", location_name)) {
    location_name = "unknown";
  }

  std::string leg_name;
  if (!getInput("leg", leg_name)) {
    leg_name = "pickup";
  }

  if (!context_.nav_client || !context_.nav_client->action_server_is_ready()) {
    RCLCPP_ERROR(context_.node->get_logger(), "GoToLocation: Nav2 action server is not ready");
    return BT::NodeStatus::FAILURE;
  }

  if (context_.status) {
    // Compared before it is assigned, because assigning first would erase
    // the very change being looked for.
    const Leg leg = (leg_name == "pickup") ? Leg::PICKUP : Leg::DROPOFF;
    if (leg != context_.status->leg) {
      context_.status->attempt = 0;
    }
    context_.status->leg = leg;
    context_.status->target_location = location_name;
    context_.status->state = "NAVIGATING";
    context_.status->attempt++;
    context_.status->attempts_total++;
    context_.status->distance_remaining = 0.0f;
    context_.status->nav2_recoveries = 0;
  }

  finished_ = false;
  outcome_ = false;
  halted_ = false;
  nav_handle_.reset();

  NavigateToPose::Goal goal;
  goal.pose = to_pose_stamped(target_pose, context_.goal_frame, context_.node->now());

  // Guarded by `alive` because these are bound to a tree leaf, and a leaf
  // dies with its tree when the next job replaces it.
  std::weak_ptr<bool> alive = alive_;

  rclcpp_action::Client<NavigateToPose>::SendGoalOptions opts;
  opts.goal_response_callback =
    [this, alive](NavGoalHandle::SharedPtr handle) {
      if (alive.expired()) { return; }
      on_goal_response(handle);
    };
  opts.feedback_callback =
    [this, alive](NavGoalHandle::SharedPtr handle,
           const std::shared_ptr<const NavigateToPose::Feedback> fb) {
      if (alive.expired()) { return; }
      on_feedback(handle, fb);
    };
  opts.result_callback =
    [this, alive](const NavGoalHandle::WrappedResult & result) {
      if (alive.expired()) { return; }
      on_result(result);
    };

  RCLCPP_INFO(
    context_.node->get_logger(),
    "BT GoToLocation: driving to '%s' (leg: %s, attempt %d)",
    location_name.c_str(), leg_name.c_str(), context_.status ? context_.status->attempt : 1);

  context_.nav_client->async_send_goal(goal, opts);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GoToLocation::onRunning()
{
  if (!finished_) {
    return BT::NodeStatus::RUNNING;
  }

  if (outcome_) {
    RCLCPP_INFO(
      context_.node->get_logger(), "BT GoToLocation: successfully reached target");
    return BT::NodeStatus::SUCCESS;
  }

  RCLCPP_WARN(
    context_.node->get_logger(), "BT GoToLocation: failed to reach target");
  if (context_.status) {
    context_.status->state = "RETRYING";
  }
  return BT::NodeStatus::FAILURE;
}

void GoToLocation::onHalted()
{
  RCLCPP_INFO(context_.node->get_logger(), "BT GoToLocation: halted/cancelled");
  halted_ = true;
  if (nav_handle_) {
    context_.nav_client->async_cancel_goal(nav_handle_);
  }
  // With no handle yet, on_goal_response cancels it when it arrives.
  finished_ = true;
  outcome_ = false;
  nav_handle_.reset();
}

void GoToLocation::on_goal_response(const NavGoalHandle::SharedPtr & handle)
{
  if (!handle) {
    RCLCPP_WARN(context_.node->get_logger(), "BT GoToLocation: Nav2 rejected goal");
    finished_ = true;
    outcome_ = false;
    return;
  }

  if (halted_) {
    // The halt landed before Nav2 acknowledged the goal; without this the
    // tree stops while the robot drives on.
    RCLCPP_INFO(
      context_.node->get_logger(),
      "BT GoToLocation: goal acknowledged after the halt -- cancelling it");
    context_.nav_client->async_cancel_goal(handle);
    return;
  }

  nav_handle_ = handle;
}

void GoToLocation::on_feedback(
  NavGoalHandle::SharedPtr,
  const std::shared_ptr<const NavigateToPose::Feedback> feedback)
{
  if (context_.status) {
    context_.status->distance_remaining = feedback->distance_remaining;
    context_.status->nav2_recoveries = static_cast<uint16_t>(feedback->number_of_recoveries);
  }
}

void GoToLocation::on_result(const NavGoalHandle::WrappedResult & result)
{
  finished_ = true;
  outcome_ = (result.code == rclcpp_action::ResultCode::SUCCEEDED);
  nav_handle_.reset();
}

void register_courier_nodes(BT::BehaviorTreeFactory & factory, const NavContext & context)
{
  factory.registerBuilder<GoToLocation>(
    "GoToLocation",
    [context](const std::string & name, const BT::NodeConfig & config) {
      return std::make_unique<GoToLocation>(name, config, context);
    });
}

}  // namespace acadbot_courier
