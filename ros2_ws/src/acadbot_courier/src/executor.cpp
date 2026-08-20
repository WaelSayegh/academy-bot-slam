// executor.cpp — see executor.hpp for the contract this implements.
#include "executor.hpp"
#include <tf2/LinearMath/Quaternion.h>

// Executor owns the action client and manages sending Nav2 goals. Important
// ownership notes:
// - `node_` is a non-owning raw pointer to the rclcpp::Node that created
//   this Executor. The caller must guarantee the Node outlives the Executor.
// - `client_` is a SharedPtr returned by `create_client` and manages the
//   action-client connection internally.
Executor::Executor(rclcpp::Node * node)
: node_(node)
{
  // Create an action client for Nav2's `navigate_to_pose` action. The
  // returned shared pointer is stored in `client_` and cleaned up when this
  // Executor object is destroyed (RAII semantics).
  client_ = rclcpp_action::create_client<NavigateToPose>(node_, "navigate_to_pose");
}

void Executor::driveTo(double x, double y, double yaw,
                        const std::string & frame_id,
                        FeedbackCb on_feedback, DoneCb on_done)
{
  // Check that the action server is available. This call queries the
  // internally-held client_ and is safe to call from any thread that has
  // access to this Executor object.
  if (!client_->action_server_is_ready()) {
    RCLCPP_ERROR(node_->get_logger(), "Executor: Nav2 action server not ready.");
    goal_active_ = false;
    on_done(false, "nav2 not available");
    return;
  }

  NavigateToPose::Goal goal;
  goal.pose.header.frame_id = frame_id;
  goal.pose.header.stamp = node_->now();
  goal.pose.pose.position.x = x;
  goal.pose.pose.position.y = y;

  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  goal.pose.pose.orientation.x = q.x();
  goal.pose.pose.orientation.y = q.y();
  goal.pose.pose.orientation.z = q.z();
  goal.pose.pose.orientation.w = q.w();

  rclcpp_action::Client<NavigateToPose>::SendGoalOptions opts;

  // Fires once, immediately: did Nav2 accept the goal at all?
  // Capture `this` to update `current_goal_handle_` and `goal_active_`.
  // These lambdas are invoked asynchronously by rclcpp; they must not
  // outlive the Executor object. Because the Dispatcher owns the Executor
  // and the rclcpp spin loop runs until shutdown, this capture is safe in
  // this codebase's lifecycle model. For more robust code consider
  // `std::weak_ptr` promotions to detect object lifetime at callback time.
  opts.goal_response_callback =
    [this, on_done](GoalHandle::SharedPtr gh) {
      if (!gh) {
        RCLCPP_WARN(node_->get_logger(), "Executor: goal rejected by Nav2.");
        goal_active_ = false;
        on_done(false, "goal rejected by nav2");
        return;
      }
      // Keep a copy of the GoalHandle so we can cancel later.
      current_goal_handle_ = gh;
    };

  // Fires repeatedly while driving. The feedback callback forwards the
  // remaining distance to the caller-provided `on_feedback` callback. We
  // capture `on_feedback` by value; it should be a lightweight function or
  // lambda that is safe to invoke from the rclcpp thread.
  opts.feedback_callback =
    [on_feedback](GoalHandle::SharedPtr,
                  const std::shared_ptr<const NavigateToPose::Feedback> fb) {
      on_feedback(fb->distance_remaining);
    };

  // Fires exactly once when the goal ends, however it ends. We reset
  // `current_goal_handle_` to break ownership cycles and mark the goal as
  // inactive. `on_done` is invoked so higher-level code can react.
  opts.result_callback =
    [this, on_done](const GoalHandle::WrappedResult & result) {
      goal_active_ = false;
      // Release our copy of the GoalHandle — any remaining references will
      // be held by rclcpp internals or other callbacks.
      current_goal_handle_.reset();
      switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          on_done(true, "reached");
          break;
        case rclcpp_action::ResultCode::ABORTED:
          on_done(false, "nav2 aborted (recoveries exhausted)");
          break;
        case rclcpp_action::ResultCode::CANCELED:
          on_done(false, "canceled");
          break;
        default:
          on_done(false, "unknown result code");
          break;
      }
    };

  // Mark an outgoing goal as active and send it asynchronously. The
  // `async_send_goal` returns immediately; the three callbacks above drive
  // the goal lifecycle events.
  goal_active_ = true;
  client_->async_send_goal(goal, opts);
}

void Executor::cancel()
{
  // Cancel the currently outstanding goal if we have a handle. `current_goal_handle_`
  // is a SharedPtr that was set in the goal response callback. We do not
  // reset it here — the result callback will clear it when the cancel
  // completes — but calling async_cancel_goal is idempotent and thread-safe.
  if (current_goal_handle_) {
    client_->async_cancel_goal(current_goal_handle_);
  }
}