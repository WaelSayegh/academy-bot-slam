#include "acadbot_courier/courier_server.hpp"

#include <chrono>
#include <exception>
#include <iomanip>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <utility>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "acadbot_courier/location_markers.hpp"

namespace acadbot_courier
{

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

namespace
{
constexpr double kMarkerPeriod = 1.0;   // s
}  // namespace

// ---- Construction ----------------------------------------------------------
CourierServer::CourierServer()
: Node("courier_server")
{
  goal_frame_ = declare_parameter<std::string>("goal_frame", "map");
  max_retries_ = declare_parameter<int>("max_retries", 2);
  retry_delay_ = declare_parameter<double>("retry_delay", 3.0);
  feedback_period_ = declare_parameter<double>("feedback_period", 0.5);
  leg_timeout_ = declare_parameter<double>("leg_timeout", 120.0);

  // Throws on a bad floor plan; main() catches and reports it.
  locations_ = LocationTable::from_parameters(*this);

  nav_client_ =
    rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

  request_service_ = create_service<RequestDelivery>(
    "request_delivery",
    std::bind(&CourierServer::handle_request, this, _1, _2));

  delivery_server_ = rclcpp_action::create_server<ExecuteDelivery>(
    this, "execute_delivery",
    std::bind(&CourierServer::handle_goal, this, _1, _2),
    std::bind(&CourierServer::handle_cancel, this, _1),
    std::bind(&CourierServer::handle_accepted, this, _1));

  rclcpp::QoS marker_qos(1);
  marker_qos.transient_local();
  marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    "~/locations", marker_qos);

  // Timed as well as latched, because RViz's MarkerArray display is volatile.
  marker_timer_ = create_wall_timer(
    std::chrono::duration<double>(kMarkerPeriod),
    std::bind(&CourierServer::publish_markers, this));

  RCLCPP_INFO(get_logger(),
    "courier_server ready: %zu locations (%s); %d retries per leg, "
    "%.0fs leg timeout",
    locations_.size(), locations_.known_names().c_str(),
    max_retries_, leg_timeout_);
}

// ---- Booking: validate, mint an id, reply immediately ----------------------
void CourierServer::handle_request(
  const std::shared_ptr<RequestDelivery::Request> request,
  std::shared_ptr<RequestDelivery::Response> response)
{
  const auto reject = [&](const std::string & why) {
      response->accepted = false;
      response->job_id = "";
      response->reason = why;
      RCLCPP_WARN(get_logger(), "rejected '%s' -> '%s': %s",
        request->pickup.c_str(), request->dropoff.c_str(), why.c_str());
    };

  // A running job owns the robot.
  if (!active_job_id_.empty()) {
    reject("a delivery is already running (" + active_job_id_ + ")");
    return;
  }

  const auto pickup = locations_.find(request->pickup);
  if (!pickup) {
    reject("unknown pickup '" + request->pickup + "'; known locations are: " +
      locations_.known_names());
    return;
  }

  const auto dropoff = locations_.find(request->dropoff);
  if (!dropoff) {
    reject("unknown dropoff '" + request->dropoff + "'; known locations are: " +
      locations_.known_names());
    return;
  }

  if (request->pickup == request->dropoff) {
    reject("pickup and dropoff are both '" + request->pickup +
      "'; there is nothing to deliver");
    return;
  }

  // Checked rather than waited on, so the reply stays immediate.
  if (!nav_client_->action_server_is_ready()) {
    reject("navigation is not available: Nav2's navigate_to_pose action "
      "server is not up");
    return;
  }

  std::ostringstream id;
  id << "job_" << std::setw(4) << std::setfill('0') << ++next_job_number_;

  Job job;
  job.id = id.str();
  job.pickup_name = request->pickup;
  job.dropoff_name = request->dropoff;
  job.pickup = *pickup;     // resolved once, here, and frozen
  job.dropoff = *dropoff;
  job.state = JobState::BOOKED;
  jobs_.emplace(job.id, job);

  response->accepted = true;
  response->job_id = job.id;
  response->reason = "";

  RCLCPP_INFO(get_logger(), "booked %s: '%s' -> '%s'",
    job.id.c_str(), job.pickup_name.c_str(), job.dropoff_name.c_str());
}

// ---- Execution: the action server ------------------------------------------
rclcpp_action::GoalResponse CourierServer::handle_goal(
  const rclcpp_action::GoalUUID &,
  std::shared_ptr<const ExecuteDelivery::Goal> goal)
{
  if (!active_job_id_.empty()) {
    RCLCPP_WARN(get_logger(), "goal for %s rejected: %s is already running",
      goal->job_id.c_str(), active_job_id_.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }

  const auto it = jobs_.find(goal->job_id);
  if (it == jobs_.end()) {
    RCLCPP_WARN(get_logger(), "goal rejected: no job '%s' was ever booked",
      goal->job_id.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }

  // Finished jobs are kept so a replay is refused, not delivered twice.
  if (it->second.state != JobState::BOOKED) {
    RCLCPP_WARN(get_logger(), "goal for %s rejected: it is already %s",
      goal->job_id.c_str(), to_string(it->second.state));
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (!nav_client_->action_server_is_ready()) {
    RCLCPP_WARN(get_logger(), "goal for %s rejected: Nav2 is not available",
      goal->job_id.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse CourierServer::handle_cancel(
  const std::shared_ptr<ExecuteGoalHandle> goal_handle)
{
  if (!execute_handle_ || goal_handle != execute_handle_) {
    return rclcpp_action::CancelResponse::REJECT;
  }

  RCLCPP_INFO(get_logger(), "%s: cancel requested", active_job_id_.c_str());
  cancel_reason_ = CancelReason::CLIENT_REQUEST;

  if (phase_ == Phase::RETRY_WAIT) {
    // Nothing is driving, so no result is coming to finish on -- and the goal
    // only enters CANCELING once this returns, so finish on the next spin.
    if (retry_timer_) {
      retry_timer_->cancel();
    }
    retry_timer_ = create_wall_timer(1ms, [this]() {
        retry_timer_->cancel();
        finish_canceled();
      });
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  phase_ = Phase::CANCELING;
  if (nav_handle_) {
    nav_client_->async_cancel_goal(nav_handle_);
  }
  // With no handle yet, on_nav_goal_response cancels it on arrival.
  return rclcpp_action::CancelResponse::ACCEPT;
}

void CourierServer::handle_accepted(
  const std::shared_ptr<ExecuteGoalHandle> goal_handle)
{
  execute_handle_ = goal_handle;
  active_job_id_ = goal_handle->get_goal()->job_id;
  current_job().state = JobState::RUNNING;

  leg_ = Leg::PICKUP;
  phase_ = Phase::NAVIGATING;
  cancel_reason_ = CancelReason::NONE;
  attempt_ = 0;
  attempts_total_ = 0;

  // Our own clock, so a stalled Nav2 shows a frozen distance, not silence.
  feedback_timer_ = create_wall_timer(
    std::chrono::duration<double>(feedback_period_),
    std::bind(&CourierServer::publish_feedback, this));

  RCLCPP_INFO(get_logger(), "%s: starting; pickup '%s', then dropoff '%s'",
    active_job_id_.c_str(),
    current_job().pickup_name.c_str(),
    current_job().dropoff_name.c_str());

  send_leg_goal();
}

// ---- The state machine -----------------------------------------------------
void CourierServer::send_leg_goal()
{
  Job & job = current_job();
  ++attempt_;
  ++attempts_total_;
  phase_ = Phase::NAVIGATING;
  nav_handle_.reset();
  last_distance_ = 0.0f;
  last_recoveries_ = 0;

  const Pose2D & target = job.pose_of(leg_);

  NavigateToPose::Goal goal;
  goal.pose = to_goal_pose(target);

  rclcpp_action::Client<NavigateToPose>::SendGoalOptions opts;
  opts.goal_response_callback =
    [this](NavGoalHandle::SharedPtr handle) {on_nav_goal_response(handle);};
  opts.feedback_callback =
    [this](NavGoalHandle::SharedPtr handle,
      const std::shared_ptr<const NavigateToPose::Feedback> feedback) {
      on_nav_feedback(handle, feedback);
    };
  opts.result_callback =
    [this](const NavGoalHandle::WrappedResult & result) {on_nav_result(result);};

  RCLCPP_INFO(get_logger(), "%s: %s leg, attempt %d of %d -> '%s' (%.2f, %.2f)",
    job.id.c_str(), to_string(leg_), attempt_, max_retries_ + 1,
    job.name_of(leg_).c_str(), target.x, target.y);

  nav_client_->async_send_goal(goal, opts);

  leg_timeout_timer_ = create_wall_timer(
    std::chrono::duration<double>(leg_timeout_),
    std::bind(&CourierServer::on_leg_timeout, this));

  publish_markers();     // move the highlight to the new target
}

void CourierServer::on_nav_goal_response(const NavGoalHandle::SharedPtr & handle)
{
  if (active_job_id_.empty()) {
    return;
  }

  if (!handle) {
    // A leg that was never driven is a leg that failed.
    leg_failed("Nav2 rejected the goal");
    return;
  }

  nav_handle_ = handle;

  // The cancel landed before the handle did; there is something to cancel now.
  if (cancel_reason_ != CancelReason::NONE) {
    nav_client_->async_cancel_goal(nav_handle_);
  }
}

void CourierServer::on_nav_feedback(
  NavGoalHandle::SharedPtr,
  const std::shared_ptr<const NavigateToPose::Feedback> feedback)
{
  last_distance_ = feedback->distance_remaining;
  last_recoveries_ = static_cast<uint16_t>(feedback->number_of_recoveries);
}

void CourierServer::on_nav_result(const NavGoalHandle::WrappedResult & result)
{
  if (leg_timeout_timer_) {
    leg_timeout_timer_->cancel();
  }
  nav_handle_.reset();

  if (active_job_id_.empty()) {
    return;                   // a late result for a job that already ended
  }

  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      if (cancel_reason_ == CancelReason::CLIENT_REQUEST) {
        // Arrived as the cancel landed, so this is cancelled, not successful.
        finish_canceled();
        return;
      }
      if (leg_ == Leg::PICKUP) {
        RCLCPP_INFO(get_logger(), "%s: reached pickup '%s'",
          active_job_id_.c_str(), current_job().pickup_name.c_str());
        leg_ = Leg::DROPOFF;
        attempt_ = 0;
        send_leg_goal();
      } else {
        finish_succeeded();
      }
      return;

    case rclcpp_action::ResultCode::CANCELED:
      // Ambiguous on its own, hence the reason recorded at cancel time.
      switch (cancel_reason_) {
        case CancelReason::CLIENT_REQUEST:
          finish_canceled();
          return;
        case CancelReason::LEG_TIMEOUT:
          cancel_reason_ = CancelReason::NONE;
          leg_failed("attempt ran past the " +
            std::to_string(static_cast<int>(leg_timeout_)) + "s leg timeout");
          return;
        case CancelReason::NONE:
          leg_failed("Nav2 cancelled the goal on its own");
          return;
      }
      return;

    case rclcpp_action::ResultCode::ABORTED:
      leg_failed("Nav2 aborted the goal (its recoveries were exhausted)");
      return;

    default:
      leg_failed("Nav2 returned an unknown result code");
      return;
  }
}

void CourierServer::on_leg_timeout()
{
  if (leg_timeout_timer_) {
    leg_timeout_timer_->cancel();
  }
  if (active_job_id_.empty() || phase_ != Phase::NAVIGATING) {
    return;
  }

  RCLCPP_WARN(get_logger(), "%s: %s leg attempt %d ran past leg_timeout (%.0fs)",
    active_job_id_.c_str(), to_string(leg_), attempt_, leg_timeout_);

  if (!nav_handle_) {
    // Nav2 never acknowledged the goal, so there is nothing to cancel.
    leg_failed("Nav2 never acknowledged the goal");
    return;
  }

  cancel_reason_ = CancelReason::LEG_TIMEOUT;
  phase_ = Phase::CANCELING;
  nav_client_->async_cancel_goal(nav_handle_);
}

void CourierServer::on_retry_elapsed()
{
  if (retry_timer_) {
    retry_timer_->cancel();
  }
  if (active_job_id_.empty()) {
    return;
  }
  if (cancel_reason_ != CancelReason::NONE) {
    finish_canceled();
    return;
  }
  send_leg_goal();
}

void CourierServer::leg_failed(const std::string & why)
{
  Job & job = current_job();
  RCLCPP_WARN(get_logger(), "%s: %s leg, attempt %d failed: %s",
    job.id.c_str(), to_string(leg_), attempt_, why.c_str());

  if (attempt_ <= max_retries_) {
    phase_ = Phase::RETRY_WAIT;
    RCLCPP_INFO(get_logger(), "%s: retrying the %s leg in %.1fs (attempt %d of %d)",
      job.id.c_str(), to_string(leg_), retry_delay_,
      attempt_ + 1, max_retries_ + 1);
    retry_timer_ = create_wall_timer(
      std::chrono::duration<double>(retry_delay_),
      std::bind(&CourierServer::on_retry_elapsed, this));
    return;
  }

  finish_failed(std::string("the ") + to_string(leg_) + " leg failed after " +
    std::to_string(attempt_) + " attempts: " + why);
}

// ---- Endings: exactly one runs per accepted goal ---------------------------
void CourierServer::finish_succeeded()
{
  Job & job = current_job();
  auto result = std::make_shared<ExecuteDelivery::Result>();
  result->success = true;
  result->job_id = job.id;
  result->message =
    "delivered from '" + job.pickup_name + "' to '" + job.dropoff_name + "'";
  result->failed_leg = "";
  result->attempts_used = static_cast<uint8_t>(attempts_total_);

  job.state = JobState::SUCCEEDED;
  RCLCPP_INFO(get_logger(), "%s: %s", job.id.c_str(), result->message.c_str());

  execute_handle_->succeed(result);
  reset_to_idle();
}

void CourierServer::finish_failed(const std::string & message)
{
  Job & job = current_job();
  auto result = std::make_shared<ExecuteDelivery::Result>();
  result->success = false;
  result->job_id = job.id;
  result->message = message;
  result->failed_leg = to_string(leg_);
  result->attempts_used = static_cast<uint8_t>(attempts_total_);

  job.state = JobState::FAILED;
  RCLCPP_ERROR(get_logger(), "%s: %s", job.id.c_str(), message.c_str());

  // abort(), never succeed().
  execute_handle_->abort(result);
  reset_to_idle();
}

void CourierServer::finish_canceled()
{
  Job & job = current_job();
  auto result = std::make_shared<ExecuteDelivery::Result>();
  result->success = false;
  result->job_id = job.id;
  result->message = std::string("cancelled during the ") + to_string(leg_) +
    " leg, heading for '" + job.name_of(leg_) + "'";
  result->failed_leg = "";      // nothing failed; it was called off
  result->attempts_used = static_cast<uint8_t>(attempts_total_);

  job.state = JobState::CANCELED;
  RCLCPP_INFO(get_logger(), "%s: %s", job.id.c_str(), result->message.c_str());

  execute_handle_->canceled(result);
  reset_to_idle();
}

void CourierServer::reset_to_idle()
{
  if (feedback_timer_) {feedback_timer_->cancel();}
  if (retry_timer_) {retry_timer_->cancel();}
  if (leg_timeout_timer_) {leg_timeout_timer_->cancel();}
  feedback_timer_.reset();
  retry_timer_.reset();
  leg_timeout_timer_.reset();

  execute_handle_.reset();
  nav_handle_.reset();
  active_job_id_.clear();
  cancel_reason_ = CancelReason::NONE;
  phase_ = Phase::NAVIGATING;
  attempt_ = 0;
  attempts_total_ = 0;
  last_distance_ = 0.0f;
  last_recoveries_ = 0;

  publish_markers();     // drop the highlight
}

// ---- Output ----------------------------------------------------------------
void CourierServer::publish_feedback()
{
  if (active_job_id_.empty() || !execute_handle_) {
    return;
  }

  Job & job = current_job();
  auto feedback = std::make_shared<ExecuteDelivery::Feedback>();
  feedback->leg = (leg_ == Leg::PICKUP)
    ? ExecuteDelivery::Feedback::LEG_PICKUP
    : ExecuteDelivery::Feedback::LEG_DROPOFF;
  feedback->target_location = job.name_of(leg_);
  feedback->distance_remaining = last_distance_;
  feedback->attempt = static_cast<uint8_t>(attempt_);
  feedback->nav2_recoveries = last_recoveries_;
  feedback->state = phase_name(phase_);

  execute_handle_->publish_feedback(feedback);
}

void CourierServer::publish_markers()
{
  std::string active;
  if (!active_job_id_.empty()) {
    active = current_job().name_of(leg_);
  }
  marker_pub_->publish(
    make_location_markers(locations_, goal_frame_, now(), active));
}

// ---- Helpers ---------------------------------------------------------------
geometry_msgs::msg::PoseStamped CourierServer::to_goal_pose(
  const Pose2D & pose) const
{
  geometry_msgs::msg::PoseStamped out;
  out.header.frame_id = goal_frame_;
  out.header.stamp = now();
  out.pose.position.x = pose.x;
  out.pose.position.y = pose.y;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, pose.yaw);
  out.pose.orientation = tf2::toMsg(q);
  return out;
}

Job & CourierServer::current_job()
{
  const auto it = jobs_.find(active_job_id_);
  if (it == jobs_.end()) {
    throw std::runtime_error("no job '" + active_job_id_ + "' is active");
  }
  return it->second;
}

const char * CourierServer::phase_name(Phase phase)
{
  switch (phase) {
    case Phase::NAVIGATING: return "NAVIGATING";
    case Phase::RETRY_WAIT: return "RETRYING";
    case Phase::CANCELING:  return "CANCELING";
  }
  return "UNKNOWN";
}

}  // namespace acadbot_courier

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    // Single-threaded on purpose: nothing blocks, so serialising callbacks
    // is enough to make the state safe without a mutex.
    rclcpp::spin(std::make_shared<acadbot_courier::CourierServer>());
  } catch (const std::exception & e) {
    // Caught so a bad floor plan prints one fatal line, not an abort trace.
    RCLCPP_FATAL(rclcpp::get_logger("courier_server"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
