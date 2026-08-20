// ---------------------------------------------------------------------------
// AcadBot Courier — final project.
//
// Owns the set of named locations and the registry of booked/running/
// finished deliveries, and exposes /request_delivery as the entry point for
// booking a new job.
// ---------------------------------------------------------------------------
#include <chrono>
#include <cmath>
#include <cstdio>
#include <functional>
#include <map>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "acadbot_courier_interfaces/srv/request_delivery.hpp"
#include "acadbot_courier_interfaces/action/deliver.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

using namespace std::chrono_literals;
using RequestDelivery = acadbot_courier_interfaces::srv::RequestDelivery;
using Deliver = acadbot_courier_interfaces::action::Deliver;
using GoalHandleDeliver = rclcpp_action::ServerGoalHandle<Deliver>;
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

struct Pose2D { double x{0.0}, y{0.0}, yaw{0.0}; };

enum class JobState { BOOKED, RUNNING, DONE, FAILED, CANCELED, EXPIRED };

// Outcome of one navigate_to() round trip.
enum class NavOutcome { SUCCEEDED, ABORTED, REJECTED, CANCELED, TIMEOUT };

// navigate_to()'s full answer: what happened, and, for a failure, why —
// carrying Nav2's own error_msg forward rather than just a bare enum value.
struct NavResult { NavOutcome outcome; std::string detail; };

struct Job
{
  std::string id;
  std::string pickup;
  std::string dropoff;
  JobState state{JobState::BOOKED};
  rclcpp::Time booked_at;
};

class CourierNode : public rclcpp::Node
{
public:
  CourierNode() : Node("courier_node")
  {
    load_locations();

    booking_timeout_sec_ = declare_parameter<double>("booking_timeout_sec", 120.0);
    max_attempts_        = declare_parameter<int>("max_attempts", 3);
    // feedback_period_sec_ = declare_parameter<double>("feedback_period_sec", 1.0);
    // Unused since navigate_to() relays every Nav2 feedback tick directly
    // rather than throttling to a rate of its own.
    leg_timeout_sec_     = declare_parameter<double>("leg_timeout_sec", 60.0);

    // Booking stays MutuallyExclusive: two bookings can't run their checks
    // against jobs_ at the same time. The action side is Reentrant so the
    // action server's own callbacks and, once Phase 4 adds a Nav2 client,
    // Nav2's own callbacks can all be serviced without blocking each other.
    service_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    action_group_  = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    // ---- Booking service ----
    service_ = create_service<RequestDelivery>(
      "request_delivery",
      std::bind(&CourierNode::handle_request_delivery, this,
                std::placeholders::_1, std::placeholders::_2),
      rclcpp::ServicesQoS(), service_group_);

    // ---- Delivery action ----
    action_server_ = rclcpp_action::create_server<Deliver>(
      this,
      "deliver",
      std::bind(&CourierNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&CourierNode::handle_cancel, this, std::placeholders::_1),
      std::bind(&CourierNode::handle_accepted, this, std::placeholders::_1),
      rcl_action_server_get_default_options(), action_group_);

    // ---- Client of Nav2's own action, one per node not one per leg ----
    // Same Reentrant group as the /deliver server, so Nav2's callbacks and
    // our own action server's callbacks can both be serviced concurrently.
    nav_client_ = rclcpp_action::create_client<NavigateToPose>(
      this, "navigate_to_pose", action_group_);

    // ---- Sweep bookings nobody ever ran ----
    expiry_timer_ = create_wall_timer(
      5s, std::bind(&CourierNode::expire_stale_bookings, this));

    RCLCPP_INFO(get_logger(), "courier_node: %zu known locations, ready for bookings.",
                locations_.size());
  }

private:
  // Reads 'location_names', then for each name reads 'locations.<name>.x/y/yaw'.
  // ROS 2 parameters cannot hold a list of structs, so this names-list plus
  // per-name sub-parameters is the pattern that gets around that.
  void load_locations()
  {
    auto names = declare_parameter<std::vector<std::string>>("location_names", std::vector<std::string>{});
    if (names.empty()) {
      RCLCPP_FATAL(get_logger(),
        "No 'location_names' configured — refusing to start with no known locations.");
      throw std::runtime_error("no locations configured");
    }
    for (const auto & name : names) {
      Pose2D pose;
      pose.x   = declare_parameter<double>("locations." + name + ".x", 0.0);
      pose.y   = declare_parameter<double>("locations." + name + ".y", 0.0);
      pose.yaw = declare_parameter<double>("locations." + name + ".yaw", 0.0);
      locations_[name] = pose;
    }
  }

  std::string valid_location_list() const
  {
    std::string out;
    for (const auto & [name, pose] : locations_) {
      (void)pose;
      if (!out.empty()) out += ", ";
      out += name;
    }
    return out;
  }

  void handle_request_delivery(
    const std::shared_ptr<RequestDelivery::Request> request,
    std::shared_ptr<RequestDelivery::Response> response)
  {
    std::lock_guard<std::mutex> lock(jobs_mutex_);

    // Checked first, ahead of anything about this specific request: an
    // unready Nav2 blocks every booking equally, so there is no point
    // validating a pickup/dropoff pair the robot could not act on yet
    // anyway. action_server_is_ready() just reads the client's already-known
    // connection state — it is a cheap local check, not a round trip to
    // Nav2, so it is fine to call directly here under the lock.
    if (!nav_client_->action_server_is_ready()) {
      response->accepted = false;
      response->job_id = "";
      response->reason = "Nav2 is not ready yet — try again shortly";
      RCLCPP_WARN(get_logger(), "Rejected booking: %s", response->reason.c_str());
      return;
    }

    if (!locations_.count(request->pickup)) {
      response->accepted = false;
      response->job_id = "";
      response->reason = "unknown pickup '" + request->pickup +
        "' — known locations: " + valid_location_list();
      RCLCPP_WARN(get_logger(), "Rejected booking: %s", response->reason.c_str());
      return;
    }
    if (!locations_.count(request->dropoff)) {
      response->accepted = false;
      response->job_id = "";
      response->reason = "unknown dropoff '" + request->dropoff +
        "' — known locations: " + valid_location_list();
      RCLCPP_WARN(get_logger(), "Rejected booking: %s", response->reason.c_str());
      return;
    }
    if (request->pickup == request->dropoff) {
      response->accepted = false;
      response->job_id = "";
      response->reason = "pickup and dropoff must be different locations";
      RCLCPP_WARN(get_logger(), "Rejected booking: %s", response->reason.c_str());
      return;
    }

    std::string blocking_id;
    for (const auto & [id, job] : jobs_) {
      if (job.state == JobState::RUNNING) {
        blocking_id = id;
        break;
      }
    }
    if (!blocking_id.empty()) {
      response->accepted = false;
      response->job_id = "";
      response->reason = "busy, " + blocking_id + " is in progress";
      RCLCPP_WARN(get_logger(), "Rejected booking: %s", response->reason.c_str());
      return;
    }

    const std::string job_id = next_job_id();
    jobs_[job_id] = Job{job_id, request->pickup, request->dropoff, JobState::BOOKED, now()};

    response->accepted = true;
    response->job_id = job_id;
    response->reason = "booked";
    RCLCPP_INFO(get_logger(), "Booked %s: %s -> %s",
                job_id.c_str(), request->pickup.c_str(), request->dropoff.c_str());
  }

  std::string next_job_id()
  {
    char buf[16];
    std::snprintf(buf, sizeof(buf), "D-%04d", next_job_number_++);
    return std::string(buf);
  }

  void expire_stale_bookings()
  {
    std::lock_guard<std::mutex> lock(jobs_mutex_);
    for (auto & [id, job] : jobs_) {
      if (job.state == JobState::BOOKED &&
          (now() - job.booked_at).seconds() > booking_timeout_sec_) {
        job.state = JobState::EXPIRED;
        RCLCPP_WARN(get_logger(), "%s expired: booked but never run.", id.c_str());
      }
    }
  }

  // ---- /deliver goal acceptance ----
  // Four checks, in order: job_id must be non-empty, must name a job we
  // actually know about, that job must still be BOOKED (covers already
  // RUNNING, DONE, FAILED, CANCELED and EXPIRED in one check), and nothing
  // else may already be RUNNING (one robot, one job at a time). Acceptance
  // and marking the job RUNNING happen under the same lock as the checks,
  // so two goals arriving back to back can't both pass the busy-check
  // before either has claimed the job.
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const Deliver::Goal> goal)
  {
    std::lock_guard<std::mutex> lock(jobs_mutex_);

    if (goal->job_id.empty()) {
      RCLCPP_WARN(get_logger(), "Rejected goal: no job_id provided");
      return rclcpp_action::GoalResponse::REJECT;
    }
    auto it = jobs_.find(goal->job_id);
    if (it == jobs_.end()) {
      RCLCPP_WARN(get_logger(), "Rejected goal: unknown job_id '%s'", goal->job_id.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (it->second.state != JobState::BOOKED) {
      RCLCPP_WARN(get_logger(), "Rejected goal: job '%s' is not runnable right now",
                  goal->job_id.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }
    for (const auto & [id, job] : jobs_) {
      if (job.state == JobState::RUNNING) {
        RCLCPP_WARN(get_logger(), "Rejected goal: busy, %s is in progress", id.c_str());
        return rclcpp_action::GoalResponse::REJECT;
      }
    }

    it->second.state = JobState::RUNNING;
    RCLCPP_INFO(get_logger(), "Accepted goal for %s", goal->job_id.c_str());
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  // Always allow a cancel request through. This callback only decides
  // whether cancelling is permitted at all, not whether it succeeds — the
  // actual stopping happens inside navigate_to()/execute(), which notice
  // is_canceling() and unwind.
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleDeliver>)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  // Goal callbacks (handle_goal, handle_cancel) must return fast, so the
  // actual multi-minute drive can't happen here. This hands it off to a
  // detached thread and returns immediately — the standard rclcpp_action
  // server pattern.
  void handle_accepted(const std::shared_ptr<GoalHandleDeliver> goal_handle)
  {
    std::thread(&CourierNode::execute, this, goal_handle).detach();
  }

  // State one navigate_to() call shares with Nav2's async callbacks, which
  // run on the node's executor threads rather than the detached execute()
  // thread navigate_to() itself runs on. Owned by a shared_ptr rather than
  // living on navigate_to()'s stack, since the timeout path below returns
  // without waiting for Nav2 to confirm a cancel — a callback can still fire
  // after that return, and needs somewhere valid to write into. Scoped to a
  // single call rather than kept as node members, so a late callback from an
  // abandoned goal can never bleed into a later delivery's in-flight state.
  struct NavCallState
  {
    std::mutex mutex;
    bool finished{false};
    bool goal_response_received{false};
    GoalHandleNav::SharedPtr nav_goal_handle;
    bool result_ready{false};
    rclcpp_action::ResultCode result_code{rclcpp_action::ResultCode::UNKNOWN};
    std::string error_msg;
  };

  // Drives one leg via Nav2's navigate_to_pose action and reports how it
  // went. Sends the goal, then polls with wait-and-check rather than any
  // blocking wait — the node's own MultiThreadedExecutor is what actually
  // services Nav2's callbacks, on threads this function does not own, so
  // blocking here would only stall the wait without helping it resolve.
  NavResult navigate_to(
    const std::shared_ptr<GoalHandleDeliver> & goal_handle,
    const std::string & job_id, const std::string & leg, const std::string & target,
    int attempt, int max_attempts)
  {
    const Pose2D pose = locations_.at(target);

    NavigateToPose::Goal nav_goal;
    nav_goal.pose.header.frame_id = "map";
    nav_goal.pose.header.stamp = now();
    nav_goal.pose.pose.position.x = pose.x;
    nav_goal.pose.pose.position.y = pose.y;
    nav_goal.pose.pose.orientation.z = std::sin(pose.yaw / 2.0);
    nav_goal.pose.pose.orientation.w = std::cos(pose.yaw / 2.0);

    auto state = std::make_shared<NavCallState>();

    rclcpp_action::Client<NavigateToPose>::SendGoalOptions opts;
    opts.goal_response_callback =
      [state](GoalHandleNav::SharedPtr nav_goal_handle) {
        std::lock_guard<std::mutex> lock(state->mutex);
        if (state->finished) return;
        state->goal_response_received = true;
        state->nav_goal_handle = nav_goal_handle;
      };
    opts.feedback_callback =
      [this, state, goal_handle, job_id, leg, target, attempt, max_attempts](
        GoalHandleNav::SharedPtr, const std::shared_ptr<const NavigateToPose::Feedback> nav_feedback) {
        std::lock_guard<std::mutex> lock(state->mutex);
        if (state->finished) return;

        auto feedback = std::make_shared<Deliver::Feedback>();
        feedback->job_id = job_id;
        feedback->leg = leg;
        feedback->target_location = target;
        feedback->distance_remaining = nav_feedback->distance_remaining;
        feedback->attempt = attempt;
        feedback->max_attempts = max_attempts;
        goal_handle->publish_feedback(feedback);
      };
    opts.result_callback =
      [state](const GoalHandleNav::WrappedResult & result) {
        std::lock_guard<std::mutex> lock(state->mutex);
        if (state->finished) return;
        state->result_ready = true;
        state->result_code = result.code;
        state->error_msg = result.result ? result.result->error_msg : "";
      };

    nav_client_->async_send_goal(nav_goal, opts);

    const auto deadline = now() + rclcpp::Duration::from_seconds(leg_timeout_sec_);
    while (rclcpp::ok()) {
      {
        std::lock_guard<std::mutex> lock(state->mutex);
        if (state->goal_response_received && !state->nav_goal_handle) {
          state->finished = true;
          RCLCPP_WARN(get_logger(), "Nav2 rejected the goal to '%s'", target.c_str());
          return NavResult{NavOutcome::REJECTED, "Nav2 rejected the goal"};
        }
        if (state->result_ready) {
          state->finished = true;
          switch (state->result_code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
              return NavResult{NavOutcome::SUCCEEDED, ""};
            case rclcpp_action::ResultCode::CANCELED:
              return NavResult{NavOutcome::CANCELED, ""};
            default:
              RCLCPP_WARN(get_logger(), "Nav2 aborted en route to '%s': %s",
                          target.c_str(), state->error_msg.c_str());
              return NavResult{NavOutcome::ABORTED, state->error_msg};
          }
        }
      }

      if (goal_handle->is_canceling()) {
        // The requester asked to stop and Nav2 is presumably still
        // responsive, so this waits (bounded by the same leg deadline) for
        // Nav2 to actually confirm the goal stopped, unlike the timeout
        // path below.
        bool cancel_sent = false;
        while (rclcpp::ok() && now() < deadline) {
          GoalHandleNav::SharedPtr nav_goal_handle;
          bool done;
          {
            std::lock_guard<std::mutex> lock(state->mutex);
            nav_goal_handle = state->nav_goal_handle;
            done = state->result_ready;
          }
          if (done) break;
          if (!cancel_sent && nav_goal_handle) {
            nav_client_->async_cancel_goal(nav_goal_handle);
            cancel_sent = true;
          }
          std::this_thread::sleep_for(100ms);
        }
        std::lock_guard<std::mutex> lock(state->mutex);
        state->finished = true;
        return NavResult{NavOutcome::CANCELED, ""};
      }

      if (now() > deadline) {
        // Nav2 hasn't answered in time. Cancel as cleanup, but don't wait
        // for confirmation: a goal already this unresponsive might never
        // confirm, and waiting here would defeat the point of timing out.
        GoalHandleNav::SharedPtr nav_goal_handle;
        {
          std::lock_guard<std::mutex> lock(state->mutex);
          nav_goal_handle = state->nav_goal_handle;
        }
        if (nav_goal_handle) {
          nav_client_->async_cancel_goal(nav_goal_handle);
        }
        RCLCPP_WARN(get_logger(), "Nav2 did not answer within %.0fs for '%s'",
                    leg_timeout_sec_, target.c_str());
        std::lock_guard<std::mutex> lock(state->mutex);
        state->finished = true;
        return NavResult{NavOutcome::TIMEOUT,
                          "Nav2 did not respond within " +
                            std::to_string(static_cast<int>(leg_timeout_sec_)) + "s"};
      }

      std::this_thread::sleep_for(100ms);
    }

    std::lock_guard<std::mutex> lock(state->mutex);
    state->finished = true;
    return NavResult{NavOutcome::ABORTED, "node is shutting down"};
  }

  // Drives one leg, retrying on ABORTED/TIMEOUT up to max_attempts_.
  // REJECTED and CANCELED are not retried: a pose Nav2 refused outright
  // won't become valid by asking again, and a cancel means the requester
  // wants to stop, not that the robot should try harder.
  NavResult drive_leg_with_retries(
    const std::shared_ptr<GoalHandleDeliver> & goal_handle,
    const std::string & job_id, const std::string & leg, const std::string & target)
  {
    NavResult result;
    for (int attempt = 1; attempt <= max_attempts_; ++attempt) {
      result = navigate_to(goal_handle, job_id, leg, target, attempt, max_attempts_);
      if (result.outcome == NavOutcome::SUCCEEDED ||
          result.outcome == NavOutcome::CANCELED ||
          result.outcome == NavOutcome::REJECTED)
      {
        return result;
      }
      // ABORTED or TIMEOUT: worth trying again unless attempts are exhausted.
    }
    return result;
  }

  // Moves the job to its terminal state and ends the action accordingly.
  void finish(
    const std::shared_ptr<GoalHandleDeliver> & goal_handle, const std::string & job_id,
    JobState final_state, bool success, const std::string & failed_leg,
    const std::string & message)
  {
    {
      std::lock_guard<std::mutex> lock(jobs_mutex_);
      jobs_[job_id].state = final_state;
    }

    auto result = std::make_shared<Deliver::Result>();
    result->success = success;
    result->job_id = job_id;
    result->failed_leg = failed_leg;
    result->message = message;

    if (final_state == JobState::CANCELED) {
      goal_handle->canceled(result);
    } else if (final_state == JobState::DONE) {
      goal_handle->succeed(result);
    } else {
      goal_handle->abort(result);
    }
    RCLCPP_INFO(get_logger(), "%s finished: %s", job_id.c_str(), message.c_str());
  }

  // Drives the pickup leg, then the dropoff leg, retrying each one through
  // drive_leg_with_retries(). A CANCELED outcome always means the requester
  // asked to stop; anything else short of SUCCEEDED, after retries are
  // exhausted, means the job genuinely failed.
  void execute(const std::shared_ptr<GoalHandleDeliver> goal_handle)
  {
    const std::string job_id = goal_handle->get_goal()->job_id;
    std::string pickup, dropoff;
    {
      std::lock_guard<std::mutex> lock(jobs_mutex_);
      pickup = jobs_[job_id].pickup;
      dropoff = jobs_[job_id].dropoff;
    }

    auto failure_message = [](const std::string & leg, const std::string & detail) {
      std::string message = "navigation to " + leg + " failed";
      if (!detail.empty()) message += ": " + detail;
      return message;
    };

    auto result = drive_leg_with_retries(goal_handle, job_id, Deliver::Feedback::LEG_PICKUP, pickup);
    if (result.outcome == NavOutcome::CANCELED) {
      finish(goal_handle, job_id, JobState::CANCELED, false, "",
             "cancelled during pickup leg");
      return;
    }
    if (result.outcome != NavOutcome::SUCCEEDED) {
      finish(goal_handle, job_id, JobState::FAILED, false, "pickup",
             failure_message("pickup", result.detail));
      return;
    }

    result = drive_leg_with_retries(goal_handle, job_id, Deliver::Feedback::LEG_DROPOFF, dropoff);
    if (result.outcome == NavOutcome::CANCELED) {
      finish(goal_handle, job_id, JobState::CANCELED, false, "",
             "cancelled during dropoff leg");
      return;
    }
    if (result.outcome != NavOutcome::SUCCEEDED) {
      finish(goal_handle, job_id, JobState::FAILED, false, "dropoff",
             failure_message("dropoff", result.detail));
      return;
    }

    finish(goal_handle, job_id, JobState::DONE, true, "", "delivered");
  }

  std::map<std::string, Pose2D> locations_;
  std::map<std::string, Job> jobs_;
  std::mutex jobs_mutex_;
  int next_job_number_{1};
  double booking_timeout_sec_;
  int max_attempts_;
  // double feedback_period_sec_;
  double leg_timeout_sec_;

  rclcpp::CallbackGroup::SharedPtr service_group_;
  rclcpp::CallbackGroup::SharedPtr action_group_;
  rclcpp::Service<RequestDelivery>::SharedPtr service_;
  rclcpp_action::Server<Deliver>::SharedPtr action_server_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  rclcpp::TimerBase::SharedPtr expiry_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CourierNode>();
  // A plain rclcpp::spin() only ever uses one thread. That's fine while
  // execute() runs on its own detached thread, but Phase 4's Nav2 client
  // callbacks and the booking service both need to be serviced *while*
  // execute() is mid-drive, so this needs real worker threads behind it.
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
