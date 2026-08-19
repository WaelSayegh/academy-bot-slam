// ---------------------------------------------------------------------------
// AcadBot Courier — final project.
//
// Owns the set of named locations and the registry of booked/running/
// finished deliveries, and exposes /request_delivery as the entry point for
// booking a new job.
// ---------------------------------------------------------------------------
#include <chrono>
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

using namespace std::chrono_literals;
using RequestDelivery = acadbot_courier_interfaces::srv::RequestDelivery;
using Deliver = acadbot_courier_interfaces::action::Deliver;
using GoalHandleDeliver = rclcpp_action::ServerGoalHandle<Deliver>;

struct Pose2D { double x{0.0}, y{0.0}, yaw{0.0}; };

enum class JobState { BOOKED, RUNNING, DONE, FAILED, CANCELED, EXPIRED };

// Outcome of one navigate_to() round trip. The stub can only ever produce
// SUCCEEDED or CANCELED; ABORTED, REJECTED and TIMEOUT become reachable once
// Phase 4 replaces the stub with a real Nav2 client.
enum class NavOutcome { SUCCEEDED, ABORTED, REJECTED, CANCELED, TIMEOUT };

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

    booking_timeout_sec_  = declare_parameter<double>("booking_timeout_sec", 120.0);
    max_attempts_         = declare_parameter<int>("max_attempts", 3);
    feedback_period_sec_  = declare_parameter<double>("feedback_period_sec", 1.0);
    stub_leg_duration_sec_ = declare_parameter<double>("stub_leg_duration_sec", 3.0);

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

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleDeliver>)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleDeliver> goal_handle)
  {
    std::thread(&CourierNode::execute, this, goal_handle).detach();
  }

  // Stub for one leg of the drive. Sleeps in 100ms ticks standing in for a
  // real navigate_to_pose round trip, checking for a cancel and publishing
  // feedback each tick. Phase 4 replaces the body of this function with a
  // real Nav2 action client; execute() below does not change.
  NavOutcome navigate_to(
    const std::shared_ptr<GoalHandleDeliver> & goal_handle,
    const std::string & job_id, const std::string & leg, const std::string & target,
    int attempt, int max_attempts)
  {
    const auto tick = 100ms;
    const int total_ticks = std::max(1, static_cast<int>(stub_leg_duration_sec_ * 1000 / 100));
    const int feedback_every = std::max(1, static_cast<int>(feedback_period_sec_ * 1000 / 100));
    float distance = 5.0f;

    for (int i = 0; i < total_ticks; ++i) {
      if (goal_handle->is_canceling()) {
        return NavOutcome::CANCELED;
      }
      if (i % feedback_every == 0) {
        auto feedback = std::make_shared<Deliver::Feedback>();
        feedback->job_id = job_id;
        feedback->leg = leg;
        feedback->target_location = target;
        feedback->distance_remaining = distance;
        feedback->attempt = attempt;
        feedback->max_attempts = max_attempts;
        goal_handle->publish_feedback(feedback);
        distance = std::max(0.0f, distance - 1.0f);
      }
      std::this_thread::sleep_for(tick);
    }
    return NavOutcome::SUCCEEDED;
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

  // Drives the pickup leg, then the dropoff leg. No retry loop yet — that is
  // Phase 5 — so any non-SUCCEEDED, non-CANCELED outcome ends the job as
  // FAILED immediately.
  void execute(const std::shared_ptr<GoalHandleDeliver> goal_handle)
  {
    const std::string job_id = goal_handle->get_goal()->job_id;
    std::string pickup, dropoff;
    {
      std::lock_guard<std::mutex> lock(jobs_mutex_);
      pickup = jobs_[job_id].pickup;
      dropoff = jobs_[job_id].dropoff;
    }

    auto outcome = navigate_to(goal_handle, job_id, Deliver::Feedback::LEG_PICKUP,
                                pickup, 1, max_attempts_);
    if (outcome == NavOutcome::CANCELED) {
      finish(goal_handle, job_id, JobState::CANCELED, false, "",
             "cancelled during pickup leg");
      return;
    }
    if (outcome != NavOutcome::SUCCEEDED) {
      finish(goal_handle, job_id, JobState::FAILED, false, "pickup",
             "navigation to pickup failed");
      return;
    }

    outcome = navigate_to(goal_handle, job_id, Deliver::Feedback::LEG_DROPOFF,
                           dropoff, 1, max_attempts_);
    if (outcome == NavOutcome::CANCELED) {
      finish(goal_handle, job_id, JobState::CANCELED, false, "",
             "cancelled during dropoff leg");
      return;
    }
    if (outcome != NavOutcome::SUCCEEDED) {
      finish(goal_handle, job_id, JobState::FAILED, false, "dropoff",
             "navigation to dropoff failed");
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
  double feedback_period_sec_;
  double stub_leg_duration_sec_;

  rclcpp::CallbackGroup::SharedPtr service_group_;
  rclcpp::CallbackGroup::SharedPtr action_group_;
  rclcpp::Service<RequestDelivery>::SharedPtr service_;
  rclcpp_action::Server<Deliver>::SharedPtr action_server_;
  rclcpp::TimerBase::SharedPtr expiry_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CourierNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
