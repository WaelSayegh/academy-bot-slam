#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include "acadbot_courier/job_registry.hpp"
#include "acadbot_courier/location_book.hpp"
#include "acadbot_courier/nav_leg_client.hpp"
#include "acadbot_courier/types.hpp"
#include "acadbot_courier_msgs/action/deliver.hpp"
#include "acadbot_courier_msgs/srv/request_delivery.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace acadbot_courier
{

using RequestDelivery = acadbot_courier_msgs::srv::RequestDelivery;
using Deliver = acadbot_courier_msgs::action::Deliver;
using DeliverGoalHandle = rclcpp_action::ServerGoalHandle<Deliver>;
using namespace std::chrono_literals;

class CourierNode : public rclcpp::Node
{
public:
  explicit CourierNode(const rclcpp::NodeOptions & options)
  : Node("courier_node", options)
  {
    const auto location_names = required_parameter<std::vector<std::string>>("location_names");
    if (location_names.empty()) {
      throw std::runtime_error("location_names must contain at least one location");
    }

    global_frame_ = required_parameter<std::string>("global_frame");
    std::unordered_map<std::string, LocationPose> locations;
    for (const auto & name : location_names) {
      const auto prefix = "locations." + name + ".";
      LocationPose location;
      if (!get_parameter(prefix + "x", location.x) ||
        !get_parameter(prefix + "y", location.y) ||
        !get_parameter(prefix + "yaw", location.yaw))
      {
        throw std::runtime_error(
                "location '" + name + "' is missing x, y, or yaw in locations.yaml");
      }
      locations.emplace(name, location);
    }
    location_book_ = std::make_unique<LocationBook>(location_names, locations, global_frame_);

    max_attempts_per_leg_ = required_parameter<std::int64_t>("max_attempts_per_leg");
    retry_backoff_sec_ = required_parameter<double>("retry_backoff_sec");
    clear_costmap_before_retry_ = required_parameter<bool>("clear_costmap_before_retry");
    leg_timeout_sec_ = required_parameter<double>("leg_timeout_sec");
    feedback_period_sec_ = required_parameter<double>("feedback_period_sec");
    job_ttl_sec_ = required_parameter<double>("job_ttl_sec");
    pickup_dwell_sec_ = required_parameter<double>("dwell.pickup");
    dropoff_dwell_sec_ = required_parameter<double>("dwell.dropoff");
    max_concurrent_jobs_ = required_parameter<std::int64_t>("max_concurrent_jobs");
    queue_depth_ = required_parameter<std::int64_t>("queue_depth");

    if (max_attempts_per_leg_ < 1 || retry_backoff_sec_ < 0.0 || leg_timeout_sec_ <= 0.0 ||
      feedback_period_sec_ <= 0.0 || job_ttl_sec_ <= 0.0 || pickup_dwell_sec_ < 0.0 ||
      dropoff_dwell_sec_ < 0.0)
    {
      throw std::runtime_error("courier timing and attempt parameters are outside valid ranges");
    }
    if (max_concurrent_jobs_ != 1 || queue_depth_ != 0) {
      throw std::runtime_error(
              "this Nav2-backed implementation requires max_concurrent_jobs=1 and queue_depth=0");
    }

    registry_ = std::make_unique<JobRegistry>(job_ttl_sec_);
    nav_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    server_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    nav_client_ = std::make_unique<NavLegClient>(
      this,
      nav_callback_group_,
      required_parameter<std::string>("nav_action_name"),
      global_frame_,
      required_parameter<std::string>("robot_base_frame"),
      required_parameter<std::string>("global_costmap_clear_service"),
      required_parameter<std::string>("local_costmap_clear_service"),
      required_parameter<double>("nav_goal_response_timeout_sec"),
      required_parameter<double>("nav_cancel_confirm_timeout_sec"),
      required_parameter<double>("costmap_service_timeout_sec"));

    request_service_ = create_service<RequestDelivery>(
      "~/request_delivery",
      std::bind(
        &CourierNode::handle_request, this, std::placeholders::_1, std::placeholders::_2),
      rclcpp::ServicesQoS(),
      server_callback_group_);

    action_server_ = rclcpp_action::create_server<Deliver>(
      this,
      "~/deliver",
      std::bind(
        &CourierNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&CourierNode::handle_cancel, this, std::placeholders::_1),
      std::bind(&CourierNode::handle_accepted, this, std::placeholders::_1),
      rcl_action_server_get_default_options(),
      server_callback_group_);

    feedback_timer_ = create_wall_timer(
      std::chrono::duration<double>(feedback_period_sec_),
      std::bind(&CourierNode::publish_feedback, this));
    expiry_timer_ = create_wall_timer(
      std::chrono::duration<double>(feedback_period_sec_),
      [this]() {registry_->expire_stale(job_ttl_sec_);});

    RCLCPP_INFO(
      get_logger(), "Courier ready with %zu locations and %ld attempt(s) per leg",
      location_names.size(), static_cast<long>(max_attempts_per_leg_));
  }

private:
  template<typename T>
  T required_parameter(const std::string & name) const
  {
    T value{};
    if (!get_parameter(name, value)) {
      throw std::runtime_error("required parameter is missing: " + name);
    }
    return value;
  }

  static std::string join_names(const std::vector<std::string> & names)
  {
    std::ostringstream output;
    for (std::size_t index = 0; index < names.size(); ++index) {
      if (index != 0) {
        output << ", ";
      }
      output << names[index];
    }
    return output.str();
  }

  void handle_request(
    const std::shared_ptr<RequestDelivery::Request> request,
    std::shared_ptr<RequestDelivery::Response> response)
  {
    try {
      if (!location_book_->has(request->pickup)) {
        response->accepted = false;
        response->reason = "unknown pickup location '" + request->pickup +
          "'; known locations: " + join_names(location_book_->names());
        return;
      }
      if (!location_book_->has(request->dropoff)) {
        response->accepted = false;
        response->reason = "unknown dropoff location '" + request->dropoff +
          "'; known locations: " + join_names(location_book_->names());
        return;
      }
      if (request->pickup == request->dropoff) {
        response->accepted = false;
        response->reason = "pickup and dropoff must be different locations";
        return;
      }
      if (max_concurrent_jobs_ == 1) {
        const auto active_id = registry_->active_id();
        if (active_id.has_value()) {
          response->accepted = false;
          response->reason = "robot busy with " + active_id.value();
          return;
        }
      }
      if (queue_depth_ == 0) {
        const auto pending_id = registry_->pending_id();
        if (pending_id.has_value()) {
          response->accepted = false;
          response->reason = "robot busy with " + pending_id.value();
          return;
        }
      }

      response->job_id = registry_->create(request->pickup, request->dropoff);
      response->accepted = true;
      response->reason = "accepted " + response->job_id + " from '" + request->pickup +
        "' to '" + request->dropoff + "'";
      RCLCPP_INFO(get_logger(), "%s", response->reason.c_str());
    } catch (const std::exception & error) {
      response->accepted = false;
      response->job_id.clear();
      response->reason = std::string("request rejected: ") + error.what();
      RCLCPP_ERROR(get_logger(), "%s", response->reason.c_str());
    }
  }

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const Deliver::Goal> goal)
  {
    if (!registry_->claim(goal->job_id)) {
      RCLCPP_WARN(get_logger(), "Rejecting unclaimable job id '%s'", goal->job_id.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }
    cancel_requested_.store(false);
    RCLCPP_INFO(get_logger(), "%s: action goal accepted", goal->job_id.c_str());
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<DeliverGoalHandle> goal_handle)
  {
    cancel_requested_.store(true);
    RCLCPP_INFO(
      get_logger(), "%s: cancellation requested",
      goal_handle->get_goal()->job_id.c_str());
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<DeliverGoalHandle> goal_handle)
  {
    {
      std::lock_guard<std::mutex> lock(feedback_mutex_);
      active_goal_ = goal_handle;
      goal_started_at_ = std::chrono::steady_clock::now();
    }
    std::thread([this, goal_handle]() {execute(goal_handle);}).detach();
  }

  void set_snapshot(
    const std::string & job_id,
    const Leg & leg,
    std::uint16_t attempt,
    const std::string & state)
  {
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    feedback_active_ = true;
    feedback_snapshot_.job_id = job_id;
    feedback_snapshot_.leg = leg.name;
    feedback_snapshot_.target_location = leg.location;
    feedback_snapshot_.attempt = attempt;
    feedback_snapshot_.state = state;
    RCLCPP_INFO(
      get_logger(), "%s: leg=%s target=%s attempt=%u state=%s",
      job_id.c_str(), leg.name.c_str(), leg.location.c_str(), attempt, state.c_str());
  }

  void publish_feedback()
  {
    std::shared_ptr<DeliverGoalHandle> goal_handle;
    FeedbackSnapshot snapshot;
    {
      std::lock_guard<std::mutex> lock(feedback_mutex_);
      if (!feedback_active_) {
        return;
      }
      goal_handle = active_goal_.lock();
      if (!goal_handle) {
        feedback_active_ = false;
        return;
      }
      feedback_snapshot_.elapsed_sec =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - goal_started_at_).count();
      feedback_snapshot_.distance_remaining = nav_client_->distance_remaining();
      snapshot = feedback_snapshot_;
    }

    auto feedback = std::make_shared<Deliver::Feedback>();
    feedback->job_id = snapshot.job_id;
    feedback->leg = snapshot.leg;
    feedback->target_location = snapshot.target_location;
    feedback->distance_remaining = static_cast<float>(snapshot.distance_remaining);
    feedback->attempt = snapshot.attempt;
    feedback->state = snapshot.state;
    feedback->elapsed_sec = static_cast<float>(snapshot.elapsed_sec);
    goal_handle->publish_feedback(feedback);
  }

  bool sleep_polling_cancel(double duration_sec) const
  {
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(duration_sec);
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      if (cancel_requested_.load()) {
        return false;
      }
      std::this_thread::sleep_for(100ms);
    }
    return !cancel_requested_.load();
  }

  void finish_feedback()
  {
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    feedback_active_ = false;
    active_goal_.reset();
  }

  void finish_canceled(
    const std::shared_ptr<DeliverGoalHandle> & goal_handle,
    const std::string & job_id,
    const Leg & leg,
    std::uint16_t attempts_used)
  {
    // The outer action is not marked CANCELED until the robot-facing Nav2 goal
    // has completed its cancellation handshake.
    if (!nav_client_->cancel()) {
      auto result = std::make_shared<Deliver::Result>();
      result->success = false;
      result->outcome = kOutcomeTimeout;
      result->failed_leg = leg.name;
      result->attempts_used = attempts_used;
      result->message =
        "Nav2 cancellation was not confirmed during " + leg.name + " leg";
      goal_handle->abort(result);
      registry_->finish(job_id, kOutcomeTimeout);
      finish_feedback();
      RCLCPP_ERROR(
        get_logger(), "%s: %s", job_id.c_str(), result->message.c_str());
      return;
    }
    auto result = std::make_shared<Deliver::Result>();
    result->success = false;
    result->outcome = kOutcomeCanceled;
    result->failed_leg = leg.name;
    result->attempts_used = attempts_used;
    result->message = "cancelled during " + leg.name + " leg";
    goal_handle->canceled(result);
    registry_->finish(job_id, kOutcomeCanceled);
    finish_feedback();
    RCLCPP_INFO(get_logger(), "%s: CANCELED", job_id.c_str());
  }

  void execute(const std::shared_ptr<DeliverGoalHandle> goal_handle)
  {
    const std::string job_id = goal_handle->get_goal()->job_id;
    const auto job = registry_->get(job_id);
    if (!job.has_value()) {
      auto result = std::make_shared<Deliver::Result>();
      result->success = false;
      result->outcome = kOutcomeInvalidJob;
      result->message = "job disappeared after action acceptance";
      goal_handle->abort(result);
      registry_->finish(job_id, kOutcomeInvalidJob);
      finish_feedback();
      return;
    }

    const std::vector<Leg> legs = {
      {kLegPickup, job->pickup, pickup_dwell_sec_},
      {kLegDropoff, job->dropoff, dropoff_dwell_sec_}};
    std::uint16_t attempts_used = 0;

    for (const auto & leg : legs) {
      bool leg_ok = false;
      NavResult last_result = NavResult::ABORTED;
      std::uint16_t successful_attempt = 0;

      for (std::int64_t attempt_number = 1; attempt_number <= max_attempts_per_leg_;
        ++attempt_number)
      {
        if (cancel_requested_.load()) {
          finish_canceled(goal_handle, job_id, leg, attempts_used);
          return;
        }

        const auto attempt = static_cast<std::uint16_t>(attempt_number);
        auto target_pose = location_book_->pose(leg.location);
        target_pose.header.stamp = now();
        nav_client_->set_target(target_pose);
        set_snapshot(
          job_id, leg, attempt,
          attempt_number == 1 ? kStateNavigating : kStateRetrying);

        if (attempt_number > 1 && clear_costmap_before_retry_) {
          set_snapshot(job_id, leg, attempt, kStateRecovering);
          nav_client_->clear_costmaps();
          if (cancel_requested_.load()) {
            finish_canceled(goal_handle, job_id, leg, attempts_used);
            return;
          }
        }

        ++attempts_used;
        if (!nav_client_->send(target_pose)) {
          last_result = NavResult::ABORTED;
          RCLCPP_WARN(
            get_logger(), "%s: Nav2 rejected %s attempt %u",
            job_id.c_str(), leg.name.c_str(), attempt);
          set_snapshot(job_id, leg, attempt, kStateRetrying);
          if (!sleep_polling_cancel(retry_backoff_sec_)) {
            finish_canceled(goal_handle, job_id, leg, attempts_used);
            return;
          }
          continue;
        }

        set_snapshot(job_id, leg, attempt, kStateNavigating);
        last_result = nav_client_->wait_for_result(
          leg_timeout_sec_, [this]() {return cancel_requested_.load();});
        if (last_result == NavResult::SUCCEEDED) {
          leg_ok = true;
          successful_attempt = attempt;
          break;
        }
        if (last_result == NavResult::CANCELED) {
          finish_canceled(goal_handle, job_id, leg, attempts_used);
          return;
        }

        RCLCPP_WARN(
          get_logger(), "%s: %s attempt %u ended as %s",
          job_id.c_str(), leg.name.c_str(), attempt,
          last_result == NavResult::TIMEOUT ? kOutcomeTimeout : kOutcomeNavAborted);
        set_snapshot(job_id, leg, attempt, kStateRetrying);
        if (!sleep_polling_cancel(retry_backoff_sec_)) {
          finish_canceled(goal_handle, job_id, leg, attempts_used);
          return;
        }
      }

      if (!leg_ok) {
        auto result = std::make_shared<Deliver::Result>();
        result->success = false;
        result->outcome =
          last_result == NavResult::TIMEOUT ? kOutcomeTimeout : kOutcomeNavAborted;
        result->failed_leg = leg.name;
        result->attempts_used = attempts_used;
        result->message = leg.name + " leg to '" + leg.location + "' failed after " +
          std::to_string(max_attempts_per_leg_) + " attempts";
        goal_handle->abort(result);
        registry_->finish(job_id, result->outcome);
        finish_feedback();
        RCLCPP_WARN(get_logger(), "%s: %s", job_id.c_str(), result->message.c_str());
        return;
      }

      set_snapshot(job_id, leg, successful_attempt, kStateDwelling);
      if (!sleep_polling_cancel(leg.dwell_sec)) {
        finish_canceled(goal_handle, job_id, leg, attempts_used);
        return;
      }
      if (cancel_requested_.load()) {
        finish_canceled(goal_handle, job_id, leg, attempts_used);
        return;
      }
    }

    // This is deliberately the only success path: both legs broke out of their
    // retry loops only after Nav2 returned SUCCEEDED.
    auto result = std::make_shared<Deliver::Result>();
    result->success = true;
    result->outcome = kOutcomeSucceeded;
    result->failed_leg.clear();
    result->attempts_used = attempts_used;
    result->message = "pickup and dropoff poses reached";
    goal_handle->succeed(result);
    registry_->finish(job_id, kOutcomeSucceeded);
    finish_feedback();
    RCLCPP_INFO(get_logger(), "%s: SUCCEEDED", job_id.c_str());
  }

  std::unique_ptr<LocationBook> location_book_;
  std::unique_ptr<JobRegistry> registry_;
  std::unique_ptr<NavLegClient> nav_client_;
  rclcpp::CallbackGroup::SharedPtr nav_callback_group_;
  rclcpp::CallbackGroup::SharedPtr server_callback_group_;
  rclcpp::Service<RequestDelivery>::SharedPtr request_service_;
  rclcpp_action::Server<Deliver>::SharedPtr action_server_;
  rclcpp::TimerBase::SharedPtr feedback_timer_;
  rclcpp::TimerBase::SharedPtr expiry_timer_;

  std::int64_t max_attempts_per_leg_;
  double retry_backoff_sec_;
  bool clear_costmap_before_retry_;
  double leg_timeout_sec_;
  double feedback_period_sec_;
  double job_ttl_sec_;
  double pickup_dwell_sec_;
  double dropoff_dwell_sec_;
  std::int64_t max_concurrent_jobs_;
  std::int64_t queue_depth_;
  std::string global_frame_;

  std::atomic<bool> cancel_requested_{false};
  std::mutex feedback_mutex_;
  FeedbackSnapshot feedback_snapshot_;
  std::weak_ptr<DeliverGoalHandle> active_goal_;
  bool feedback_active_{false};
  std::chrono::steady_clock::time_point goal_started_at_;
};

}  // namespace acadbot_courier

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<acadbot_courier::CourierNode>(options);
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("courier_node"), "Startup failed: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
