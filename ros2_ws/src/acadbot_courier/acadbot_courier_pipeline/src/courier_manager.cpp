// Copyright 2026 Hiba Tarabay
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "acadbot_courier_interfaces/srv/request_delivery.hpp"
#include "acadbot_courier_interfaces/action/execute_delivery.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

using RequestDelivery = acadbot_courier_interfaces::srv::RequestDelivery;
using ExecuteDelivery = acadbot_courier_interfaces::action::ExecuteDelivery;
using DeliveryGoalHandle = rclcpp_action::ServerGoalHandle<ExecuteDelivery>;
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using NavGoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

struct Pose2D { double x, y, yaw; };
enum class Leg { Pickup, Dropoff };

class CourierManager : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CourierManager()
  : LifecycleNode("courier_manager")
  {
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    max_retries_ = declare_parameter<int>("max_retries", 2);
    frame_id_ = declare_parameter<std::string>("frame_id", "map");
    leg_goal_timeout_ = declare_parameter<double>("leg_goal_timeout", 10.0);
    max_queue_size_ = declare_parameter<int>("max_queue_size", 5);

    auto names = declare_parameter<std::vector<std::string>>("location_names", {});
    auto poses = declare_parameter<std::vector<double>>("location_poses", {});

    if (names.empty() || poses.size() != 3 * names.size()) {
      RCLCPP_ERROR(get_logger(), "location_names/location_poses malformed.");
      return CallbackReturn::FAILURE;
    }

    locations_.clear();
    for (size_t i = 0; i < names.size(); ++i) {
      locations_[names[i]] = Pose2D{poses[3 * i], poses[3 * i + 1], poses[3 * i + 2]};
    }

    nav2_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    service_ = create_service<RequestDelivery>(
      "request_delivery",
      std::bind(&CourierManager::handle_request_delivery, this, _1, _2));

    action_server_ = rclcpp_action::create_server<ExecuteDelivery>(
      this, "execute_delivery",
      std::bind(&CourierManager::handle_goal, this, _1, _2),
      std::bind(&CourierManager::handle_cancel, this, _1),
      std::bind(&CourierManager::handle_accepted, this, _1));

    queued_feedback_timer_ = create_wall_timer(
      1s, std::bind(&CourierManager::publish_queued_feedback, this));

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    // Hard-stop: cancels whatever job is currently active, then drains
    // every job still waiting in queue_ (those never got a Nav2 goal, so
    // there's nothing to cancel with Nav2 — finalize them directly).
    if (current_nav_goal_handle_) {
      nav2_client_->async_cancel_goal(current_nav_goal_handle_);
    }
    if (leg_timeout_timer_) {
      leg_timeout_timer_->cancel();
    }
    if (queued_feedback_timer_) {
      queued_feedback_timer_->cancel();
    }

    for (auto & queued_goal_handle : queue_) {
      if (queued_goal_handle == current_goal_handle_) {
        continue;
      }
      auto res = std::make_shared<ExecuteDelivery::Result>();
      res->success = false;
      res->failed_leg = "pickup";
      res->message = "node deactivating";
      // A queued goal here was never asked to cancel, so it's unconditionally
      // EXECUTING (see finalize_canceling_queued_goals()'s comment) unless a
      // cancel request happened to land on it moments before deactivate --
      // in that case it may already be CANCELING. canceled() is only valid
      // from CANCELING and abort() is only valid from EXECUTING, so branch
      // on the goal's actual state instead of assuming one or the other;
      // either way the client gets a terminal success=false result, not a
      // hang, without this loop ever throwing mid-drain.
      if (queued_goal_handle->is_canceling()) {
        queued_goal_handle->canceled(res);
      } else {
        queued_goal_handle->abort(res);
      }
    }
    queue_.clear();
    pending_count_ = 0;

    service_.reset();
    action_server_.reset();

    // Bump the attempt token so any Nav2 callback still in flight from the
    // canceled goal (goal_response/feedback/result) sees a stale token and
    // no-ops instead of touching current_goal_handle_/current_nav_goal_handle_
    // after they're cleared below — the same guard send_nav_goal_for_current_leg
    // relies on, just invalidated from outside instead of from a new attempt.
    ++attempt_token_;
    current_goal_handle_.reset();
    current_nav_goal_handle_.reset();

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
  {
    nav2_client_.reset();
    return CallbackReturn::SUCCESS;
  }

private:
  // service: admission control only, never drives
  void handle_request_delivery(
    const std::shared_ptr<RequestDelivery::Request> request,
    std::shared_ptr<RequestDelivery::Response> response)
  {
    if (!locations_.count(request->pickup)) {
      response->accepted = false;
      response->reason = "unknown pickup location: " + request->pickup;
      return;
    }
    if (!locations_.count(request->dropoff)) {
      response->accepted = false;
      response->reason = "unknown dropoff location: " + request->dropoff;
      return;
    }
    if (pending_count_ >= max_queue_size_) {
      response->accepted = false;
      response->reason = "queue full (max " + std::to_string(max_queue_size_) + ")";
      return;
    }

    response->accepted = true;
    response->reason = "";
    response->job_id = "job-" + std::to_string(next_job_id_++);
    ++pending_count_;
  }

  // action server: the actual drive
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const ExecuteDelivery::Goal> goal)
  {
    (void)uuid;
    if (!locations_.count(goal->pickup) || !locations_.count(goal->dropoff)) {
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<DeliveryGoalHandle> goal_handle)
  {
    if (goal_handle == current_goal_handle_) {
      // Actively driving: forward to Nav2 and let handle_nav_result's
      // is_canceling() branch finish it and run the queue-runner.
      if (current_nav_goal_handle_) {
        nav2_client_->async_cancel_goal(current_nav_goal_handle_);
      }
      return rclcpp_action::CancelResponse::ACCEPT;
    }

    // A goal still sitting in queue_ is unconditionally EXECUTING (rclcpp_
    // action transitions ACCEPT_AND_EXECUTE goals to EXECUTING immediately
    // on acceptance, whether or not send_nav_goal_for_current_leg() has
    // ever run for it) -- canceled() is only valid once the goal is
    // CANCELING. That transition only happens via the framework's own
    // _cancel_goal() call, which runs right after this callback returns
    // ACCEPT, not before. So we must not finalize here: just accept, and
    // let finalize_canceling_queued_goals() (polled from the ~1Hz
    // publish_queued_feedback timer) notice is_canceling() and finalize it
    // once that transition has actually happened.
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<DeliveryGoalHandle> goal_handle)
  {
    queue_.push_back(goal_handle);
    if (queue_.size() == 1) {
      start_active_job(goal_handle);
    }
  }

  // Promotes goal_handle to "currently active": populates the job-state
  // members and kicks off the first leg. Used both for a freshly accepted
  // goal that finds the queue idle, and for the new front of queue_ once
  // the previous active job finishes.
  void start_active_job(const std::shared_ptr<DeliveryGoalHandle> & goal_handle)
  {
    auto goal = goal_handle->get_goal();
    active_job_id_ = goal->job_id;
    job_pickup_ = goal->pickup;
    job_dropoff_ = goal->dropoff;
    current_goal_handle_ = goal_handle;
    current_leg_ = Leg::Pickup;
    current_attempt_ = 1;
    send_nav_goal_for_current_leg();
  }

  // Called from the three places an active job reaches a terminal state
  // (succeed()/canceled() in handle_nav_result, abort() in
  // on_leg_attempt_failed) after the goal handle has already been
  // finalized. Pops it off the queue, releases its admission-control slot,
  // and starts the next queued job if there is one.
  void finish_current_and_advance_queue()
  {
    if (!queue_.empty()) {
      queue_.pop_front();
    }
    if (pending_count_ > 0) {
      --pending_count_;
    }
    if (!queue_.empty()) {
      start_active_job(queue_.front());
    } else {
      current_goal_handle_.reset();
      current_nav_goal_handle_.reset();
    }
  }

  // Manufactures "queued" feedback for every goal in queue_ except the
  // active front, which still gets its feedback from Nav2's own callback.
  void publish_queued_feedback()
  {
    finalize_canceling_queued_goals();

    uint32_t position = 0;
    for (const auto & queued_goal_handle : queue_) {
      if (queued_goal_handle == current_goal_handle_) {
        continue;
      }
      ++position;
      auto goal = queued_goal_handle->get_goal();
      auto fb = std::make_shared<ExecuteDelivery::Feedback>();
      fb->leg = "queued";
      fb->target = goal->pickup;
      fb->distance_left = 0.0f;
      fb->attempt = 0;
      fb->queue_position = position;
      queued_goal_handle->publish_feedback(fb);
    }
  }

  // Removes and finalizes every still-queued goal (never the active front,
  // which is finalized by handle_nav_result's own is_canceling() branch
  // instead) whose cancel request the framework has, since the last tick,
  // actually transitioned to CANCELING via its own _cancel_goal() call
  // (triggered right after handle_cancel() returns ACCEPT for it).
  // canceled() throws unless the goal is already CANCELING, so this poll is
  // what makes it safe to call -- see handle_cancel() for why finalizing
  // inline there is not an option.
  void finalize_canceling_queued_goals()
  {
    for (auto it = queue_.begin(); it != queue_.end(); ) {
      if (*it == current_goal_handle_ || !(*it)->is_canceling()) {
        ++it;
        continue;
      }
      auto res = std::make_shared<ExecuteDelivery::Result>();
      res->success = false;
      res->failed_leg = "pickup";
      res->message = "canceled while queued";
      (*it)->canceled(res);
      it = queue_.erase(it);
      if (pending_count_ > 0) {
        --pending_count_;
      }
    }
  }

  // vav2 client: one leg at a time
  void send_nav_goal_for_current_leg()
  {
    const std::string & target = (current_leg_ == Leg::Pickup) ? job_pickup_ : job_dropoff_;
    const Pose2D & pose = locations_.at(target);

    NavigateToPose::Goal goal;
    goal.pose.header.frame_id = frame_id_;
    goal.pose.header.stamp = now();
    goal.pose.pose.position.x = pose.x;
    goal.pose.pose.position.y = pose.y;

    tf2::Quaternion q;
    q.setRPY(0, 0, pose.yaw);
    goal.pose.pose.orientation.x = q.x();
    goal.pose.pose.orientation.y = q.y();
    goal.pose.pose.orientation.z = q.z();
    goal.pose.pose.orientation.w = q.w();

    RCLCPP_INFO(get_logger(), "[%s] leg=%s attempt=%u target=%s (%.2f, %.2f)",
                active_job_id_.c_str(),
                current_leg_ == Leg::Pickup ? "to_pickup" : "to_dropoff",
                current_attempt_, target.c_str(), pose.x, pose.y);

    // bound this single attempt Nav2 aborting is already handled via the
    // result callback below; this catches the attempt just never resolving
    if (leg_timeout_timer_) {
      leg_timeout_timer_->cancel();
    }
    // Tags this attempt so late/stale callbacks from a superseded goal
    // (e.g. a cancel that Nav2 hadn't actually honored yet) can be told
    // apart from the attempt we're currently tracking and ignored.
    const uint64_t token = ++attempt_token_;
    leg_timeout_timer_ = create_wall_timer(
      std::chrono::duration<double>(leg_goal_timeout_),
      [this, token]() {on_leg_timeout(token);});

    rclcpp_action::Client<NavigateToPose>::SendGoalOptions opts;
    opts.goal_response_callback =
      [this, token](NavGoalHandle::SharedPtr gh) {
        if (token != attempt_token_) {
          return;
        }
        current_nav_goal_handle_ = gh;
        if (!gh) {
          RCLCPP_WARN(get_logger(), "Nav2 rejected the goal outright.");
          on_leg_attempt_failed();
        }
      };
    opts.feedback_callback =
      [this, token](NavGoalHandle::SharedPtr,
      const std::shared_ptr<const NavigateToPose::Feedback> fb) {
        if (token != attempt_token_) {
          return;
        }
        auto courier_fb = std::make_shared<ExecuteDelivery::Feedback>();
        courier_fb->leg = (current_leg_ == Leg::Pickup) ? "to_pickup" : "to_dropoff";
        courier_fb->target = (current_leg_ == Leg::Pickup) ? job_pickup_ : job_dropoff_;
        courier_fb->distance_left = fb->distance_remaining;
        courier_fb->attempt = current_attempt_;
        current_goal_handle_->publish_feedback(courier_fb);
      };
    opts.result_callback =
      [this, token](const NavGoalHandle::WrappedResult & result) {
        if (token != attempt_token_) {
          return;
        }
        handle_nav_result(result);
      };

    nav2_client_->async_send_goal(goal, opts);
  }

  void handle_nav_result(const NavGoalHandle::WrappedResult & result)
  {
    // A real result arrived the per-attempt timeout no longer applies.
    if (leg_timeout_timer_) {
      leg_timeout_timer_->cancel();
    }

    // cancel takes priority over whatever Nav2's result code says
    if (current_goal_handle_->is_canceling()) {
      auto res = std::make_shared<ExecuteDelivery::Result>();
      res->success = false;
      res->failed_leg = (current_leg_ == Leg::Pickup) ? "pickup" : "dropoff";
      res->message = "canceled by requester";
      current_goal_handle_->canceled(res);
      finish_current_and_advance_queue();
      return;
    }

    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      if (current_leg_ == Leg::Pickup) {
        RCLCPP_INFO(get_logger(), "[%s] pickup leg complete.", active_job_id_.c_str());
        current_leg_ = Leg::Dropoff;
        current_attempt_ = 1;
        send_nav_goal_for_current_leg();
      } else {
        RCLCPP_INFO(get_logger(), "[%s] delivery complete.", active_job_id_.c_str());
        auto res = std::make_shared<ExecuteDelivery::Result>();
        res->success = true;
        res->failed_leg = "";
        res->message = "delivery complete";
        current_goal_handle_->succeed(res);
        finish_current_and_advance_queue();
      }
      return;
    }

    // ABORTED, unexpected CANCELED from Nav2 itself, or any other non-success code.
    on_leg_attempt_failed();
  }

  void on_leg_attempt_failed()
  {
    if (current_attempt_ <= static_cast<uint32_t>(max_retries_)) {
      RCLCPP_WARN(get_logger(), "[%s] leg attempt %u failed, retrying...",
                  active_job_id_.c_str(), current_attempt_);
      current_attempt_++;
      send_nav_goal_for_current_leg();
      return;
    }
    auto res = std::make_shared<ExecuteDelivery::Result>();
    res->success = false;
    res->failed_leg = (current_leg_ == Leg::Pickup) ? "pickup" : "dropoff";
    res->message = "Nav2 exhausted recoveries after " +
      std::to_string(max_retries_ + 1) + " attempts";
    current_goal_handle_->abort(res);
    finish_current_and_advance_queue();
  }

  // Fires if a single navigate_to_pose attempt neither succeeds nor gets
  // ABORTED by Nav2 within leg_goal_timeout_ — e.g. an unbounded recovery
  // loop. Treated the same as an ABORTED attempt (req 7).
  void on_leg_timeout(uint64_t token)
  {
    if (token != attempt_token_) {
      return;  // stale timer for an attempt we've already moved past
    }
    leg_timeout_timer_->cancel();

    // user-requested cancel is already in flight let handle_nav_result's
    // CANCELED branch resolve it instead of also retrying/aborting here.
    if (current_goal_handle_->is_canceling()) {
      return;
    }

    RCLCPP_WARN(get_logger(), "[%s] leg attempt %u timed out after %.1fs — canceling Nav2 goal.",
                active_job_id_.c_str(), current_attempt_, leg_goal_timeout_);
    if (current_nav_goal_handle_) {
      nav2_client_->async_cancel_goal(current_nav_goal_handle_);
    }
    on_leg_attempt_failed();
  }

  rclcpp::Service<RequestDelivery>::SharedPtr service_;
  rclcpp_action::Server<ExecuteDelivery>::SharedPtr action_server_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav2_client_;
  rclcpp::TimerBase::SharedPtr leg_timeout_timer_;
  rclcpp::TimerBase::SharedPtr queued_feedback_timer_;

  std::unordered_map<std::string, Pose2D> locations_;
  std::string frame_id_;
  int max_retries_;
  double leg_goal_timeout_;
  int max_queue_size_;

  // FIFO of every goal handle currently accepted: queue_.front() is the
  // active job (if any), the rest are waiting their turn.
  std::deque<std::shared_ptr<DeliveryGoalHandle>> queue_;
  // Admission-control counter, deliberately NOT queue_.size(): it's
  // incremented the moment RequestDelivery hands out a job_id and only
  // decremented when that job's ExecuteDelivery goal reaches a terminal
  // state. A job can be "pending" (id issued) before ExecuteDelivery is
  // ever called for it, so pending_count_ can exceed queue_.size() — see
  // the documented queue-slot-leak limitation in the task notes.
  int pending_count_{0};
  std::string active_job_id_;
  int next_job_id_{1};

  // current job state, valid only while current_goal_handle_ is set
  std::shared_ptr<DeliveryGoalHandle> current_goal_handle_;
  NavGoalHandle::SharedPtr current_nav_goal_handle_;
  std::string job_pickup_, job_dropoff_;
  Leg current_leg_{Leg::Pickup};
  uint32_t current_attempt_{1};
  uint64_t attempt_token_{0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CourierManager>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
