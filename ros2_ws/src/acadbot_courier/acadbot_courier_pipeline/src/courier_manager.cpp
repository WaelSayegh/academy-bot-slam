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
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
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

class CourierManager : public rclcpp::Node
{
public:
  CourierManager()
  : Node("courier_manager")
  {
    max_retries_ = declare_parameter<int>("max_retries", 2);
    frame_id_ = declare_parameter<std::string>("frame_id", "map");
    nav2_wait_timeout_ = declare_parameter<double>("nav2_wait_timeout", 10.0);
    leg_goal_timeout_ = declare_parameter<double>("leg_goal_timeout", 10.0);

    auto names = declare_parameter<std::vector<std::string>>("location_names", {});
    auto poses = declare_parameter<std::vector<double>>("location_poses", {});

    if (names.empty() || poses.size() != 3 * names.size()) {
      RCLCPP_FATAL(get_logger(), "location_names/location_poses malformed.");
      throw std::runtime_error("bad location config");
    }

    for (size_t i = 0; i < names.size(); ++i) {
      locations_[names[i]] = Pose2D{poses[3 * i], poses[3 * i + 1], poses[3 * i + 2]};
    }

    service_ = create_service<RequestDelivery>(
      "request_delivery",
      std::bind(&CourierManager::handle_request_delivery, this, _1, _2));

    action_server_ = rclcpp_action::create_server<ExecuteDelivery>(
      this, "execute_delivery",
      std::bind(&CourierManager::handle_goal, this, _1, _2),
      std::bind(&CourierManager::handle_cancel, this, _1),
      std::bind(&CourierManager::handle_accepted, this, _1));

    nav2_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
    startup_timer_ = create_wall_timer(
      1s, std::bind(&CourierManager::check_nav2_ready, this));
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
    if (delivery_active_) {
      response->accepted = false;
      response->reason = "robot is already executing " + active_job_id_;
      return;
    }

    response->accepted = true;
    response->reason = "";
    response->job_id = "job-" + std::to_string(next_job_id_++);
  }

  // action server: the actual drive
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const ExecuteDelivery::Goal> goal)
  {
    (void)uuid;
    if (!nav2_ready_ || delivery_active_) {
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (!locations_.count(goal->pickup) || !locations_.count(goal->dropoff)) {
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<DeliveryGoalHandle> goal_handle)
  {
    (void)goal_handle;
    if (current_nav_goal_handle_) {
      nav2_client_->async_cancel_goal(current_nav_goal_handle_);
    }
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<DeliveryGoalHandle> goal_handle)
  {
    auto goal = goal_handle->get_goal();
    delivery_active_ = true;
    active_job_id_ = goal->job_id;
    job_pickup_ = goal->pickup;
    job_dropoff_ = goal->dropoff;
    current_goal_handle_ = goal_handle;
    current_leg_ = Leg::Pickup;
    current_attempt_ = 1;
    send_nav_goal_for_current_leg();
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
      delivery_active_ = false;
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
        delivery_active_ = false;
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
    delivery_active_ = false;
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

  // startup: don't accept jobs until Nav2 can actually drive
  void check_nav2_ready()
  {
    if (nav2_client_->action_server_is_ready()) {
      nav2_ready_ = true;
      startup_timer_->cancel();
      RCLCPP_INFO(get_logger(), "Nav2 'navigate_to_pose' action server is ready.");
      return;
    }
    nav2_wait_elapsed_ += 1.0;
    if (nav2_wait_elapsed_ >= nav2_wait_timeout_) {
      RCLCPP_WARN(get_logger(),
        "Still waiting for Nav2 after %.0fs — is autonomy.launch.py running?",
        nav2_wait_elapsed_);
    } else {
      RCLCPP_INFO(get_logger(), "Waiting for Nav2 'navigate_to_pose' action server...");
    }
  }

  rclcpp::Service<RequestDelivery>::SharedPtr service_;
  rclcpp_action::Server<ExecuteDelivery>::SharedPtr action_server_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav2_client_;
  rclcpp::TimerBase::SharedPtr startup_timer_;
  rclcpp::TimerBase::SharedPtr leg_timeout_timer_;

  std::unordered_map<std::string, Pose2D> locations_;
  std::string frame_id_;
  int max_retries_;
  double nav2_wait_timeout_;
  double nav2_wait_elapsed_{0.0};
  double leg_goal_timeout_;
  bool nav2_ready_{false};

  bool delivery_active_{false};
  std::string active_job_id_;
  int next_job_id_{1};

  // current job state, valid only while delivery_active_ is true
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
  rclcpp::spin(std::make_shared<CourierManager>());
  rclcpp::shutdown();
  return 0;
}
