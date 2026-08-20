// Courier-specific BehaviorTree.CPP nodes for the acadbot courier bonus track.
//
// DriveToLocation  - the one custom leaf: sends a Nav2 navigate_to_pose goal,
//                 streams feedback and honours cancellation. The target comes
//                 from the `pose` port ("x,y,yaw" in frame_id) or, failing
//                 that, from a named location in CourierNavContext::locations.
//
// Everything else is expressed in courier_bt.xml with built-in BT.CPP nodes:
//   RetryUntilSuccessful num_attempts="{retry_count}"  - retry budget per leg
//   Fallback + Timeout msec="{leg_timeout_msec}"       - per-attempt leg timeout
//   Fallback + Delay delay_msec="{retry_delay_msec}"   - pause between attempts
// Navigation-level recovery (spin/backup/wait) is owned by Nav2 through the
// bt_navigator `default_bt_xml` recovery tree, so the mission tree has no
// recovery leaves; a failing leg simply retries, then gives up.
//
// The tree is loaded from courier_bt.xml at runtime, so it can be edited
// without recompiling. Wire these up in courier_server.cpp like this:
//
//     auto context = std::make_shared<acadbot_courier::CourierNavContext>();
//     context->node = shared_from_this();
//     context->frame_id = frame_id_;
//     context->locations = /* map<std::string, NavLocation> */;
//     context->is_cancelled = [this]() { return cancelling_.load(); };
//     context->on_feedback = [this](const std::string& leg,
//                                   const std::string& target,
//                                   float distance,
//                                   const std::string& note) {
//       std::lock_guard<std::mutex> lock(state_mutex_);
//       fb_leg_ = leg; fb_target_ = target; fb_distance_ = distance; fb_note_ = note;
//     };
//
//     BT::BehaviorTreeFactory factory;
//     registerCourierNodes(factory, context);
//     auto blackboard = BT::Blackboard::create();
//     blackboard->set("pickup_name", job.pickup);
//     blackboard->set("dropoff_name", job.dropoff);
//     blackboard->set("pickup_pose", "0.6,4.2,0.0");
//     blackboard->set("dropoff_pose", "5.5,0.6,0.0");
//     blackboard->set("retry_count", retry_count_);
//     blackboard->set("retry_delay_msec",
//                     static_cast<unsigned int>(retry_delay_sec_ * 1000));
//     blackboard->set("leg_timeout_msec",
//                     static_cast<unsigned int>(leg_timeout_sec_ * 1000));
//     blackboard->set("attempts", 0);
//     blackboard->set("failed_leg", "");
//     auto tree = factory.createTreeFromFile(bt_file_path, blackboard);
//
// After tickOnce() returns, read blackboard->get("attempts", ...) and
// blackboard->get("failed_leg", ...) for the honest-failure result.

#ifndef ACADBOT_COURIER__COURIER_BT_NODES_HPP_
#define ACADBOT_COURIER__COURIER_BT_NODES_HPP_

#include <atomic>
#include <chrono>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2/LinearMath/Quaternion.h"

namespace acadbot_courier {

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

struct NavLocation {
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
};

struct CourierNavContext {
  rclcpp::Node::SharedPtr node;
  std::string frame_id{"map"};
  std::unordered_map<std::string, NavLocation> locations;
  std::function<bool()> is_cancelled{[]() { return false; }};
  std::function<void(const std::string & leg, const std::string & target,
                     float distance, const std::string & note)> on_feedback;
};

inline const char * bt_result_name(rclcpp_action::ResultCode code) {
  switch (code) {
    case rclcpp_action::ResultCode::SUCCEEDED: return "SUCCEEDED";
    case rclcpp_action::ResultCode::CANCELED: return "CANCELED";
    case rclcpp_action::ResultCode::ABORTED: return "ABORTED";
    default: return "UNKNOWN";
  }
}

class DriveToLocation : public BT::StatefulActionNode {
public:
  DriveToLocation(const std::string & name, const BT::NodeConfig & config,
               std::shared_ptr<CourierNavContext> context)
    : BT::StatefulActionNode(name, config), context_(std::move(context)) {}

  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<std::string>("pose"),          // "x,y,yaw" in frame_id
      BT::InputPort<std::string>("location_name"), // label for logs/feedback
      BT::InputPort<std::string>("leg"),
    };
  }

  BT::NodeStatus onStart() override {
    ++attempt_;
    if (!getInput("leg", leg_)) { return BT::NodeStatus::FAILURE; }
    getInput("location_name", location_);

    std::string pose_str;
    if (getInput("pose", pose_str)) {
      if (!parse_pose(pose_str, target_)) {
        RCLCPP_ERROR(context_->node->get_logger(),
                     "BT: bad pose '%s' for %s leg", pose_str.c_str(), leg_.c_str());
        return BT::NodeStatus::FAILURE;
      }
    } else {
      auto it = context_->locations.find(location_);
      if (it == context_->locations.end()) {
        RCLCPP_ERROR(context_->node->get_logger(),
                     "BT: unknown location '%s'", location_.c_str());
        return BT::NodeStatus::FAILURE;
      }
      target_ = it->second;
    }

    max_attempts_ = 0;
    static_cast<void>(config().blackboard->get("retry_count", max_attempts_));

    int total_attempts = 0;
    static_cast<void>(config().blackboard->get("attempts", total_attempts));
    config().blackboard->set("attempts", total_attempts + 1);

    if (!nav_client_) {
      nav_client_ = rclcpp_action::create_client<NavigateToPose>(
          context_->node.get(), "navigate_to_pose");
      if (!nav_client_->wait_for_action_server(std::chrono::seconds(10))) {
        RCLCPP_ERROR(context_->node->get_logger(),
                     "BT: Nav2 navigate_to_pose server not available");
        return BT::NodeStatus::FAILURE;
      }
    }

    NavigateToPose::Goal goal;
    goal.pose.header.frame_id = context_->frame_id;
    goal.pose.header.stamp = context_->node->now();
    goal.pose.pose.position.x = target_.x;
    goal.pose.pose.position.y = target_.y;
    tf2::Quaternion q;
    q.setRPY(0, 0, target_.yaw);
    goal.pose.pose.orientation.x = q.x();
    goal.pose.pose.orientation.y = q.y();
    goal.pose.pose.orientation.z = q.z();
    goal.pose.pose.orientation.w = q.w();

    RCLCPP_INFO(context_->node->get_logger(),
                "BT: %s leg -> %s (%.2f, %.2f) attempt %d",
                leg_.c_str(), location_.c_str(), target_.x, target_.y, attempt_);

    cancelled_.store(false);
    result_promise_ = std::make_shared<std::promise<BT::NodeStatus>>();
    result_future_ = result_promise_->get_future();
    worker_ = std::thread([this, goal]() {
      result_promise_->set_value(run_goal(goal));
    });
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override {
    if (result_future_.wait_for(std::chrono::milliseconds(50)) !=
        std::future_status::ready) {
      return BT::NodeStatus::RUNNING;
    }
    BT::NodeStatus status = result_future_.get();
    if (worker_.joinable()) { worker_.join(); }
    if (status == BT::NodeStatus::FAILURE) {
      config().blackboard->set("failed_leg", leg_);
    }
    return status;
  }

  void onHalted() override {
    cancelled_.store(true);
    if (worker_.joinable()) { worker_.join(); }
  }

private:
  static bool parse_pose(const std::string & s, NavLocation & out) {
    char comma;
    std::stringstream ss(s);
    if (!(ss >> out.x >> comma >> out.y >> comma >> out.yaw) ||
        comma != ',') {
      return false;
    }
    return true;
  }

  BT::NodeStatus run_goal(const NavigateToPose::Goal & goal) {
    rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
    options.feedback_callback =
        [this](GoalHandleNav::SharedPtr,
               const std::shared_ptr<const NavigateToPose::Feedback> feedback) {
          if (context_->on_feedback) {
            context_->on_feedback(leg_, location_, feedback->distance_remaining,
                                  "attempt " + std::to_string(attempt_) +
                                      " of " + std::to_string(max_attempts_));
          }
        };

    auto gh_future = nav_client_->async_send_goal(goal, options);
    if (gh_future.wait_for(std::chrono::seconds(10)) != std::future_status::ready) {
      RCLCPP_WARN(context_->node->get_logger(),
                  "BT: navigate_to_pose goal not acknowledged in 10s");
      return BT::NodeStatus::FAILURE;
    }
    auto goal_handle = gh_future.get();
    if (!goal_handle) {
      RCLCPP_WARN(context_->node->get_logger(),
                  "BT: navigate_to_pose goal rejected");
      return BT::NodeStatus::FAILURE;
    }

    auto result_future = nav_client_->async_get_result(goal_handle);
    while (result_future.wait_for(std::chrono::milliseconds(250)) !=
           std::future_status::ready) {
      if (cancelled_.load() ||
          (context_->is_cancelled && context_->is_cancelled())) {
        nav_client_->async_cancel_goal(goal_handle);
        RCLCPP_INFO(context_->node->get_logger(),
                    "BT: %s leg cancelled, aborting", leg_.c_str());
        return BT::NodeStatus::FAILURE;
      }
    }

    auto wrapped = result_future.get();
    RCLCPP_INFO(context_->node->get_logger(), "BT: %s leg -> %s finished: %s",
                leg_.c_str(), location_.c_str(), bt_result_name(wrapped.code));
    return wrapped.code == rclcpp_action::ResultCode::SUCCEEDED
               ? BT::NodeStatus::SUCCESS
               : BT::NodeStatus::FAILURE;
  }

  std::shared_ptr<CourierNavContext> context_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  std::shared_ptr<std::promise<BT::NodeStatus>> result_promise_;
  std::future<BT::NodeStatus> result_future_;
  std::thread worker_;
  std::atomic<bool> cancelled_{false};
  std::string location_;
  std::string leg_;
  NavLocation target_;
  int attempt_{0};
  int max_attempts_{0};
};

inline void registerCourierNodes(BT::BehaviorTreeFactory & factory,
                                 const std::shared_ptr<CourierNavContext> & context) {
  factory.registerBuilder<DriveToLocation>(
      "DriveToLocation",
      [context](const std::string & name, const BT::NodeConfig & config) {
        return std::make_unique<DriveToLocation>(name, config, context);
      });
}

}  // namespace acadbot_courier

#endif  // ACADBOT_COURIER__COURIER_BT_NODES_HPP_