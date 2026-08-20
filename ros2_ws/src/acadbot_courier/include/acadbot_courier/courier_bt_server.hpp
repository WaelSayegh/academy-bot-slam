#ifndef ACADBOT_COURIER__COURIER_BT_SERVER_HPP_
#define ACADBOT_COURIER__COURIER_BT_SERVER_HPP_

#include <map>
#include <memory>
#include <string>

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "acadbot_courier_msgs/action/execute_delivery.hpp"
#include "acadbot_courier_msgs/srv/request_delivery.hpp"

#include "acadbot_courier/bt_nodes.hpp"
#include "acadbot_courier/location_table.hpp"
#include "acadbot_courier/types.hpp"

namespace acadbot_courier
{

class CourierBtServer : public rclcpp::Node
{
public:
  CourierBtServer();

private:
  using RequestDelivery = acadbot_courier_msgs::srv::RequestDelivery;
  using ExecuteDelivery = acadbot_courier_msgs::action::ExecuteDelivery;
  using ExecuteGoalHandle = rclcpp_action::ServerGoalHandle<ExecuteDelivery>;
  using NavigateToPose = nav2_msgs::action::NavigateToPose;

  // ---- Service Booking ----------------------------------------------------
  void handle_request(
    const std::shared_ptr<RequestDelivery::Request> request,
    std::shared_ptr<RequestDelivery::Response> response);

  // ---- Action Execution ---------------------------------------------------
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const ExecuteDelivery::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<ExecuteGoalHandle> goal_handle);
  void handle_accepted(const std::shared_ptr<ExecuteGoalHandle> goal_handle);

  // ---- Behavior Tree Execution --------------------------------------------
  void on_bt_tick();

  void finish_succeeded();
  void finish_failed(const std::string & message);
  void finish_canceled();
  void reset_to_idle();

  // ---- Output & Feedback --------------------------------------------------
  void publish_feedback();
  void publish_markers();

  Job & current_job();

  // ---- Configuration ------------------------------------------------------
  std::string goal_frame_{"map"};
  int max_retries_{2};
  double retry_delay_{3.0};
  double feedback_period_{0.5};
  double leg_timeout_{120.0};
  std::string bt_xml_path_;

  LocationTable locations_;

  // ---- ROS Interfaces -----------------------------------------------------
  rclcpp::Service<RequestDelivery>::SharedPtr request_service_;
  rclcpp_action::Server<ExecuteDelivery>::SharedPtr delivery_server_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  rclcpp::TimerBase::SharedPtr feedback_timer_;
  rclcpp::TimerBase::SharedPtr bt_tick_timer_;
  rclcpp::TimerBase::SharedPtr marker_timer_;
  rclcpp::TimerBase::SharedPtr cancel_timer_;

  // ---- Jobs & State -------------------------------------------------------
  std::map<std::string, Job> jobs_;
  unsigned int next_job_number_{0};

  std::shared_ptr<ExecuteGoalHandle> execute_handle_;
  std::string active_job_id_;

  // Shared BT context & feedback status
  std::shared_ptr<LegStatus> leg_status_;
  NavContext nav_context_;
  BT::BehaviorTreeFactory factory_;
  BT::Tree tree_;
  bool tree_running_{false};
};

}  // namespace acadbot_courier

#endif  // ACADBOT_COURIER__COURIER_BT_SERVER_HPP_
