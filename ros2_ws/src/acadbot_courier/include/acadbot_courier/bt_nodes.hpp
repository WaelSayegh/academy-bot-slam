// GoToLocation, the one custom leaf in courier.xml, written as a
// StatefulActionNode because a tick that waited for Nav2 would block the
// thread that has to deliver the result it waits for.
#ifndef ACADBOT_COURIER__BT_NODES_HPP_
#define ACADBOT_COURIER__BT_NODES_HPP_

#include <memory>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/bt_factory.h"

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "acadbot_courier/types.hpp"

namespace acadbot_courier
{

/// Lives beside the tree rather than in the leaf, because between attempts no
/// leaf is running and the feedback still has to flow.
struct LegStatus
{
  Leg leg{Leg::PICKUP};
  std::string target_location;
  std::string state{"NAVIGATING"};   ///< NAVIGATING / RETRYING / CANCELING
  float distance_remaining{0.0f};
  uint16_t nav2_recoveries{0};
  int attempt{0};                    ///< on this leg, 1-based
  int attempts_total{0};             ///< across both legs, for the result
};

/// What a GoToLocation needs from the node that owns the tree.
struct NavContext
{
  rclcpp::Node * node{nullptr};
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_client;
  std::shared_ptr<LegStatus> status;
  std::string goal_frame{"map"};
};

/// Drive to one pose, knowing nothing about retries, timeouts or leg order --
/// those are the decorators wrapped around it in courier.xml.
class GoToLocation : public BT::StatefulActionNode
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using NavGoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  GoToLocation(
    const std::string & name, const BT::NodeConfig & config, const NavContext & context);

  /// A pose and not a name, because a job's targets are frozen at booking.
  static BT::PortsList providedPorts();

private:
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

  void on_goal_response(const NavGoalHandle::SharedPtr & handle);
  void on_feedback(
    NavGoalHandle::SharedPtr,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback);
  void on_result(const NavGoalHandle::WrappedResult & result);

  NavContext context_;

  NavGoalHandle::SharedPtr nav_handle_;

  /// Set by the Nav2 callbacks; `outcome_` means nothing until `finished_`.
  bool finished_{false};
  bool outcome_{false};

  /// Set on halt, because a halt can land before Nav2 has acknowledged the
  /// goal and left no handle to cancel -- without this the robot drives on.
  bool halted_{false};

  /// Expires with this leaf, so a Nav2 callback still in flight when the next
  /// job rebuilds the tree gives up instead of writing into freed memory.
  std::shared_ptr<bool> alive_{std::make_shared<bool>(true)};
};

/// Registers GoToLocation with the factory, once, before the first tree.
void register_courier_nodes(BT::BehaviorTreeFactory & factory, const NavContext & context);

}  // namespace acadbot_courier

#endif  // ACADBOT_COURIER__BT_NODES_HPP_
