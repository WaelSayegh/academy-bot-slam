#ifndef ACADBOT_COURIER__NAV_LEG_CLIENT_HPP_
#define ACADBOT_COURIER__NAV_LEG_CLIENT_HPP_

#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/srv/clear_entire_costmap.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "acadbot_courier/types.hpp"

namespace acadbot_courier
{

class NavLegClient
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;
  using ClearEntireCostmap = nav2_msgs::srv::ClearEntireCostmap;

  NavLegClient(
    rclcpp::Node * node,
    const rclcpp::CallbackGroup::SharedPtr & callback_group,
    std::string action_name,
    std::string global_frame,
    std::string robot_base_frame,
    std::string global_costmap_clear_service,
    std::string local_costmap_clear_service,
    double goal_response_timeout_sec,
    double cancel_confirm_timeout_sec,
    double costmap_service_timeout_sec);

  bool send(const geometry_msgs::msg::PoseStamped & goal);
  void set_target(const geometry_msgs::msg::PoseStamped & goal);
  double distance_remaining() const;
  NavResult wait_for_result(
    double timeout_sec, const std::function<bool()> & cancel_requested);
  bool cancel();
  void clear_costmaps();

private:
  void clear_costmap(
    const rclcpp::Client<ClearEntireCostmap>::SharedPtr & client,
    const std::string & service_name);

  rclcpp::Node * node_;
  rclcpp::Logger logger_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
  rclcpp::Client<ClearEntireCostmap>::SharedPtr global_costmap_client_;
  rclcpp::Client<ClearEntireCostmap>::SharedPtr local_costmap_client_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::string global_frame_;
  std::string robot_base_frame_;
  std::string global_costmap_clear_service_;
  std::string local_costmap_clear_service_;
  double goal_response_timeout_sec_;
  double cancel_confirm_timeout_sec_;
  double costmap_service_timeout_sec_;

  mutable std::mutex mutex_;
  GoalHandle::SharedPtr goal_handle_;
  std::optional<std::shared_future<GoalHandle::WrappedResult>> result_future_;
  geometry_msgs::msg::PoseStamped target_pose_;
  std::optional<double> nav_distance_remaining_;
};

}  // namespace acadbot_courier

#endif  // ACADBOT_COURIER__NAV_LEG_CLIENT_HPP_
