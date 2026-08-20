// The courier: a booking service, a delivery action and a Nav2 client, in a
// node that never blocks -- which is why one thread is enough and no state
// needs a mutex.
#ifndef ACADBOT_COURIER__COURIER_SERVER_HPP_
#define ACADBOT_COURIER__COURIER_SERVER_HPP_

#include <map>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "acadbot_courier_msgs/action/execute_delivery.hpp"
#include "acadbot_courier_msgs/srv/request_delivery.hpp"

#include "acadbot_courier/location_table.hpp"
#include "acadbot_courier/types.hpp"

namespace acadbot_courier
{

class CourierServer : public rclcpp::Node
{
public:
  CourierServer();

private:
  using RequestDelivery = acadbot_courier_msgs::srv::RequestDelivery;
  using ExecuteDelivery = acadbot_courier_msgs::action::ExecuteDelivery;
  using ExecuteGoalHandle = rclcpp_action::ServerGoalHandle<ExecuteDelivery>;
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using NavGoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  /// Mirrors the `state` field the feedback publishes.
  enum class Phase
  {
    NAVIGATING,
    RETRY_WAIT,
    CANCELING,
  };

  /// A CANCELED result from Nav2 is ambiguous, so the reason is recorded when
  /// the cancel is issued rather than guessed at when the result lands.
  enum class CancelReason
  {
    NONE,
    CLIENT_REQUEST,
    LEG_TIMEOUT,
  };

  // ---- booking (service) ----------------------------------------------
  void handle_request(
    const std::shared_ptr<RequestDelivery::Request> request,
    std::shared_ptr<RequestDelivery::Response> response);

  // ---- execution (action server) ---------------------------------------
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const ExecuteDelivery::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<ExecuteGoalHandle> goal_handle);
  void handle_accepted(const std::shared_ptr<ExecuteGoalHandle> goal_handle);

  // ---- the state machine ------------------------------------------------
  void send_leg_goal();
  void on_nav_goal_response(const NavGoalHandle::SharedPtr & handle);
  void on_nav_feedback(
    NavGoalHandle::SharedPtr,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback);
  void on_nav_result(const NavGoalHandle::WrappedResult & result);
  void on_leg_timeout();
  void on_retry_elapsed();

  /// Retries if any attempts remain, otherwise ends the job naming this leg.
  void leg_failed(const std::string & why);

  void finish_succeeded();
  void finish_failed(const std::string & message);
  void finish_canceled();
  void reset_to_idle();

  // ---- output -----------------------------------------------------------
  void publish_feedback();
  void publish_markers();

  // ---- helpers ----------------------------------------------------------
  geometry_msgs::msg::PoseStamped to_goal_pose(const Pose2D & pose) const;
  Job & current_job();

  /// Not named to_string: that would hide the free functions in types.hpp.
  static const char * phase_name(Phase phase);

  // ---- configuration (all from YAML) ------------------------------------
  std::string goal_frame_;
  int max_retries_{2};          // per leg, not per job
  double retry_delay_{3.0};     // s, between attempts on the same leg
  double feedback_period_{0.5}; // s; the action promises at least 1 Hz
  double leg_timeout_{120.0};   // s, before an attempt is abandoned

  LocationTable locations_;

  // ---- interfaces -------------------------------------------------------
  rclcpp::Service<RequestDelivery>::SharedPtr request_service_;
  rclcpp_action::Server<ExecuteDelivery>::SharedPtr delivery_server_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  rclcpp::TimerBase::SharedPtr feedback_timer_;
  rclcpp::TimerBase::SharedPtr retry_timer_;
  rclcpp::TimerBase::SharedPtr leg_timeout_timer_;
  rclcpp::TimerBase::SharedPtr marker_timer_;

  // ---- jobs -------------------------------------------------------------
  /// Every job ever booked, finished ones included, so a replayed id is
  /// refused rather than delivered twice.
  std::map<std::string, Job> jobs_;
  unsigned int next_job_number_{0};

  // ---- the one job in flight --------------------------------------------
  std::shared_ptr<ExecuteGoalHandle> execute_handle_;
  NavGoalHandle::SharedPtr nav_handle_;
  std::string active_job_id_;
  Leg leg_{Leg::PICKUP};
  Phase phase_{Phase::NAVIGATING};
  CancelReason cancel_reason_{CancelReason::NONE};
  int attempt_{0};              // on the current leg, 1-based once sent
  int attempts_total_{0};       // across both legs, reported in the result

  // Republished on our own timer, so a stalled Nav2 shows as a frozen number.
  float last_distance_{0.0f};
  uint16_t last_recoveries_{0};
};

}  // namespace acadbot_courier

#endif  // ACADBOT_COURIER__COURIER_SERVER_HPP_
