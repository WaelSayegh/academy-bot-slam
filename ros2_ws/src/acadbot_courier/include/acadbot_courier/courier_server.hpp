#ifndef ACADBOT_COURIER__COURIER_SERVER_HPP_
#define ACADBOT_COURIER__COURIER_SERVER_HPP_

#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <cstdint>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "acadbot_courier_interfaces/srv/submit_delivery.hpp"
#include "acadbot_courier/navigation_manager.hpp"
#include "acadbot_courier/types.hpp"

using SubmitDelivery =
  acadbot_courier_interfaces::srv::SubmitDelivery;

class CourierServer : public rclcpp::Node{
public:
  // ---------------------------------------------------------------------------
  // Handle a new delivery submission request.
  //
  // A request is accepted only when the courier is idle and both the pickup and
  // dropoff names exist in the configured location map. Accepted requests are
  // assigned a unique job ID and stored as the current reserved job.
  //
  // Invalid locations or requests received while the courier is busy are
  // rejected immediately with a human-readable reason.
  // ---------------------------------------------------------------------------
  CourierServer();

private:
  // ---------------------------------------------------------------------------
  // Handle a new delivery submission request.
  //
  // A request is accepted only when the courier is idle and both the pickup and
  // dropoff names exist in the configured location map. Accepted requests are
  // assigned a unique job ID and stored as the current reserved job.
  //
  // Invalid locations or requests received while the courier is busy are
  // rejected immediately with a human-readable reason.
  // ---------------------------------------------------------------------------
  void handle_submit_delivery(
    const std::shared_ptr<SubmitDelivery::Request> request,
    std::shared_ptr<SubmitDelivery::Response> response);

  // ---------------------------------------------------------------------------
  // Validate a request to execute an accepted delivery job.
  //
  // The action goal is accepted only when the courier has a reserved job and the
  // supplied job ID matches that reservation. This prevents callers from
  // executing unknown, stale, or unrelated jobs.
  // ---------------------------------------------------------------------------
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const ExecuteDelivery::Goal> goal);

  // ---------------------------------------------------------------------------
  // Accept cancellation requests for the active courier action.
  //
  // The execution thread detects the cancellation request and propagates it to
  // the currently active Nav2 NavigateToPose goal.
  // ---------------------------------------------------------------------------
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<CourierGoalHandle> goal_handle);

  // ---------------------------------------------------------------------------
  // Start execution of an accepted delivery action goal.
  //
  // Delivery execution is moved to a separate thread so the ROS executor remains
  // free to process service requests, action feedback, cancellation requests,
  // and later Nav2 callbacks while the mission is running.
  // ---------------------------------------------------------------------------
  void handle_accepted(const std::shared_ptr<CourierGoalHandle> goal_handle);

  // ---------------------------------------------------------------------------
  // Execute the currently reserved courier job.
  //
  // The courier performs two sequential Nav2 navigation legs:
  //   1. Navigate to the configured pickup location.
  //   2. After pickup succeeds, navigate to the configured dropoff location.
  //
  // The dropoff leg is never started unless the pickup leg actually succeeds.
  // Any Nav2 abort or cancellation is propagated honestly to the courier action
  // and identifies which delivery leg failed.
  // ---------------------------------------------------------------------------
  void execute_delivery(
    const std::shared_ptr<CourierGoalHandle> goal_handle);

  // ---------------------------------------------------------------------------
  // Generate the next process-local delivery job identifier.
  //
  // Job IDs use a simple sequential format (job_0001, job_0002, ...). They only
  // need to remain unique during the current courier_server execution; persistent
  // identifiers across robot restarts are outside the scope of this prototype.
  // ---------------------------------------------------------------------------
  std::string generate_job_id();

  // ---------------------------------------------------------------------------
  // Clear the current delivery and return the courier to the idle state.
  //
  // This is called whenever a job reaches a terminal state so the robot becomes
  // available to accept a new delivery request.
  // ---------------------------------------------------------------------------
  void reset_job();

  // ---------------------------------------------------------------------------
  // Validate courier configuration limits.
  //
  // Rejects invalid values at startup so configuration errors are detected
  // immediately instead of causing unexpected behavior during a delivery.
  // ---------------------------------------------------------------------------
  void validate_configuration();

  std::string frame_id_;

  int max_retries_;
  int nav_result_poll_period_ms_;

  double feedback_rate_hz_;
  double nav2_server_timeout_sec_;
  double nav_goal_timeout_sec_;

  std::vector<std::string> location_names_;

  std::unordered_map<std::string, Location> locations_;

  rclcpp::Service<SubmitDelivery>::SharedPtr submit_service_;

  rclcpp_action::Server<ExecuteDelivery>::SharedPtr execute_action_server_;

  std::unique_ptr<NavigationManager> navigation_manager_;

  std::mutex state_mutex_;

  std::optional<DeliveryJob> current_job_;

  CourierState state_{CourierState::IDLE};

  uint64_t next_job_id_{1};
};

#endif  // ACADBOT_COURIER__COURIER_SERVER_HPP_
