// courier_server.cpp
// ---------------------------------------------------------------------------
// Final Project — AcadBot Courier
//
// High-level mission node for the AcadBot courier system.
//
// This node loads named delivery locations and courier behavior parameters
// from YAML configuration. It will expose the service used to accept delivery
// jobs, execute accepted jobs through a custom ROS 2 action, and command Nav2
// through the `navigate_to_pose` action.
//
// The courier mission consists of two navigation legs:
//   1. Navigate to the pickup location.
//   2. Navigate to the dropoff location.
//
// The node is responsible for job validation, progress feedback, cancellation,
// retry handling, and truthful reporting of navigation failures. Nav2 remains
// responsible for path planning, control, and its internal recovery behaviors.
//
// All location poses and tunable limits are loaded from configuration rather
// than hardcoded in the C++ source.
// ---------------------------------------------------------------------------

#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>
#include <iomanip>
#include <optional>
#include <sstream>
#include <mutex>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "acadbot_courier_interfaces/srv/submit_delivery.hpp"
#include "acadbot_courier_interfaces/action/execute_delivery.hpp"

using SubmitDelivery =
  acadbot_courier_interfaces::srv::SubmitDelivery;

using ExecuteDelivery =
  acadbot_courier_interfaces::action::ExecuteDelivery;

using CourierGoalHandle =
  rclcpp_action::ServerGoalHandle<ExecuteDelivery>;

// Named navigation pose loaded from YAML.
// Yaw is stored in radians and later converted to a quaternion for Nav2 goals.
struct Location{
  double x;
  double y;
  double yaw;
};

// Minimal information associated with one accepted courier request.
struct DeliveryJob{
  std::string id;
  std::string pickup;
  std::string dropoff;
};

// High-level job lifecycle of the courier.
// IDLE     : available to accept a new job.
// RESERVED : a job was accepted but execution has not started.
// RUNNING  : the accepted job is currently being executed.
enum class CourierState{
  IDLE,
  RESERVED,
  RUNNING
};

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
  CourierServer(): Node("courier_server")
  {

    // General courier configuration
    frame_id_ = declare_parameter<std::string>("frame_id", "map");

    max_retries_ = declare_parameter<int>("max_retries", 1);

    feedback_rate_hz_ =
      declare_parameter<double>("feedback_rate_hz", 2.0);

    nav2_server_timeout_sec_ =
      declare_parameter<double>("nav2_server_timeout_sec", 10.0);

    nav_goal_timeout_sec_ =
      declare_parameter<double>("nav_goal_timeout_sec", 120.0);

    // List of location names
    location_names_ =
      declare_parameter<std::vector<std::string>>(
        "location_names",
        std::vector<std::string>{});

    if (location_names_.empty()) {
      RCLCPP_FATAL(get_logger(),
        "No courier locations were configured.");

      throw std::runtime_error("no courier locations");
    }

    // Load each named location.
    for (const auto & name : location_names_) {
      const std::string parameter_name = "locations." + name;

      auto pose =
        declare_parameter<std::vector<double>>(
          parameter_name,
          std::vector<double>{});

      if (pose.size() != 3) {
        RCLCPP_FATAL(get_logger(),
          "Location '%s' must contain exactly [x, y, yaw].",
          name.c_str());

        throw std::runtime_error(
          "invalid location configuration");
      }

      locations_[name] = {pose[0], pose[1], pose[2]};
    }

    validate_configuration();

    RCLCPP_INFO(get_logger(),
      "Courier server configuration loaded successfully.");

    RCLCPP_INFO(get_logger(),
      "Frame: %s | max_retries: %d | feedback: %.1f Hz",
      frame_id_.c_str(), max_retries_, feedback_rate_hz_);

    for (const auto & name : location_names_) {
      const auto & location = locations_.at(name);

      RCLCPP_INFO(get_logger(),
        "Location '%s': x=%.4f, y=%.4f, yaw=%.4f",
        name.c_str(), location.x, location.y,
        location.yaw);
    }

    // Create SubmitDelivery service
    submit_service_ = create_service<SubmitDelivery>(
        "submit_delivery",
        std::bind(&CourierServer::handle_submit_delivery,
            this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(get_logger(),
        "Service '/submit_delivery' is ready.");

    // Create ExecuteDelivery action server
    execute_action_server_ =
      rclcpp_action::create_server<ExecuteDelivery>(
      this, "execute_delivery",
        std::bind(&CourierServer::handle_goal,
          this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&CourierServer::handle_cancel,
          this, std::placeholders::_1),
        std::bind(&CourierServer::handle_accepted,
          this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
      "Action '/execute_delivery' is ready.");
  }

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
  std::shared_ptr<SubmitDelivery::Response> response)
  {
    RCLCPP_INFO(get_logger(),
      "Received delivery request: '%s' -> '%s'",
      request->pickup.c_str(),
      request->dropoff.c_str());

    std::lock_guard<std::mutex> lock(state_mutex_);

    // The robot only handles one delivery at a time.
    if (state_ != CourierState::IDLE) {
      response->accepted = false;
      response->job_id = "";
      response->reason =
        "courier is busy with job " + current_job_->id;

      RCLCPP_WARN(get_logger(),
        "Rejected delivery request: %s",
        response->reason.c_str());

      return;
    }

    // Validate pickup location.
    if (locations_.find(request->pickup) == locations_.end()) {
      response->accepted = false;
      response->job_id = "";
      response->reason =
        "unknown pickup location: " + request->pickup;

      RCLCPP_WARN(get_logger(),
        "Rejected delivery request: %s",
        response->reason.c_str());

      return;
    }

    // Validate dropoff location.
    if (locations_.find(request->dropoff) == locations_.end()) {
      response->accepted = false;
      response->job_id = "";
      response->reason =
        "unknown dropoff location: " + request->dropoff;

        RCLCPP_WARN(get_logger(),
        "Rejected delivery request: %s",
        response->reason.c_str());

      return;
    }

    DeliveryJob job;
    job.id = generate_job_id();
    job.pickup = request->pickup;
    job.dropoff = request->dropoff;

    current_job_ = job;
    state_ = CourierState::RESERVED;

    response->accepted = true;
    response->job_id = job.id;
    response->reason = "";

    RCLCPP_INFO(get_logger(),
      "Accepted job %s: %s -> %s",
      job.id.c_str(),
      job.pickup.c_str(),
      job.dropoff.c_str());
  }

  // ---------------------------------------------------------------------------
  // Validate a request to execute an accepted delivery job.
  //
  // The action goal is accepted only when the courier has a reserved job and the
  // supplied job ID matches that reservation. This prevents callers from
  // executing unknown, stale, or unrelated jobs.
  // ---------------------------------------------------------------------------
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const ExecuteDelivery::Goal> goal)
    {
      std::lock_guard<std::mutex> lock(state_mutex_);

      RCLCPP_INFO(get_logger(),
        "Received execution request for job '%s'.",
        goal->job_id.c_str());

      if (state_ != CourierState::RESERVED || !current_job_) {
        RCLCPP_WARN(get_logger(),
          "Rejected execution request: no reserved delivery job.");

        return rclcpp_action::GoalResponse::REJECT;
      }

      if (goal->job_id != current_job_->id) {
        RCLCPP_WARN(get_logger(),
          "Rejected execution request: expected '%s', received '%s'.",
          current_job_->id.c_str(),
          goal->job_id.c_str());

        return rclcpp_action::GoalResponse::REJECT;
      }

      RCLCPP_INFO(get_logger(),
        "Accepted execution request for job '%s'.",
        goal->job_id.c_str());

      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

  // ---------------------------------------------------------------------------
  // Accept cancellation requests for a running delivery.
  //
  // At this stage the action execution itself responds to cancellation. Once
  // Nav2 integration is added, the same cancellation path will also cancel the
  // active `navigate_to_pose` goal so the physical robot stops promptly.
  // ---------------------------------------------------------------------------
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<CourierGoalHandle>)
    {
      RCLCPP_WARN(get_logger(),
        "Cancellation requested for the active delivery.");

      return rclcpp_action::CancelResponse::ACCEPT;
    }

  // ---------------------------------------------------------------------------
  // Start execution of an accepted delivery action goal.
  //
  // Delivery execution is moved to a separate thread so the ROS executor remains
  // free to process service requests, action feedback, cancellation requests,
  // and later Nav2 callbacks while the mission is running.
  // ---------------------------------------------------------------------------
  void handle_accepted(const std::shared_ptr<CourierGoalHandle> goal_handle){
    std::thread(
      [this, goal_handle]() {execute_delivery(goal_handle);}).detach();
  }

  // ---------------------------------------------------------------------------
  // Execute the currently reserved courier job.
  //
  // This first implementation simulates a short delivery while publishing action
  // feedback. It verifies the complete custom action lifecycle before real Nav2
  // navigation is introduced. The simulated section will later be replaced by
  // pickup and dropoff NavigateToPose goals.
  // ---------------------------------------------------------------------------
  void execute_delivery(const std::shared_ptr<CourierGoalHandle> goal_handle)
  {
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      state_ = CourierState::RUNNING;
    }

    RCLCPP_INFO(get_logger(),
      "Starting simulated delivery execution.");

    auto feedback =
      std::make_shared<ExecuteDelivery::Feedback>();

    rclcpp::WallRate rate(feedback_rate_hz_);

    for (int step = 0; step < 6; ++step) {

      // Check whether the requester canceled the action.
      if (goal_handle->is_canceling()) {

        auto result =
          std::make_shared<ExecuteDelivery::Result>();

        result->success = false;
        result->message = "delivery canceled";
        result->failed_leg = "";

        goal_handle->canceled(result);

        RCLCPP_WARN(get_logger(), "Delivery ended as CANCELED.");

        reset_job();
        return;
      }

      feedback->leg = "pickup";
      feedback->target =
        current_job_ ? current_job_->pickup : "";

      feedback->distance_remaining =
        static_cast<float>(6 - step);

      feedback->attempt = 1;

      feedback->max_attempts =
        static_cast<uint32_t>(max_retries_ + 1);

      goal_handle->publish_feedback(feedback);

      RCLCPP_INFO(get_logger(),
        "Simulated feedback: pickup -> %s, %.1f m remaining",
        feedback->target.c_str(),
        feedback->distance_remaining);

      rate.sleep();
    }

    auto result =
      std::make_shared<ExecuteDelivery::Result>();

    result->success = true;
    result->message =
      "simulated delivery completed successfully";
    result->failed_leg = "";

    goal_handle->succeed(result);

    RCLCPP_INFO(get_logger(),
      "Simulated delivery completed successfully.");

    reset_job();
  }


  // ---------------------------------------------------------------------------
  // Generate the next process-local delivery job identifier.
  //
  // Job IDs use a simple sequential format (job_0001, job_0002, ...). They only
  // need to remain unique during the current courier_server execution; persistent
  // identifiers across robot restarts are outside the scope of this prototype.
  // ---------------------------------------------------------------------------
  std::string generate_job_id(){
    std::ostringstream stream;

    stream
      << "job_"
      << std::setw(4)
      << std::setfill('0')
      << next_job_id_++;

    return stream.str();
  }

  // ---------------------------------------------------------------------------
  // Clear the current delivery and return the courier to the idle state.
  //
  // This is called whenever a job reaches a terminal state so the robot becomes
  // available to accept a new delivery request.
  // ---------------------------------------------------------------------------
  void reset_job(){
    std::lock_guard<std::mutex> lock(state_mutex_);

    current_job_.reset();
    state_ = CourierState::IDLE;

    RCLCPP_INFO(get_logger(), "Courier returned to IDLE.");
  }

  // ---------------------------------------------------------------------------
  // Validate courier configuration limits.
  //
  // Rejects invalid values at startup so configuration errors are detected
  // immediately instead of causing unexpected behavior during a delivery.
  // ---------------------------------------------------------------------------
  void validate_configuration()
  {
    if (max_retries_ < 0) {
      throw std::runtime_error(
        "max_retries must be >= 0");
    }

    if (feedback_rate_hz_ <= 0.0) {
      throw std::runtime_error(
        "feedback_rate_hz must be > 0");
    }

    if (nav2_server_timeout_sec_ <= 0.0) {
      throw std::runtime_error(
        "nav2_server_timeout_sec must be > 0");
    }

    if (nav_goal_timeout_sec_ <= 0.0) {
      throw std::runtime_error(
        "nav_goal_timeout_sec must be > 0");
    }
  }

  std::string frame_id_;

  int max_retries_;

  double feedback_rate_hz_;
  double nav2_server_timeout_sec_;
  double nav_goal_timeout_sec_;

  std::vector<std::string> location_names_;

  std::unordered_map<std::string, Location> locations_;

  rclcpp::Service<SubmitDelivery>::SharedPtr submit_service_;

  rclcpp_action::Server<ExecuteDelivery>::SharedPtr execute_action_server_;

  std::mutex state_mutex_;

  std::optional<DeliveryJob> current_job_;

  CourierState state_{CourierState::IDLE};

  uint64_t next_job_id_{1};
};

// ---------------------------------------------------------------------------
// Program entry point.
//
// Initializes ROS 2, creates the courier server node, and spins it so service
// and later action callbacks can be processed. Startup configuration errors are
// caught and reported before the process exits with failure.
// ---------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    rclcpp::spin(
      std::make_shared<CourierServer>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(
      rclcpp::get_logger("courier_server"),
      "Failed to start courier server: %s",
      e.what());

    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}