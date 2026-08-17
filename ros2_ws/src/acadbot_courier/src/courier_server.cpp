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

#include "acadbot_courier/courier_server.hpp"

#include <functional>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <thread>

CourierServer::CourierServer(): Node("courier_server")
{

  // General courier configuration
  frame_id_ = declare_parameter<std::string>("frame_id", "map");

  max_retries_ = declare_parameter<int>("max_retries", 1);

  feedback_rate_hz_ =
    declare_parameter<double>("feedback_rate_hz", 2.0);

  nav2_server_timeout_sec_ =
    declare_parameter<double>("nav2_server_timeout_sec", 10.0);

  nav_goal_timeout_sec_ =
    declare_parameter<double>("nav_goal_timeout_sec", 180.0);

  nav_result_poll_period_ms_ =
    declare_parameter<int>("nav_result_poll_period_ms", 100);

  if (nav_result_poll_period_ms_ <= 0) {
    throw std::runtime_error(
    "nav_result_poll_period_ms must be > 0");
  }

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

  // Create Nav2 client
  navigation_manager_ = std::make_unique<NavigationManager>(
    this,
    frame_id_,
    max_retries_,
    feedback_rate_hz_,
    nav2_server_timeout_sec_,
    nav_goal_timeout_sec_,
    nav_result_poll_period_ms_,
    locations_);

  RCLCPP_INFO(get_logger(),
    "Nav2 'navigate_to_pose' action client created.");

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

void CourierServer::handle_submit_delivery(
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

  // Verify that the Nav2 action server is available.
  if (!navigation_manager_->action_server_is_ready()) {
    response->accepted = false;
    response->job_id = "";
    response->reason =
      "Nav2 navigate_to_pose action server is not available";

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

rclcpp_action::GoalResponse CourierServer::handle_goal(
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

rclcpp_action::CancelResponse CourierServer::handle_cancel(
  const std::shared_ptr<CourierGoalHandle> goal_handle)
{
  RCLCPP_WARN(get_logger(),
    "Cancellation requested for courier job '%s'.",
    goal_handle->get_goal()->job_id.c_str());

  return rclcpp_action::CancelResponse::ACCEPT;
}

void CourierServer::handle_accepted(
  const std::shared_ptr<CourierGoalHandle> goal_handle)
{
  std::thread(
    [this, goal_handle]() {execute_delivery(goal_handle);}).detach();
}

void CourierServer::execute_delivery(
  const std::shared_ptr<CourierGoalHandle> goal_handle)
{
  DeliveryJob job;

  {
    std::lock_guard<std::mutex> lock(state_mutex_);

    if (!current_job_) {
      RCLCPP_ERROR(get_logger(),
        "Execution started without a reserved job.");

      auto result = std::make_shared<ExecuteDelivery::Result>();

      result->success = false;
      result->message = "no reserved delivery job";
      result->failed_leg = "pickup";

      goal_handle->abort(result);
      state_ = CourierState::IDLE;
      return;
    }

    state_ = CourierState::RUNNING;
    job = *current_job_;
  }

  // -------------------------------------------------------------------------
  // PICKUP LEG
  // -------------------------------------------------------------------------

  RCLCPP_INFO(get_logger(),
    "Job %s starting PICKUP leg: %s",
    job.id.c_str(),
    job.pickup.c_str());

  const NavigationOutcome pickup_result =
    navigation_manager_->navigate_with_retries(
      "pickup",
      job.pickup,
      goal_handle);

  if (pickup_result != NavigationOutcome::SUCCEEDED) {
    auto result = std::make_shared<ExecuteDelivery::Result>();

    result->success = false;
    result->failed_leg = "pickup";

    if (pickup_result == NavigationOutcome::CANCELED) {
      result->message = "delivery canceled during pickup";

      goal_handle->canceled(result);

      RCLCPP_WARN(get_logger(),
        "Job %s canceled during pickup.",
        job.id.c_str());
    } 
    else if (pickup_result == NavigationOutcome::TIMED_OUT) {
      result->message = "pickup navigation timed out";
      goal_handle->abort(result);

      RCLCPP_ERROR(get_logger(),
        "Job %s timed out during pickup.",
        job.id.c_str());
    } 
    else {
      result->message = "pickup navigation failed";

      goal_handle->abort(result);

      RCLCPP_ERROR(get_logger(),
        "Job %s failed during pickup.",
        job.id.c_str());
    }

    reset_job();
    return;
  }

  RCLCPP_INFO(get_logger(),
    "Job %s reached pickup '%s'.",
    job.id.c_str(),
    job.pickup.c_str());

  // -------------------------------------------------------------------------
  // DROPOFF LEG
  // -------------------------------------------------------------------------

  RCLCPP_INFO(get_logger(),
    "Job %s starting DROPOFF leg: %s",
    job.id.c_str(),
    job.dropoff.c_str());

  const NavigationOutcome dropoff_result =
    navigation_manager_->navigate_with_retries(
      "dropoff",
      job.dropoff,
      goal_handle);

  if (dropoff_result != NavigationOutcome::SUCCEEDED) {
    auto result = std::make_shared<ExecuteDelivery::Result>();

    result->success = false;
    result->failed_leg = "dropoff";

    if (dropoff_result == NavigationOutcome::CANCELED) {
      result->message = "delivery canceled during dropoff";

      goal_handle->canceled(result);

      RCLCPP_WARN(get_logger(),
        "Job %s canceled during dropoff.",
        job.id.c_str());
    } 
    else if (dropoff_result == NavigationOutcome::TIMED_OUT) {
      result->message = "dropoff navigation timed out";
      goal_handle->abort(result);

      RCLCPP_ERROR(get_logger(),
        "Job %s timed out during dropoff.",
        job.id.c_str());
    } 
    else {
      result->message = "dropoff navigation failed";

      goal_handle->abort(result);

      RCLCPP_ERROR(get_logger(),
        "Job %s failed during dropoff.",
        job.id.c_str());
    }

    reset_job();
    return;
  }

  RCLCPP_INFO(get_logger(),
    "Job %s reached dropoff '%s'.",
    job.id.c_str(),
    job.dropoff.c_str());

  // -------------------------------------------------------------------------
  // DELIVERY SUCCESS
  // -------------------------------------------------------------------------

  auto result = std::make_shared<ExecuteDelivery::Result>();

  result->success = true;
  result->message = "delivery completed successfully";
  result->failed_leg = "";

  goal_handle->succeed(result);

  RCLCPP_INFO(get_logger(),
    "Job %s completed successfully: %s -> %s",
    job.id.c_str(),
    job.pickup.c_str(),
    job.dropoff.c_str());

  reset_job();
}

std::string CourierServer::generate_job_id()
{
  std::ostringstream stream;

  stream
    << "job_"
    << std::setw(4)
    << std::setfill('0')
    << next_job_id_++;

  return stream.str();
}

void CourierServer::reset_job()
{
  std::lock_guard<std::mutex> lock(state_mutex_);

  current_job_.reset();
  state_ = CourierState::IDLE;

  RCLCPP_INFO(get_logger(), "Courier returned to IDLE.");
}

void CourierServer::validate_configuration()
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
