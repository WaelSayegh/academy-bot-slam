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
#include <future>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "acadbot_courier_interfaces/srv/submit_delivery.hpp"
#include "acadbot_courier_interfaces/action/execute_delivery.hpp"

using namespace std::chrono_literals;

using SubmitDelivery =
  acadbot_courier_interfaces::srv::SubmitDelivery;

using ExecuteDelivery =
  acadbot_courier_interfaces::action::ExecuteDelivery;

using CourierGoalHandle =
  rclcpp_action::ServerGoalHandle<ExecuteDelivery>;

using NavigateToPose =
  nav2_msgs::action::NavigateToPose;

using NavGoalHandle =
  rclcpp_action::ClientGoalHandle<NavigateToPose>;

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

// Possible terminal outcomes of one Nav2 navigation leg.
enum class NavigationOutcome
{
  SUCCEEDED,
  ABORTED,
  CANCELED,
  REJECTED,
  TIMED_OUT
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
    nav2_client_ =
      rclcpp_action::create_client<NavigateToPose>(
       this, "navigate_to_pose");

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

    // Verify that the Nav2 action server is available.
    if (!nav2_client_->action_server_is_ready()) {
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
  // Accept cancellation requests for the active courier action.
  //
  // The execution thread detects the cancellation request and propagates it to
  // the currently active Nav2 NavigateToPose goal.
  // ---------------------------------------------------------------------------
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<CourierGoalHandle> goal_handle)
    {
      RCLCPP_WARN(get_logger(),
        "Cancellation requested for courier job '%s'.",
        goal_handle->get_goal()->job_id.c_str());

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
  // The courier performs two sequential Nav2 navigation legs:
  //   1. Navigate to the configured pickup location.
  //   2. After pickup succeeds, navigate to the configured dropoff location.
  //
  // The dropoff leg is never started unless the pickup leg actually succeeds.
  // Any Nav2 abort or cancellation is propagated honestly to the courier action
  // and identifies which delivery leg failed.
  // ---------------------------------------------------------------------------
  void execute_delivery(
    const std::shared_ptr<CourierGoalHandle> goal_handle)
  {
    DeliveryJob job;

    uint32_t current_attempt = 1;

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
      navigate_to_location(
        "pickup",
        job.pickup,
        current_attempt,
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
      navigate_to_location(
        "dropoff",
        job.dropoff,
        current_attempt,
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

  // ---------------------------------------------------------------------------
  // Convert a configured courier location into a Nav2 goal pose.
  //
  // Courier locations are stored in YAML as [x, y, yaw]. Nav2 expects a
  // geometry_msgs::msg::PoseStamped, so this helper adds the configured map
  // frame and current timestamp and converts the planar yaw angle into a
  // quaternion orientation.
  // ---------------------------------------------------------------------------
  geometry_msgs::msg::PoseStamped make_pose(const Location & location)
  {
    geometry_msgs::msg::PoseStamped pose;

    pose.header.frame_id = frame_id_;
    pose.header.stamp = now();

    pose.pose.position.x = location.x;
    pose.pose.position.y = location.y;
    pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, location.yaw);

    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();

    return pose;
  }

  // ---------------------------------------------------------------------------
  // Publish human-readable progress for the active courier action.
  //
  // Feedback identifies the current delivery leg, named target, latest remaining
  // distance reported by Nav2, and the current navigation attempt. The maximum
  // number of attempts comes from the configured retry limit.
  // ---------------------------------------------------------------------------
  void publish_delivery_feedback(
    const std::shared_ptr<CourierGoalHandle> & goal_handle,
    const std::string & leg,
    const std::string & target,
    double distance_remaining,
    uint32_t attempt)
  {
    auto feedback =
      std::make_shared<ExecuteDelivery::Feedback>();

    feedback->leg = leg;
    feedback->target = target;
    feedback->distance_remaining =
      static_cast<float>(distance_remaining);
    feedback->attempt = attempt;
    feedback->max_attempts =
      static_cast<uint32_t>(max_retries_ + 1);

    goal_handle->publish_feedback(feedback);
  }

  // ---------------------------------------------------------------------------
  // Navigate to one configured named location using Nav2.
  //
  // The location name is resolved through the YAML-loaded location map and
  // converted into a PoseStamped before being sent to Nav2's navigate_to_pose
  // action server.
  //
  // This helper waits for Nav2's result from the delivery worker thread while
  // the ROS executor remains free to process action callbacks. It returns a
  // NavigationOutcome so the courier mission can distinguish success, abort,
  // cancellation, and goal rejection.
  // ---------------------------------------------------------------------------
  NavigationOutcome navigate_to_location(
    const std::string & leg,
    const std::string & location_name,
    uint32_t attempt,
    const std::shared_ptr<CourierGoalHandle> & courier_goal_handle)
  {
    const auto location_it = locations_.find(location_name);

    // Verify that the requested location exists in the configured map.
    if (location_it == locations_.end()) {
      RCLCPP_ERROR(get_logger(),
        "Cannot navigate to unknown location '%s'.",
        location_name.c_str());

      return NavigationOutcome::REJECTED;
    }
    
    // Reset the latest distance remaining and feedback flag before sending a new goal.
    {
      std::lock_guard<std::mutex> lock(state_mutex_);

      latest_distance_remaining_ = 0.0;
      nav_feedback_received_ = false;
    }

    NavigateToPose::Goal nav_goal;
    nav_goal.pose = make_pose(location_it->second);

    RCLCPP_INFO(get_logger(),
      "Sending Nav2 goal to '%s' at (%.4f, %.4f, yaw=%.4f).",
      location_name.c_str(),
      location_it->second.x,
      location_it->second.y,
      location_it->second.yaw);

    rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;

    options.feedback_callback =
      [this, location_name](NavGoalHandle::SharedPtr,
        const std::shared_ptr<const NavigateToPose::Feedback> feedback)
        {
          {
            std::lock_guard<std::mutex> lock(state_mutex_);

            latest_distance_remaining_ = feedback->distance_remaining;

            nav_feedback_received_ = true;
          }

          RCLCPP_DEBUG(get_logger(),
            "Nav2 feedback for '%s': %.2f m remaining",
            location_name.c_str(),
            feedback->distance_remaining);
        };

    auto goal_future =
      nav2_client_->async_send_goal(nav_goal, options);

    // Wait for Nav2 to accept or reject the goal.
    if (goal_future.wait_for(std::chrono::seconds(
      static_cast<int>(nav2_server_timeout_sec_))) !=
        std::future_status::ready)
    {
      RCLCPP_ERROR(get_logger(),
        "Timed out waiting for Nav2 to accept goal '%s'.",
        location_name.c_str());

      return NavigationOutcome::REJECTED;
    }

    auto nav_goal_handle = goal_future.get();

    if (!nav_goal_handle) {
      RCLCPP_ERROR(get_logger(),
        "Nav2 rejected goal for '%s'.",
        location_name.c_str());

      return NavigationOutcome::REJECTED;
    }

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      active_nav_goal_ = nav_goal_handle;
    }

    RCLCPP_INFO(get_logger(),
      "Nav2 accepted goal for '%s'.",
      location_name.c_str());

    auto result_future =
      nav2_client_->async_get_result(nav_goal_handle);

    const auto start_time = now();

    auto last_feedback_publish = std::chrono::steady_clock::now();

    const double feedback_period_sec = 1.0 / feedback_rate_hz_;

    bool nav_cancel_requested = false;
    bool navigation_timed_out = false;

    while (result_future.wait_for(
      std::chrono::milliseconds(nav_result_poll_period_ms_))
        != std::future_status::ready) 
    {
      // ---------------------------------------------------------
      // Courier action cancellation
      // ---------------------------------------------------------

      if (courier_goal_handle->is_canceling() &&
        !nav_cancel_requested)
      {
        RCLCPP_WARN(get_logger(),
          "Courier action canceled while navigating to '%s'. "
          "Canceling active Nav2 goal.",
          location_name.c_str());

        nav2_client_->async_cancel_goal(nav_goal_handle);

        nav_cancel_requested = true;
      }


      // ---------------------------------------------------------
      // Courier action feedback
      // ---------------------------------------------------------

      const auto wall_now = std::chrono::steady_clock::now();

      const double time_since_feedback =
        std::chrono::duration<double>(
          wall_now - last_feedback_publish).count();

      if (time_since_feedback >= feedback_period_sec) {

        double distance = 0.0;
        bool have_distance = false;

        {
          std::lock_guard<std::mutex> lock(state_mutex_);

          distance = latest_distance_remaining_;
          have_distance = nav_feedback_received_;
        }

        if (have_distance && !courier_goal_handle->is_canceling()) {
          publish_delivery_feedback(
            courier_goal_handle,
            leg,
            location_name,
            distance,
            attempt);
        }

        last_feedback_publish = wall_now;
      }


      // ---------------------------------------------------------
      // Courier navigation timeout
      // ---------------------------------------------------------

      const double elapsed = (now() - start_time).seconds();

      if (elapsed >= nav_goal_timeout_sec_ && !nav_cancel_requested) 
      {
        RCLCPP_ERROR(get_logger(),
          "Navigation to '%s' exceeded timeout of %.1f seconds.",
          location_name.c_str(),
          nav_goal_timeout_sec_);

        nav2_client_->async_cancel_goal(nav_goal_handle);

        nav_cancel_requested = true;

        // Remember that this was caused by timeout rather than user cancellation.
        navigation_timed_out = true;
      }
    }


    // ---------------------------------------------------------
    // Process the final Nav2 result
    // ---------------------------------------------------------

    const auto result = result_future.get();

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      active_nav_goal_.reset();
    }

    if (navigation_timed_out) {
      RCLCPP_ERROR(get_logger(),
        "Navigation to '%s' stopped after courier timeout.",
        location_name.c_str());

      return NavigationOutcome::TIMED_OUT;
    }

    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(get_logger(),
          "Reached location '%s'.",
          location_name.c_str());

        return NavigationOutcome::SUCCEEDED;

      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(get_logger(),
          "Nav2 aborted navigation to '%s'.",
          location_name.c_str());

        return NavigationOutcome::ABORTED;

      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_WARN(get_logger(),
          "Navigation to '%s' was canceled.",
          location_name.c_str());

        return NavigationOutcome::CANCELED;

      default:
        RCLCPP_ERROR(get_logger(),
          "Navigation to '%s' ended with an unknown result.",
          location_name.c_str());

        return NavigationOutcome::ABORTED;
    }
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
  int nav_result_poll_period_ms_;

  double feedback_rate_hz_;
  double nav2_server_timeout_sec_;
  double nav_goal_timeout_sec_;
  double latest_distance_remaining_{0.0};

  bool nav_feedback_received_{false};

  std::vector<std::string> location_names_;

  std::unordered_map<std::string, Location> locations_;

  rclcpp::Service<SubmitDelivery>::SharedPtr submit_service_;

  rclcpp_action::Server<ExecuteDelivery>::SharedPtr execute_action_server_;

  rclcpp_action::Client<NavigateToPose>::SharedPtr nav2_client_;

  NavGoalHandle::SharedPtr active_nav_goal_;

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