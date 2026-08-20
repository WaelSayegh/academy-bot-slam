#include "acadbot_courier/courier_bt_server.hpp"

#include <chrono>
#include <exception>
#include <iomanip>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <utility>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "acadbot_courier/location_markers.hpp"

namespace acadbot_courier
{

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

namespace
{
constexpr double kMarkerPeriod = 1.0;   // s
constexpr double kBtTickPeriod = 0.1;   // 10 Hz
}  // namespace

CourierBtServer::CourierBtServer()
: Node("courier_bt_server")
{
  goal_frame_ = declare_parameter<std::string>("goal_frame", "map");
  max_retries_ = declare_parameter<int>("max_retries", 2);
  retry_delay_ = declare_parameter<double>("retry_delay", 3.0);
  feedback_period_ = declare_parameter<double>("feedback_period", 0.5);
  leg_timeout_ = declare_parameter<double>("leg_timeout", 120.0);
  bt_xml_path_ = declare_parameter<std::string>("behavior_tree_xml", "");

  if (bt_xml_path_.empty()) {
    const std::string pkg_share =
      ament_index_cpp::get_package_share_directory("acadbot_courier");
    bt_xml_path_ = pkg_share + "/behavior_trees/courier.xml";
  }

  locations_ = LocationTable::from_parameters(*this);

  nav_client_ =
    rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

  request_service_ = create_service<RequestDelivery>(
    "request_delivery",
    std::bind(&CourierBtServer::handle_request, this, _1, _2));

  delivery_server_ = rclcpp_action::create_server<ExecuteDelivery>(
    this, "execute_delivery",
    std::bind(&CourierBtServer::handle_goal, this, _1, _2),
    std::bind(&CourierBtServer::handle_cancel, this, _1),
    std::bind(&CourierBtServer::handle_accepted, this, _1));

  rclcpp::QoS marker_qos(1);
  marker_qos.transient_local();
  marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    "~/locations", marker_qos);

  marker_timer_ = create_wall_timer(
    std::chrono::duration<double>(kMarkerPeriod),
    std::bind(&CourierBtServer::publish_markers, this));

  // Shared with every GoToLocation the tree builds.
  leg_status_ = std::make_shared<LegStatus>();
  nav_context_.node = this;
  nav_context_.nav_client = nav_client_;
  nav_context_.status = leg_status_;
  nav_context_.goal_frame = goal_frame_;

  // Teach the factory about GoToLocation before any tree is built.
  register_courier_nodes(factory_, nav_context_);

  RCLCPP_INFO(
    get_logger(),
    "courier_bt_server ready (Behavior Tree): %zu locations (%s); XML: %s",
    locations_.size(), locations_.known_names().c_str(), bt_xml_path_.c_str());
}

void CourierBtServer::handle_request(
  const std::shared_ptr<RequestDelivery::Request> request,
  std::shared_ptr<RequestDelivery::Response> response)
{
  const auto reject = [&](const std::string & why) {
      response->accepted = false;
      response->job_id = "";
      response->reason = why;
      RCLCPP_WARN(get_logger(), "rejected '%s' -> '%s': %s",
        request->pickup.c_str(), request->dropoff.c_str(), why.c_str());
    };

  if (!active_job_id_.empty()) {
    reject("a delivery is already running (" + active_job_id_ + ")");
    return;
  }

  const auto pickup = locations_.find(request->pickup);
  if (!pickup) {
    reject("unknown pickup '" + request->pickup + "'; known locations are: " +
      locations_.known_names());
    return;
  }

  const auto dropoff = locations_.find(request->dropoff);
  if (!dropoff) {
    reject("unknown dropoff '" + request->dropoff + "'; known locations are: " +
      locations_.known_names());
    return;
  }

  if (request->pickup == request->dropoff) {
    reject("pickup and dropoff are both '" + request->pickup +
      "'; there is nothing to deliver");
    return;
  }

  if (!nav_client_->action_server_is_ready()) {
    reject("navigation is not available: Nav2's navigate_to_pose action "
      "server is not up");
    return;
  }

  std::ostringstream id;
  id << "job_" << std::setw(4) << std::setfill('0') << ++next_job_number_;

  Job job;
  job.id = id.str();
  job.pickup_name = request->pickup;
  job.dropoff_name = request->dropoff;
  job.pickup = *pickup;
  job.dropoff = *dropoff;
  job.state = JobState::BOOKED;
  jobs_.emplace(job.id, job);

  response->accepted = true;
  response->job_id = job.id;
  response->reason = "";

  RCLCPP_INFO(get_logger(), "booked %s: '%s' -> '%s' (BT mode)",
    job.id.c_str(), job.pickup_name.c_str(), job.dropoff_name.c_str());
}

rclcpp_action::GoalResponse CourierBtServer::handle_goal(
  const rclcpp_action::GoalUUID &,
  std::shared_ptr<const ExecuteDelivery::Goal> goal)
{
  if (!active_job_id_.empty()) {
    RCLCPP_WARN(get_logger(), "goal for %s rejected: %s is already running",
      goal->job_id.c_str(), active_job_id_.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }

  const auto it = jobs_.find(goal->job_id);
  if (it == jobs_.end()) {
    RCLCPP_WARN(get_logger(), "goal rejected: no job '%s' was ever booked",
      goal->job_id.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (it->second.state != JobState::BOOKED) {
    RCLCPP_WARN(get_logger(), "goal for %s rejected: it is already %s",
      goal->job_id.c_str(), to_string(it->second.state));
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (!nav_client_->action_server_is_ready()) {
    RCLCPP_WARN(get_logger(), "goal for %s rejected: Nav2 is not available",
      goal->job_id.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse CourierBtServer::handle_cancel(
  const std::shared_ptr<ExecuteGoalHandle> goal_handle)
{
  if (!execute_handle_ || goal_handle != execute_handle_) {
    return rclcpp_action::CancelResponse::REJECT;
  }

  RCLCPP_INFO(get_logger(), "%s: BT cancel requested", active_job_id_.c_str());

  if (leg_status_) {
    leg_status_->state = "CANCELING";
  }

  if (tree_running_) {
    tree_.haltTree();
    tree_running_ = false;
  }

  // The goal only enters CANCELING once this returns, so finish on a timer.
  if (cancel_timer_) {
    cancel_timer_->cancel();
  }
  cancel_timer_ = create_wall_timer(1ms, [this]() {
    if (cancel_timer_) {
      cancel_timer_->cancel();
      cancel_timer_.reset();
    }
    finish_canceled();
  });

  return rclcpp_action::CancelResponse::ACCEPT;
}

void CourierBtServer::handle_accepted(
  const std::shared_ptr<ExecuteGoalHandle> goal_handle)
{
  execute_handle_ = goal_handle;
  active_job_id_ = goal_handle->get_goal()->job_id;
  current_job().state = JobState::RUNNING;

  Job & job = current_job();

  // Reset the feedback state for a fresh job.
  leg_status_->leg = Leg::PICKUP;
  leg_status_->target_location = job.pickup_name;
  leg_status_->state = "NAVIGATING";
  leg_status_->distance_remaining = 0.0f;
  leg_status_->nav2_recoveries = 0;
  leg_status_->attempt = 0;
  leg_status_->attempts_total = 0;

  // The blackboard the tree reads its targets and limits from.
  auto blackboard = BT::Blackboard::create();
  blackboard->set("pickup_pose", job.pickup);
  blackboard->set("dropoff_pose", job.dropoff);
  blackboard->set("pickup_name", job.pickup_name);
  blackboard->set("dropoff_name", job.dropoff_name);
  blackboard->set("max_attempts", max_retries_ + 1);
  blackboard->set("leg_timeout_msec", static_cast<unsigned int>(leg_timeout_ * 1000.0));
  blackboard->set("retry_delay_msec", static_cast<unsigned int>(retry_delay_ * 1000.0));

  try {
    tree_ = factory_.createTreeFromFile(bt_xml_path_, blackboard);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(get_logger(), "Failed to create Behavior Tree from '%s': %s",
      bt_xml_path_.c_str(), e.what());
    finish_failed(std::string("Tree creation error: ") + e.what());
    return;
  }

  tree_running_ = true;

  // Feedback at 2 Hz, tree ticks at 10 Hz.
  feedback_timer_ = create_wall_timer(
    std::chrono::duration<double>(feedback_period_),
    std::bind(&CourierBtServer::publish_feedback, this));

  bt_tick_timer_ = create_wall_timer(
    std::chrono::duration<double>(kBtTickPeriod),
    std::bind(&CourierBtServer::on_bt_tick, this));

  RCLCPP_INFO(get_logger(), "%s: Behavior Tree started; '%s' -> '%s'",
    active_job_id_.c_str(), job.pickup_name.c_str(), job.dropoff_name.c_str());

  publish_markers();
}

void CourierBtServer::on_bt_tick()
{
  if (!tree_running_ || active_job_id_.empty()) {
    return;
  }

  BT::NodeStatus status = tree_.tickOnce();

  if (status == BT::NodeStatus::RUNNING) {
    publish_markers();
    return;
  }

  tree_running_ = false;

  if (status == BT::NodeStatus::SUCCESS) {
    finish_succeeded();
  } else {
    finish_failed("the " + std::string(to_string(leg_status_->leg)) +
      " leg failed (BT exhausted retries)");
  }
}

void CourierBtServer::finish_succeeded()
{
  Job & job = current_job();
  auto result = std::make_shared<ExecuteDelivery::Result>();
  result->success = true;
  result->job_id = job.id;
  result->message =
    "delivered from '" + job.pickup_name + "' to '" + job.dropoff_name + "' (BT)";
  result->failed_leg = "";
  result->attempts_used = static_cast<uint8_t>(leg_status_->attempts_total);

  job.state = JobState::SUCCEEDED;
  RCLCPP_INFO(get_logger(), "%s: %s", job.id.c_str(), result->message.c_str());

  if (execute_handle_) {
    execute_handle_->succeed(result);
  }
  reset_to_idle();
}

void CourierBtServer::finish_failed(const std::string & message)
{
  Job & job = current_job();
  auto result = std::make_shared<ExecuteDelivery::Result>();
  result->success = false;
  result->job_id = job.id;
  result->message = message;
  result->failed_leg = to_string(leg_status_->leg);
  result->attempts_used = static_cast<uint8_t>(leg_status_->attempts_total);

  job.state = JobState::FAILED;
  RCLCPP_ERROR(get_logger(), "%s: %s", job.id.c_str(), message.c_str());

  if (execute_handle_) {
    execute_handle_->abort(result);
  }
  reset_to_idle();
}

void CourierBtServer::finish_canceled()
{
  if (active_job_id_.empty()) {
    return;
  }
  Job & job = current_job();
  auto result = std::make_shared<ExecuteDelivery::Result>();
  result->success = false;
  result->job_id = job.id;
  result->message = "cancelled during the " +
    std::string(to_string(leg_status_->leg)) + " leg";
  result->failed_leg = "";
  result->attempts_used = static_cast<uint8_t>(leg_status_->attempts_total);

  job.state = JobState::CANCELED;
  RCLCPP_INFO(get_logger(), "%s: %s", job.id.c_str(), result->message.c_str());

  if (execute_handle_) {
    execute_handle_->canceled(result);
  }
  reset_to_idle();
}

void CourierBtServer::reset_to_idle()
{
  if (feedback_timer_) {feedback_timer_->cancel();}
  if (bt_tick_timer_) {bt_tick_timer_->cancel();}
  if (cancel_timer_) {cancel_timer_->cancel();}
  feedback_timer_.reset();
  bt_tick_timer_.reset();
  cancel_timer_.reset();

  tree_running_ = false;
  execute_handle_.reset();
  active_job_id_.clear();

  publish_markers();
}

void CourierBtServer::publish_feedback()
{
  if (active_job_id_.empty() || !execute_handle_ || !leg_status_) {
    return;
  }

  auto feedback = std::make_shared<ExecuteDelivery::Feedback>();
  feedback->leg = (leg_status_->leg == Leg::PICKUP)
    ? ExecuteDelivery::Feedback::LEG_PICKUP
    : ExecuteDelivery::Feedback::LEG_DROPOFF;
  feedback->target_location = leg_status_->target_location;
  feedback->distance_remaining = leg_status_->distance_remaining;
  feedback->attempt = static_cast<uint8_t>(leg_status_->attempt);
  feedback->nav2_recoveries = leg_status_->nav2_recoveries;
  feedback->state = leg_status_->state;

  execute_handle_->publish_feedback(feedback);
}

void CourierBtServer::publish_markers()
{
  std::string active;
  if (!active_job_id_.empty() && leg_status_) {
    active = leg_status_->target_location;
  }
  marker_pub_->publish(
    make_location_markers(locations_, goal_frame_, now(), active));
}

Job & CourierBtServer::current_job()
{
  const auto it = jobs_.find(active_job_id_);
  if (it == jobs_.end()) {
    throw std::runtime_error("no job '" + active_job_id_ + "' is active");
  }
  return it->second;
}

}  // namespace acadbot_courier

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    // Single-threaded for the same reason as the state machine: nothing
    // blocks, so no state needs a mutex.
    rclcpp::spin(std::make_shared<acadbot_courier::CourierBtServer>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("courier_bt_server"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
