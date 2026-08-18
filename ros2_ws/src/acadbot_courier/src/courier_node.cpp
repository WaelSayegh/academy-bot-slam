// ---------------------------------------------------------------------------
// AcadBot Courier — final project.
//
// Owns the set of named locations and the registry of booked/running/
// finished deliveries, and exposes /request_delivery as the entry point for
// booking a new job.
// ---------------------------------------------------------------------------
#include <chrono>
#include <cstdio>
#include <functional>
#include <map>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "acadbot_courier_interfaces/srv/request_delivery.hpp"

using namespace std::chrono_literals;
using RequestDelivery = acadbot_courier_interfaces::srv::RequestDelivery;

struct Pose2D { double x{0.0}, y{0.0}, yaw{0.0}; };

enum class JobState { BOOKED, RUNNING, DONE, FAILED, CANCELED, EXPIRED };

struct Job
{
  std::string id;
  std::string pickup;
  std::string dropoff;
  JobState state{JobState::BOOKED};
  rclcpp::Time booked_at;
};

class CourierNode : public rclcpp::Node
{
public:
  CourierNode() : Node("courier_node")
  {
    load_locations();

    booking_timeout_sec_ = declare_parameter<double>("booking_timeout_sec", 120.0);

    // ---- Booking service ----
    service_ = create_service<RequestDelivery>(
      "request_delivery",
      std::bind(&CourierNode::handle_request_delivery, this,
                std::placeholders::_1, std::placeholders::_2));

    // ---- Sweep bookings nobody ever ran ----
    expiry_timer_ = create_wall_timer(
      5s, std::bind(&CourierNode::expire_stale_bookings, this));

    RCLCPP_INFO(get_logger(), "courier_node: %zu known locations, ready for bookings.",
                locations_.size());
  }

private:
  // Reads 'location_names', then for each name reads 'locations.<name>.x/y/yaw'.
  // ROS 2 parameters cannot hold a list of structs, so this names-list plus
  // per-name sub-parameters is the pattern that gets around that.
  void load_locations()
  {
    auto names = declare_parameter<std::vector<std::string>>("location_names", {});
    if (names.empty()) {
      RCLCPP_FATAL(get_logger(),
        "No 'location_names' configured — refusing to start with no known locations.");
      throw std::runtime_error("no locations configured");
    }
    for (const auto & name : names) {
      Pose2D pose;
      pose.x   = declare_parameter<double>("locations." + name + ".x", 0.0);
      pose.y   = declare_parameter<double>("locations." + name + ".y", 0.0);
      pose.yaw = declare_parameter<double>("locations." + name + ".yaw", 0.0);
      locations_[name] = pose;
    }
  }

  std::string valid_location_list() const
  {
    std::string out;
    for (const auto & [name, pose] : locations_) {
      (void)pose;
      if (!out.empty()) out += ", ";
      out += name;
    }
    return out;
  }

  void handle_request_delivery(
    const std::shared_ptr<RequestDelivery::Request> request,
    std::shared_ptr<RequestDelivery::Response> response)
  {
    std::lock_guard<std::mutex> lock(jobs_mutex_);

    if (!locations_.count(request->pickup)) {
      response->accepted = false;
      response->job_id = "";
      response->reason = "unknown pickup '" + request->pickup +
        "' — known locations: " + valid_location_list();
      RCLCPP_WARN(get_logger(), "Rejected booking: %s", response->reason.c_str());
      return;
    }
    if (!locations_.count(request->dropoff)) {
      response->accepted = false;
      response->job_id = "";
      response->reason = "unknown dropoff '" + request->dropoff +
        "' — known locations: " + valid_location_list();
      RCLCPP_WARN(get_logger(), "Rejected booking: %s", response->reason.c_str());
      return;
    }
    if (request->pickup == request->dropoff) {
      response->accepted = false;
      response->job_id = "";
      response->reason = "pickup and dropoff must be different locations";
      RCLCPP_WARN(get_logger(), "Rejected booking: %s", response->reason.c_str());
      return;
    }

    std::string blocking_id;
    for (const auto & [id, job] : jobs_) {
      if (job.state == JobState::RUNNING) {
        blocking_id = id;
        break;
      }
    }
    if (!blocking_id.empty()) {
      response->accepted = false;
      response->job_id = "";
      response->reason = "busy, " + blocking_id + " is in progress";
      RCLCPP_WARN(get_logger(), "Rejected booking: %s", response->reason.c_str());
      return;
    }

    const std::string job_id = next_job_id();
    jobs_[job_id] = Job{job_id, request->pickup, request->dropoff, JobState::BOOKED, now()};

    response->accepted = true;
    response->job_id = job_id;
    response->reason = "booked";
    RCLCPP_INFO(get_logger(), "Booked %s: %s -> %s",
                job_id.c_str(), request->pickup.c_str(), request->dropoff.c_str());
  }

  std::string next_job_id()
  {
    char buf[16];
    std::snprintf(buf, sizeof(buf), "D-%04d", next_job_number_++);
    return std::string(buf);
  }

  void expire_stale_bookings()
  {
    std::lock_guard<std::mutex> lock(jobs_mutex_);
    for (auto & [id, job] : jobs_) {
      if (job.state == JobState::BOOKED &&
          (now() - job.booked_at).seconds() > booking_timeout_sec_) {
        job.state = JobState::EXPIRED;
        RCLCPP_WARN(get_logger(), "%s expired: booked but never run.", id.c_str());
      }
    }
  }

  std::map<std::string, Pose2D> locations_;
  std::map<std::string, Job> jobs_;
  std::mutex jobs_mutex_;
  int next_job_number_{1};
  double booking_timeout_sec_;

  rclcpp::Service<RequestDelivery>::SharedPtr service_;
  rclcpp::TimerBase::SharedPtr expiry_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CourierNode>());
  rclcpp::shutdown();
  return 0;
}
