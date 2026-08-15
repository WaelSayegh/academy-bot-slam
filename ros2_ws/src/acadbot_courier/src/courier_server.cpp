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

#include "rclcpp/rclcpp.hpp"

struct Location{
  double x;
  double y;
  double yaw;
};

class CourierServer : public rclcpp::Node{
public: CourierServer(): Node("courier_server"){

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
        RCLCPP_FATAL(
          get_logger(),
          "Location '%s' must contain exactly [x, y, yaw].",
          name.c_str());

        throw std::runtime_error(
          "invalid location configuration");
      }

      locations_[name] = {
        pose[0],
        pose[1],
        pose[2]
      };
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
  }

private: void validate_configuration(){
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
};

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