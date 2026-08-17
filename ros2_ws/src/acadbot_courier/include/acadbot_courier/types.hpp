#ifndef ACADBOT_COURIER__TYPES_HPP_
#define ACADBOT_COURIER__TYPES_HPP_

#include <string>

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

#endif  // ACADBOT_COURIER__TYPES_HPP_
