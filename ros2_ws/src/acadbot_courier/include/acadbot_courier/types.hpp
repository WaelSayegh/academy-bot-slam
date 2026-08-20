// The vocabulary a delivery is described in, kept apart from the node that
// runs one.
#ifndef ACADBOT_COURIER__TYPES_HPP_
#define ACADBOT_COURIER__TYPES_HPP_

#include <string>

namespace acadbot_courier
{

/// Where to stand and which way to face; yaw rather than a quaternion because
/// that is how YAML writes it.
struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

/// A delivery is always these two legs, in this order.
enum class Leg
{
  PICKUP,
  DROPOFF,
};

/// Spelled once, because the feedback and the result's failed_leg must agree.
inline const char * to_string(Leg leg)
{
  return leg == Leg::PICKUP ? "pickup" : "dropoff";
}

/// Terminal states are remembered, so a replayed job id is refused rather than
/// delivered twice.
enum class JobState
{
  BOOKED,      // accepted by the service, waiting for an action goal
  RUNNING,
  SUCCEEDED,
  FAILED,
  CANCELED,
};

inline const char * to_string(JobState state)
{
  switch (state) {
    case JobState::BOOKED:    return "BOOKED";
    case JobState::RUNNING:   return "RUNNING";
    case JobState::SUCCEEDED: return "SUCCEEDED";
    case JobState::FAILED:    return "FAILED";
    case JobState::CANCELED:  return "CANCELED";
  }
  return "UNKNOWN";
}

/// A booked delivery, holding both the names asked for and the poses resolved
/// at booking -- resolving again later would let a YAML edit move the target.
struct Job
{
  std::string id;
  std::string pickup_name;
  std::string dropoff_name;
  Pose2D pickup;
  Pose2D dropoff;
  JobState state{JobState::BOOKED};

  const std::string & name_of(Leg leg) const
  {
    return leg == Leg::PICKUP ? pickup_name : dropoff_name;
  }

  const Pose2D & pose_of(Leg leg) const
  {
    return leg == Leg::PICKUP ? pickup : dropoff;
  }
};

}  // namespace acadbot_courier

#endif  // ACADBOT_COURIER__TYPES_HPP_
