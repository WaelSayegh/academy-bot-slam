#ifndef ACADBOT_COURIER__TYPES_HPP_
#define ACADBOT_COURIER__TYPES_HPP_

#include <cstdint>
#include <string>

namespace acadbot_courier
{

inline constexpr char kOutcomeSucceeded[] = "SUCCEEDED";
inline constexpr char kOutcomeCanceled[] = "CANCELED";
inline constexpr char kOutcomeNavAborted[] = "NAV_ABORTED";
inline constexpr char kOutcomeTimeout[] = "TIMEOUT";
inline constexpr char kOutcomeInvalidJob[] = "INVALID_JOB";

inline constexpr char kStateNavigating[] = "NAVIGATING";
inline constexpr char kStateRetrying[] = "RETRYING";
inline constexpr char kStateRecovering[] = "RECOVERING";
inline constexpr char kStateDwelling[] = "DWELLING";

inline constexpr char kLegPickup[] = "pickup";
inline constexpr char kLegDropoff[] = "dropoff";

struct Leg
{
  std::string name;
  std::string location;
  double dwell_sec;
};

struct FeedbackSnapshot
{
  std::string job_id;
  std::string leg;
  std::string target_location;
  double distance_remaining{-1.0};
  std::uint16_t attempt{0};
  std::string state;
  double elapsed_sec{0.0};
};

enum class NavResult
{
  SUCCEEDED,
  ABORTED,
  CANCELED,
  TIMEOUT
};

}  // namespace acadbot_courier

#endif  // ACADBOT_COURIER__TYPES_HPP_
