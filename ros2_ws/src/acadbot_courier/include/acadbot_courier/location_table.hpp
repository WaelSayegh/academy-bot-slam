// The name -> pose table, and the only class that knows a location has
// coordinates at all.
#ifndef ACADBOT_COURIER__LOCATION_TABLE_HPP_
#define ACADBOT_COURIER__LOCATION_TABLE_HPP_

#include <map>
#include <optional>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "acadbot_courier/types.hpp"

namespace acadbot_courier
{

class LocationTable
{
public:
  /// Reads every location parameter on `node`, throwing on an empty or
  /// malformed floor plan so the mistake stops startup rather than a delivery.
  static LocationTable from_parameters(rclcpp::Node & node);

  /// The pose for `name`, or nothing if no such location is configured.
  std::optional<Pose2D> find(const std::string & name) const;

  /// The configured names, for the rejection message.
  std::string known_names() const;

  /// Every location, for drawing the markers.
  const std::map<std::string, Pose2D> & all() const { return poses_; }

  std::size_t size() const { return poses_.size(); }

private:
  std::map<std::string, Pose2D> poses_;
};

}  // namespace acadbot_courier

#endif  // ACADBOT_COURIER__LOCATION_TABLE_HPP_
