// Draws the location table in RViz, so a badly placed location is visible
// before it costs a delivery.
#ifndef ACADBOT_COURIER__LOCATION_MARKERS_HPP_
#define ACADBOT_COURIER__LOCATION_MARKERS_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "acadbot_courier/location_table.hpp"

namespace acadbot_courier
{

/// Builds the whole array; `active` is the location being driven to, drawn
/// larger, or empty when no job is running.
visualization_msgs::msg::MarkerArray make_location_markers(
  const LocationTable & table,
  const std::string & frame_id,
  const rclcpp::Time & stamp,
  const std::string & active = "");

}  // namespace acadbot_courier

#endif  // ACADBOT_COURIER__LOCATION_MARKERS_HPP_
