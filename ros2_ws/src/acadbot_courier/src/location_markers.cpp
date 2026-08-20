#include "acadbot_courier/location_markers.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace acadbot_courier
{

namespace
{
constexpr double kArrowLength = 0.40;   // m, along the location's heading
constexpr double kArrowWidth = 0.08;    // m
constexpr double kActiveScale = 1.5;    // how much bigger the current target is
constexpr double kArrowZ = 0.05;        // just clear of the costmap
constexpr double kLabelZ = 0.35;        // above the robot, so it stays legible
constexpr double kLabelHeight = 0.22;   // text cap height, m
}  // namespace

visualization_msgs::msg::MarkerArray make_location_markers(
  const LocationTable & table,
  const std::string & frame_id,
  const rclcpp::Time & stamp,
  const std::string & active)
{
  visualization_msgs::msg::MarkerArray array;
  int id = 0;

  // Map order is stable, so a location keeps its id across republishes and
  // its marker is replaced rather than duplicated.
  for (const auto & kv : table.all()) {
    const std::string & name = kv.first;
    const Pose2D & pose = kv.second;
    const bool is_active = !active.empty() && name == active;
    const double scale = is_active ? kActiveScale : 1.0;

    visualization_msgs::msg::Marker arrow;
    arrow.header.frame_id = frame_id;
    arrow.header.stamp = stamp;
    arrow.ns = "courier_locations";
    arrow.id = id;
    arrow.type = visualization_msgs::msg::Marker::ARROW;
    arrow.action = visualization_msgs::msg::Marker::ADD;

    arrow.pose.position.x = pose.x;
    arrow.pose.position.y = pose.y;
    arrow.pose.position.z = kArrowZ;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, pose.yaw);
    arrow.pose.orientation = tf2::toMsg(q);

    arrow.scale.x = kArrowLength * scale;
    arrow.scale.y = kArrowWidth * scale;
    arrow.scale.z = kArrowWidth * scale;

    if (is_active) {
      arrow.color.r = 0.20f;
      arrow.color.g = 0.85f;
      arrow.color.b = 0.30f;
    } else {
      arrow.color.r = 0.15f;
      arrow.color.g = 0.55f;
      arrow.color.b = 0.90f;
    }
    arrow.color.a = 0.95f;

    // Zero lifetime means "until replaced", and same ns + id overwrites.
    arrow.lifetime = rclcpp::Duration(0, 0);

    // A separate namespace so labels can be switched off without losing the
    // arrows; TEXT_VIEW_FACING ignores orientation and turns to the camera.
    auto label = arrow;
    label.ns = "courier_location_labels";
    label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    label.text = name;
    label.pose.position.z = kLabelZ;
    label.scale.z = kLabelHeight;
    label.color.r = 1.0f;
    label.color.g = 1.0f;
    label.color.b = 1.0f;

    array.markers.push_back(arrow);
    array.markers.push_back(label);
    ++id;
  }

  return array;
}

}  // namespace acadbot_courier
