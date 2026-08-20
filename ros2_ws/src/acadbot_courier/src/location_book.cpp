#include "acadbot_courier/location_book.hpp"

#include <stdexcept>
#include <utility>

#include "rclcpp/clock.hpp"
#include "tf2/LinearMath/Quaternion.h"

namespace acadbot_courier
{

LocationBook::LocationBook(
  std::vector<std::string> ordered_names,
  std::unordered_map<std::string, LocationPose> locations,
  std::string global_frame)
: ordered_names_(std::move(ordered_names)),
  locations_(std::move(locations)),
  global_frame_(std::move(global_frame))
{
}

bool LocationBook::has(const std::string & name) const
{
  return locations_.find(name) != locations_.end();
}

geometry_msgs::msg::PoseStamped LocationBook::pose(const std::string & name) const
{
  const auto found = locations_.find(name);
  if (found == locations_.end()) {
    throw std::out_of_range("unknown courier location: " + name);
  }

  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = global_frame_;
  pose.header.stamp = rclcpp::Clock(RCL_SYSTEM_TIME).now();
  pose.pose.position.x = found->second.x;
  pose.pose.position.y = found->second.y;

  tf2::Quaternion orientation;
  orientation.setRPY(0.0, 0.0, found->second.yaw);
  pose.pose.orientation.x = orientation.x();
  pose.pose.orientation.y = orientation.y();
  pose.pose.orientation.z = orientation.z();
  pose.pose.orientation.w = orientation.w();
  return pose;
}

std::vector<std::string> LocationBook::names() const
{
  return ordered_names_;
}

}  // namespace acadbot_courier
