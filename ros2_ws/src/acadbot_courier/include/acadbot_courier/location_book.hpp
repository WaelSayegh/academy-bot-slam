#ifndef ACADBOT_COURIER__LOCATION_BOOK_HPP_
#define ACADBOT_COURIER__LOCATION_BOOK_HPP_

#include <string>
#include <unordered_map>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"

namespace acadbot_courier
{

struct LocationPose
{
  double x;
  double y;
  double yaw;
};

class LocationBook
{
public:
  LocationBook(
    std::vector<std::string> ordered_names,
    std::unordered_map<std::string, LocationPose> locations,
    std::string global_frame);

  bool has(const std::string & name) const;
  geometry_msgs::msg::PoseStamped pose(const std::string & name) const;
  std::vector<std::string> names() const;

private:
  std::vector<std::string> ordered_names_;
  std::unordered_map<std::string, LocationPose> locations_;
  std::string global_frame_;
};

}  // namespace acadbot_courier

#endif  // ACADBOT_COURIER__LOCATION_BOOK_HPP_
