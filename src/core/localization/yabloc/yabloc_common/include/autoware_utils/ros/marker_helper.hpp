// Minimal shim for autoware_utils::create_marker_color used by YabLoc
#ifndef AUTOWARE_UTILS__ROS__MARKER_HELPER_HPP_
#define AUTOWARE_UTILS__ROS__MARKER_HELPER_HPP_

#include <std_msgs/msg/color_rgba.hpp>

namespace autoware_utils
{
inline std_msgs::msg::ColorRGBA create_marker_color(float r, float g, float b, float a)
{
  std_msgs::msg::ColorRGBA c;
  c.r = r;
  c.g = g;
  c.b = b;
  c.a = a;
  return c;
}
}  // namespace autoware_utils

#endif  // AUTOWARE_UTILS__ROS__MARKER_HELPER_HPP_


