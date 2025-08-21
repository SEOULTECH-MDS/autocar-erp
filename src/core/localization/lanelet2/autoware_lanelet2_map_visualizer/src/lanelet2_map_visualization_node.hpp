// Copyright 2021 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef LANELET2_MAP_VISUALIZATION_NODE_HPP_
#define LANELET2_MAP_VISUALIZATION_NODE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/int64.hpp>

#include <lanelet2_core/LaneletMap.h>

#include <string>
#include <vector>

namespace autoware::lanelet2_map_visualizer
{
class Lanelet2MapVisualizationNode : public rclcpp::Node
{
public:
  explicit Lanelet2MapVisualizationNode(const rclcpp::NodeOptions & options);

private:
  rclcpp::Subscription<autoware_map_msgs::msg::LaneletMapBin>::SharedPtr sub_map_bin_;
  rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr sub_current_lanelet_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_marker_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_highlight_;

  bool viz_lanelets_centerline_;
  int64_t current_lanelet_id_ = 0;
  lanelet::LaneletMapPtr viz_lanelet_map_;

  void on_map_bin(const autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr msg);
  void on_current_lanelet(const std_msgs::msg::Int64::ConstSharedPtr msg);
};
}  // namespace autoware::lanelet2_map_visualizer

#endif  // LANELET2_MAP_VISUALIZATION_NODE_HPP_
