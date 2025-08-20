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

/*
 * Copyright 2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Authors: Simon Thompson, Ryohsuke Mitsudome
 *
 */

#include "lanelet2_map_visualization_node.hpp"

#include <autoware_lanelet2_extension/regulatory_elements/autoware_traffic_light.hpp>
#include <autoware_lanelet2_extension/utility/message_conversion.hpp>
#include <autoware_lanelet2_extension/utility/query.hpp>
#include <autoware_lanelet2_extension/visualization/visualization.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_projection/UTM.h>

#include <memory>
#include <vector>

namespace autoware::lanelet2_map_visualizer
{
void insert_marker_array(
  visualization_msgs::msg::MarkerArray * a1, const visualization_msgs::msg::MarkerArray & a2)
{
  a1->markers.insert(a1->markers.end(), a2.markers.begin(), a2.markers.end());
}

void set_color(std_msgs::msg::ColorRGBA * cl, double r, double g, double b, double a)
{
  cl->r = static_cast<float>(r);
  cl->g = static_cast<float>(g);
  cl->b = static_cast<float>(b);
  cl->a = static_cast<float>(a);
}

Lanelet2MapVisualizationNode::Lanelet2MapVisualizationNode(const rclcpp::NodeOptions & options)
: Node("lanelet2_map_visualization", options)
{
  using std::placeholders::_1;

  viz_lanelets_centerline_ = true;

  sub_map_bin_ = this->create_subscription<autoware_map_msgs::msg::LaneletMapBin>(
    "input/lanelet2_map", rclcpp::QoS{1}.transient_local(),
    std::bind(&Lanelet2MapVisualizationNode::on_map_bin, this, _1));

  sub_current_lanelet_ = this->create_subscription<std_msgs::msg::Int64>(
    "/current_lanelet_id", rclcpp::QoS{10},
    std::bind(&Lanelet2MapVisualizationNode::on_current_lanelet, this, _1));

  pub_marker_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "output/lanelet2_map_marker", rclcpp::QoS{1}.transient_local());
}

void Lanelet2MapVisualizationNode::on_map_bin(
  const autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr msg)
{
  viz_lanelet_map_.reset(new lanelet::LaneletMap);
  lanelet::utils::conversion::fromBinMsg(*msg, viz_lanelet_map_);
  RCLCPP_INFO(this->get_logger(), "Map is loaded\n");

  // get lanelets etc to visualize
  lanelet::ConstLanelets all_lanelets = lanelet::utils::query::laneletLayer(viz_lanelet_map_);
  lanelet::ConstLanelets road_lanelets = lanelet::utils::query::roadLanelets(all_lanelets);
  lanelet::ConstLanelets shoulder_lanelets = lanelet::utils::query::shoulderLanelets(all_lanelets);
  lanelet::ConstLanelets crosswalk_lanelets =
    lanelet::utils::query::crosswalkLanelets(all_lanelets);
  lanelet::ConstLanelets bicycle_lane_lanelets =
    lanelet::utils::query::bicycleLaneLanelets(all_lanelets);
  lanelet::ConstLineStrings3d partitions = lanelet::utils::query::getAllPartitions(viz_lanelet_map_);
  lanelet::ConstLineStrings3d pedestrian_polygon_markings =
    lanelet::utils::query::getAllPedestrianPolygonMarkings(viz_lanelet_map_);
  lanelet::ConstLineStrings3d pedestrian_line_markings =
    lanelet::utils::query::getAllPedestrianLineMarkings(viz_lanelet_map_);
  lanelet::ConstLanelets walkway_lanelets = lanelet::utils::query::walkwayLanelets(all_lanelets);
  std::vector<lanelet::ConstLineString3d> stop_lines =
    lanelet::utils::query::stopLinesLanelets(road_lanelets);
  // Also visualize raw lineStrings whose type is explicitly tagged as "stop_line"
  lanelet::ConstLineStrings3d raw_stop_lines;
  for (const auto & ls : viz_lanelet_map_->lineStringLayer) {
    const std::string type = ls.attributeOr(lanelet::AttributeName::Type, "none");
    if (type == "stop_line") {
      raw_stop_lines.push_back(static_cast<lanelet::ConstLineString3d>(ls));
    }
  }
  std::vector<lanelet::AutowareTrafficLightConstPtr> aw_tl_reg_elems =
    lanelet::utils::query::autowareTrafficLights(all_lanelets);
  std::vector<lanelet::DetectionAreaConstPtr> da_reg_elems =
    lanelet::utils::query::detectionAreas(all_lanelets);
  std::vector<lanelet::NoStoppingAreaConstPtr> no_reg_elems =
    lanelet::utils::query::noStoppingAreas(all_lanelets);
  std::vector<lanelet::SpeedBumpConstPtr> sb_reg_elems =
    lanelet::utils::query::speedBumps(all_lanelets);
  std::vector<lanelet::CrosswalkConstPtr> cw_reg_elems =
    lanelet::utils::query::crosswalks(all_lanelets);
  lanelet::ConstLineStrings3d parking_spaces =
    lanelet::utils::query::getAllParkingSpaces(viz_lanelet_map_);
  lanelet::ConstPolygons3d parking_lots = lanelet::utils::query::getAllParkingLots(viz_lanelet_map_);
  lanelet::ConstPolygons3d obstacle_polygons =
    lanelet::utils::query::getAllObstaclePolygons(viz_lanelet_map_);
  lanelet::ConstPolygons3d no_obstacle_segmentation_area =
    lanelet::utils::query::getAllPolygonsByType(viz_lanelet_map_, "no_obstacle_segmentation_area");
  lanelet::ConstPolygons3d no_obstacle_segmentation_area_for_run_out =
    lanelet::utils::query::getAllPolygonsByType(
      viz_lanelet_map_, "no_obstacle_segmentation_area_for_run_out");
  lanelet::ConstPolygons3d hatched_road_markings_area =
    lanelet::utils::query::getAllPolygonsByType(viz_lanelet_map_, "hatched_road_markings");
  lanelet::ConstPolygons3d intersection_areas =
    lanelet::utils::query::getAllPolygonsByType(viz_lanelet_map_, "intersection_area");
  std::vector<lanelet::NoParkingAreaConstPtr> no_parking_reg_elems =
    lanelet::utils::query::noParkingAreas(all_lanelets);
  lanelet::ConstLineStrings3d curbstones = lanelet::utils::query::curbstones(viz_lanelet_map_);
  std::vector<lanelet::BusStopAreaConstPtr> bus_stop_reg_elems =
    lanelet::utils::query::busStopAreas(all_lanelets);

  std_msgs::msg::ColorRGBA cl_road;
  std_msgs::msg::ColorRGBA cl_shoulder;
  std_msgs::msg::ColorRGBA cl_cross;
  std_msgs::msg::ColorRGBA cl_partitions;
  std_msgs::msg::ColorRGBA cl_pedestrian_markings;
  std_msgs::msg::ColorRGBA cl_ll_borders;
  std_msgs::msg::ColorRGBA cl_shoulder_borders;
  std_msgs::msg::ColorRGBA cl_stoplines;
  std_msgs::msg::ColorRGBA cl_trafficlights;
  std_msgs::msg::ColorRGBA cl_detection_areas;
  std_msgs::msg::ColorRGBA cl_speed_bumps;
  std_msgs::msg::ColorRGBA cl_crosswalks;
  std_msgs::msg::ColorRGBA cl_parking_lots;
  std_msgs::msg::ColorRGBA cl_parking_spaces;
  std_msgs::msg::ColorRGBA cl_lanelet_id;
  std_msgs::msg::ColorRGBA cl_obstacle_polygons;
  std_msgs::msg::ColorRGBA cl_no_stopping_areas;
  std_msgs::msg::ColorRGBA cl_no_obstacle_segmentation_area;
  std_msgs::msg::ColorRGBA cl_no_obstacle_segmentation_area_for_run_out;
  std_msgs::msg::ColorRGBA cl_hatched_road_markings_area;
  std_msgs::msg::ColorRGBA cl_hatched_road_markings_line;
  std_msgs::msg::ColorRGBA cl_no_parking_areas;
  std_msgs::msg::ColorRGBA cl_curbstones;
  std_msgs::msg::ColorRGBA cl_intersection_area;
  std_msgs::msg::ColorRGBA cl_bus_stop_area;
  std_msgs::msg::ColorRGBA cl_bicycle_lane;
  set_color(&cl_road, 0.27, 0.27, 0.27, 0.999);
  set_color(&cl_shoulder, 0.15, 0.15, 0.15, 0.999);
  set_color(&cl_cross, 0.27, 0.3, 0.27, 0.5);
  set_color(&cl_partitions, 0.25, 0.25, 0.25, 0.999);
  set_color(&cl_pedestrian_markings, 0.5, 0.5, 0.5, 0.999);
  set_color(&cl_ll_borders, 0.5, 0.5, 0.5, 0.999);
  set_color(&cl_shoulder_borders, 0.2, 0.2, 0.2, 0.999);
  set_color(&cl_stoplines, 0.85, 0.85, 0.85, 0.999);
  set_color(&cl_trafficlights, 0.5, 0.5, 0.5, 0.8);
  set_color(&cl_detection_areas, 0.27, 0.27, 0.37, 0.5);
  set_color(&cl_no_stopping_areas, 0.37, 0.37, 0.37, 0.5);
  set_color(&cl_speed_bumps, 0.56, 0.40, 0.27, 0.5);
  set_color(&cl_crosswalks, 0.80, 0.80, 0.0, 0.5);
  set_color(&cl_obstacle_polygons, 0.4, 0.27, 0.27, 0.5);
  set_color(&cl_parking_lots, 1.0, 1.0, 1.0, 0.2);
  set_color(&cl_parking_spaces, 1.0, 1.0, 1.0, 0.3);
  set_color(&cl_lanelet_id, 0.5, 0.5, 0.5, 0.999);
  set_color(&cl_no_obstacle_segmentation_area, 0.37, 0.37, 0.27, 0.5);
  set_color(&cl_no_obstacle_segmentation_area_for_run_out, 0.37, 0.7, 0.27, 0.5);
  set_color(&cl_hatched_road_markings_area, 0.3, 0.3, 0.3, 0.5);
  set_color(&cl_hatched_road_markings_line, 0.5, 0.5, 0.5, 0.999);
  set_color(&cl_no_parking_areas, 0.42, 0.42, 0.42, 0.5);
  set_color(&cl_curbstones, 0.1, 0.1, 0.2, 0.999);
  set_color(&cl_intersection_area, 0.16, 1.0, 0.69, 0.5);
  set_color(&cl_bus_stop_area, 0.863, 0.863, 0.863, 0.5);
  set_color(&cl_bicycle_lane, 0.0, 0.3843, 0.6274, 0.5);

  visualization_msgs::msg::MarkerArray map_marker_array;

  insert_marker_array(
    &map_marker_array,
    lanelet::visualization::lineStringsAsMarkerArray(stop_lines, "stop_lines", cl_stoplines, 0.5));
  insert_marker_array(
    &map_marker_array,
    lanelet::visualization::lineStringsAsMarkerArray(
      raw_stop_lines, "stop_lines_raw", cl_stoplines, 0.5));
  insert_marker_array(
    &map_marker_array,
    lanelet::visualization::lineStringsAsMarkerArray(partitions, "partitions", cl_partitions, 0.1));
  insert_marker_array(
    &map_marker_array,
    lanelet::visualization::laneletDirectionAsMarkerArray(shoulder_lanelets, "shoulder_"));
  insert_marker_array(
    &map_marker_array, lanelet::visualization::laneletDirectionAsMarkerArray(road_lanelets));
  insert_marker_array(
    &map_marker_array, lanelet::visualization::laneletsAsTriangleMarkerArray(
                         "crosswalk_lanelets", crosswalk_lanelets, cl_cross));
  insert_marker_array(
    &map_marker_array, lanelet::visualization::pedestrianPolygonMarkingsAsMarkerArray(
                         pedestrian_polygon_markings, cl_pedestrian_markings));

  insert_marker_array(
    &map_marker_array, lanelet::visualization::pedestrianLineMarkingsAsMarkerArray(
                         pedestrian_line_markings, cl_pedestrian_markings));
  insert_marker_array(
    &map_marker_array, lanelet::visualization::laneletsAsTriangleMarkerArray(
                         "walkway_lanelets", walkway_lanelets, cl_cross));
  insert_marker_array(
    &map_marker_array,
    lanelet::visualization::obstaclePolygonsAsMarkerArray(obstacle_polygons, cl_obstacle_polygons));
  insert_marker_array(
    &map_marker_array,
    lanelet::visualization::detectionAreasAsMarkerArray(da_reg_elems, cl_detection_areas));
  insert_marker_array(
    &map_marker_array,
    lanelet::visualization::noStoppingAreasAsMarkerArray(no_reg_elems, cl_no_stopping_areas));
  insert_marker_array(
    &map_marker_array,
    lanelet::visualization::speedBumpsAsMarkerArray(sb_reg_elems, cl_speed_bumps));
  insert_marker_array(
    &map_marker_array,
    lanelet::visualization::crosswalkAreasAsMarkerArray(cw_reg_elems, cl_crosswalks));
  insert_marker_array(
    &map_marker_array,
    lanelet::visualization::parkingLotsAsMarkerArray(parking_lots, cl_parking_lots));
  insert_marker_array(
    &map_marker_array,
    lanelet::visualization::parkingSpacesAsMarkerArray(parking_spaces, cl_parking_spaces));
  insert_marker_array(
    &map_marker_array,
    lanelet::visualization::laneletsBoundaryAsMarkerArray(
      shoulder_lanelets, cl_shoulder_borders, viz_lanelets_centerline_, "shoulder_"));
  insert_marker_array(
    &map_marker_array, lanelet::visualization::laneletsBoundaryAsMarkerArray(
                         road_lanelets, cl_ll_borders, viz_lanelets_centerline_));
  insert_marker_array(
    &map_marker_array,
    lanelet::visualization::autowareTrafficLightsAsMarkerArray(aw_tl_reg_elems, cl_trafficlights));
  insert_marker_array(
    &map_marker_array, lanelet::visualization::generateTrafficLightRegulatoryElementIdMaker(
                         road_lanelets, cl_trafficlights));
  insert_marker_array(
    &map_marker_array, lanelet::visualization::generateTrafficLightRegulatoryElementIdMaker(
                         crosswalk_lanelets, cl_trafficlights));
  insert_marker_array(
    &map_marker_array,
    lanelet::visualization::generateTrafficLightIdMaker(aw_tl_reg_elems, cl_trafficlights));
  insert_marker_array(
    &map_marker_array,
    lanelet::visualization::generateLaneletIdMarker(shoulder_lanelets, cl_lanelet_id));
  insert_marker_array(
    &map_marker_array,
    lanelet::visualization::generateLaneletIdMarker(road_lanelets, cl_lanelet_id));
  insert_marker_array(
    &map_marker_array, lanelet::visualization::generateLaneletIdMarker(
                         crosswalk_lanelets, cl_lanelet_id, "crosswalk_lanelet_id"));
  insert_marker_array(
    &map_marker_array, lanelet::visualization::laneletsAsTriangleMarkerArray(
                         "shoulder_road_lanelets", shoulder_lanelets, cl_shoulder));
  insert_marker_array(
    &map_marker_array,
    lanelet::visualization::laneletsAsTriangleMarkerArray("road_lanelets", road_lanelets, cl_road));
  insert_marker_array(
    &map_marker_array, lanelet::visualization::noObstacleSegmentationAreaAsMarkerArray(
                         no_obstacle_segmentation_area, cl_no_obstacle_segmentation_area));
  insert_marker_array(
    &map_marker_array,
    lanelet::visualization::noObstacleSegmentationAreaForRunOutAsMarkerArray(
      no_obstacle_segmentation_area_for_run_out, cl_no_obstacle_segmentation_area_for_run_out));

  insert_marker_array(
    &map_marker_array,
    lanelet::visualization::hatchedRoadMarkingsAreaAsMarkerArray(
      hatched_road_markings_area, cl_hatched_road_markings_area, cl_hatched_road_markings_line));

  insert_marker_array(
    &map_marker_array,
    lanelet::visualization::noParkingAreasAsMarkerArray(no_parking_reg_elems, cl_no_parking_areas));

  insert_marker_array(
    &map_marker_array,
    lanelet::visualization::lineStringsAsMarkerArray(curbstones, "curbstone", cl_curbstones, 0.2));

  insert_marker_array(
    &map_marker_array, lanelet::visualization::intersectionAreaAsMarkerArray(
                         intersection_areas, cl_intersection_area));

  insert_marker_array(
    &map_marker_array,
    lanelet::visualization::busStopAreasAsMarkerArray(bus_stop_reg_elems, cl_bus_stop_area));

  insert_marker_array(
    &map_marker_array,
    lanelet::visualization::laneletDirectionAsMarkerArray(bicycle_lane_lanelets, "bicycle_lane_"));
  insert_marker_array(
    &map_marker_array, lanelet::visualization::laneletsBoundaryAsMarkerArray(
                         bicycle_lane_lanelets, cl_ll_borders /* use ll_border color */,
                         viz_lanelets_centerline_, "bicycle_lane_"));
  insert_marker_array(
    &map_marker_array, lanelet::visualization::generateLaneletIdMarker(
                         bicycle_lane_lanelets, cl_lanelet_id /* use lanelet_id color */));
  insert_marker_array(
    &map_marker_array, lanelet::visualization::laneletsAsTriangleMarkerArray(
                         "bicycle_lane_lanelets", bicycle_lane_lanelets, cl_bicycle_lane));

  // Z-offset adjustments to mitigate z-fighting
  auto adjust_z_by_namespace = [&](const std::string & ns, const double dz) {
    for (auto & mk : map_marker_array.markers) {
      if (mk.ns == ns) {
        for (auto & pt : mk.points) {
          pt.z += dz;
        }
        mk.pose.position.z += dz;
      }
    }
  };

  // Lower road surface slightly and raise stop lines slightly
  adjust_z_by_namespace("road_lanelets", -0.05);
  adjust_z_by_namespace("stop_lines", +0.05);
  adjust_z_by_namespace("stop_lines_raw", +0.05);

  pub_marker_->publish(map_marker_array);
}

void Lanelet2MapVisualizationNode::on_current_lanelet(const std_msgs::msg::Int64::ConstSharedPtr msg)
{
  current_lanelet_id_ = msg->data;

  // First publish a message that deletes previous highlight markers
  {
    visualization_msgs::msg::MarkerArray delete_array;
    for (int id = 1; id <= 2; ++id) {
      visualization_msgs::msg::Marker del_marker;
      del_marker.header.frame_id = "map";
      del_marker.header.stamp = this->now();
      del_marker.ns = "current_lanelet_highlight";
      del_marker.id = id;
      del_marker.action = visualization_msgs::msg::Marker::DELETE;
      delete_array.markers.push_back(del_marker);
    }
    pub_marker_->publish(delete_array);
  }

  visualization_msgs::msg::MarkerArray marker_array;

  if (!viz_lanelet_map_) {
    pub_marker_->publish(marker_array);
    return;
  }

  // Find lanelet by id
  lanelet::ConstLanelet lanelet_obj;
  try {
    lanelet_obj = viz_lanelet_map_->laneletLayer.get(static_cast<lanelet::Id>(current_lanelet_id_));
  } catch (const std::exception & e) {
    // No such lanelet, publish only clear
    pub_marker_->publish(marker_array);
    return;
  }

  // Prepare a base marker style for highlight
  auto make_base = [this]() {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = "map"; // keep consistent with base visualizer
    m.header.stamp = this->now();
    m.ns = "current_lanelet_highlight";
    m.action = visualization_msgs::msg::Marker::ADD;
    m.type = visualization_msgs::msg::Marker::LINE_STRIP;
    m.scale.x = 0.35; // thicker than default borders
    m.color.a = 1.0;
    m.color.r = 1.0;
    m.color.g = 0.6;
    m.color.b = 0.0; // orange/yellow
    return m;
  };

  // Left bound only
  {
    visualization_msgs::msg::Marker left_mk = make_base();
    left_mk.id = 1;
    for (const auto & p : lanelet_obj.leftBound().basicLineString()) {
      geometry_msgs::msg::Point pt;
      pt.x = p.x();
      pt.y = p.y();
      pt.z = p.z();
      left_mk.points.push_back(pt);
    }
    marker_array.markers.push_back(left_mk);
  }

  // Right bound only
  {
    visualization_msgs::msg::Marker right_mk = make_base();
    right_mk.id = 2;
    for (const auto & p : lanelet_obj.rightBound().basicLineString()) {
      geometry_msgs::msg::Point pt;
      pt.x = p.x();
      pt.y = p.y();
      pt.z = p.z();
      right_mk.points.push_back(pt);
    }
    marker_array.markers.push_back(right_mk);
  }

  pub_marker_->publish(marker_array);
}
}  // namespace autoware::lanelet2_map_visualizer

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::lanelet2_map_visualizer::Lanelet2MapVisualizationNode)
