// Copyright 2021 TierIV
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

#include "autoware/map_loader/lanelet2_map_loader_node.hpp"

#include "lanelet2_local_projector.hpp"

#include <ament_index_cpp/get_package_prefix.hpp>
#include <autoware/geography_utils/lanelet2_projector.hpp>
#include <autoware_lanelet2_extension/io/autoware_osm_parser.hpp>
#include <autoware_lanelet2_extension/projection/mgrs_projector.hpp>
#include <autoware_lanelet2_extension/projection/transverse_mercator_projector.hpp>
#include <autoware_lanelet2_extension/utility/message_conversion.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>
#include <rclcpp/rclcpp.hpp>

#include <yaml-cpp/yaml.h>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>

#include <memory>
#include <stdexcept>
#include <string>

namespace autoware::map_loader
{
using autoware_map_msgs::msg::LaneletMapBin;
// Do not use `using` for MapProjectorInfo to avoid conflicts.

// Function to parse the projector info yaml file
autoware_map_msgs::msg::MapProjectorInfo parse_projector_info(
  const std::string & map_path, rclcpp::Logger logger)
{
  autoware_map_msgs::msg::MapProjectorInfo projector_info;
  const std::string projector_info_path =
    map_path.substr(0, map_path.find_last_of('/')) + "/map_projector_info.yaml";

  try {
    RCLCPP_INFO(logger, "Parsing map_projector_info.yaml from: %s", projector_info_path.c_str());
    YAML::Node config = YAML::LoadFile(projector_info_path);
    const std::string projector_type = config["projector_type"].as<std::string>();

    if (projector_type == "MGRS") {
      projector_info.projector_type = autoware_map_msgs::msg::MapProjectorInfo::MGRS;
      projector_info.mgrs_grid = config["mgrs_grid"].as<std::string>();
    } else if (projector_type == "Local") {
      projector_info.projector_type = autoware_map_msgs::msg::MapProjectorInfo::LOCAL;
    } else {
      throw std::runtime_error("Unsupported projector_type: " + projector_type);
    }
  } catch (const YAML::Exception & e) {
    throw std::runtime_error(
      "Failed to load or parse map_projector_info.yaml: " + std::string(e.what()));
  }

  return projector_info;
}

Lanelet2MapLoaderNode::Lanelet2MapLoaderNode(const rclcpp::NodeOptions & options)
: Node("lanelet2_map_loader", options)
{
  declare_parameter<bool>("allow_unsupported_version", true);
  declare_parameter<std::string>("lanelet2_map_path", "");
  declare_parameter<double>("center_line_resolution", 5.0);
  declare_parameter<bool>("use_waypoints", true);
  declare_parameter<std::string>("map_projector_type", "UTM");
  declare_parameter<double>("latitude", 0.0);
  declare_parameter<double>("longitude", 0.0);

  // Load map directly in constructor
  const auto allow_unsupported_version = get_parameter("allow_unsupported_version").as_bool();
  const auto lanelet2_filename = get_parameter("lanelet2_map_path").as_string();
  const auto center_line_resolution = get_parameter("center_line_resolution").as_double();
  const auto use_waypoints = get_parameter("use_waypoints").as_bool();
  const auto projector_type = get_parameter("map_projector_type").as_string();

  RCLCPP_INFO(get_logger(), "Received map path parameter: '%s'", lanelet2_filename.c_str());

  if (lanelet2_filename.empty()) {
    RCLCPP_ERROR(get_logger(), "Parameter 'lanelet2_map_path' is not set or empty. Map not loaded.");
    return;
  }
  RCLCPP_INFO(get_logger(), "Loading map from: %s", lanelet2_filename.c_str());

  lanelet::LaneletMapPtr map = nullptr;
  lanelet::ErrorMessages errors{};

  if (projector_type == "UTM") {
    RCLCPP_INFO(get_logger(), "Using UTM projector from launch parameters.");
    const auto latitude = get_parameter("latitude").as_double();
    const auto longitude = get_parameter("longitude").as_double();
    if (latitude == 0.0 || longitude == 0.0) {
      RCLCPP_ERROR(get_logger(), "latitude and longitude are not set for UTM projector.");
      return;
    }
    lanelet::projection::UtmProjector projector{lanelet::Origin({latitude, longitude})};
    map = lanelet::load(lanelet2_filename, projector, &errors);
  } else {
    // Parse projector info from yaml
    autoware_map_msgs::msg::MapProjectorInfo projector_info;
    try {
      projector_info = parse_projector_info(lanelet2_filename, get_logger());
    } catch (const std::runtime_error & e) {
      RCLCPP_ERROR(get_logger(), "%s", e.what());
      return;
    }
    map = load_map(lanelet2_filename, projector_info);
  }

  if (!map) {
    RCLCPP_ERROR(get_logger(), "Failed to load lanelet2_map.");
    for (const auto & error : errors) {
      RCLCPP_ERROR_STREAM(get_logger(), error);
    }
    return;
  }

  std::string format_version{"null"}, map_version{""};
  lanelet::io_handlers::AutowareOsmParser::parseVersions(
    lanelet2_filename, &format_version, &map_version);
  if (format_version == "null" || format_version.empty() || !isdigit(format_version[0])) {
    RCLCPP_WARN(
      get_logger(),
      "%s has no format_version(null) or non semver-style format_version(%s) information",
      lanelet2_filename.c_str(), format_version.c_str());
    if (!allow_unsupported_version) {
      throw std::invalid_argument(
        "allow_unsupported_version is false, so stop loading lanelet map");
    }
  } else if (const auto map_major_ver_opt = lanelet::io_handlers::parseMajorVersion(format_version);
             map_major_ver_opt.has_value()) {
    const auto map_major_ver = map_major_ver_opt.value();
    if (map_major_ver > static_cast<uint64_t>(lanelet::autoware::version)) {
      RCLCPP_WARN(
        get_logger(),
        "format_version(%ld) of the provided map(%s) is larger than the supported version(%ld)",
        map_major_ver, lanelet2_filename.c_str(),
        static_cast<uint64_t>(lanelet::autoware::version));
      if (!allow_unsupported_version) {
        throw std::invalid_argument(
          "allow_unsupported_version is false, so stop loading lanelet map");
      }
    }
  }
  RCLCPP_INFO(get_logger(), "Loaded map format_version: %s", format_version.c_str());

  // overwrite centerline
  if (use_waypoints) {
    lanelet::utils::overwriteLaneletsCenterlineWithWaypoints(map, center_line_resolution, false);
  } else {
    lanelet::utils::overwriteLaneletsCenterline(map, center_line_resolution, false);
  }

  // create map bin msg
  const auto map_bin_msg = create_map_bin_msg(map, lanelet2_filename, now());

  // create publisher and publish
  pub_map_bin_ =
    create_publisher<VectorMap::Message>(VectorMap::name, rclcpp::QoS{1}.transient_local());
  pub_map_bin_->publish(map_bin_msg);
  RCLCPP_INFO(get_logger(), "Succeeded to load lanelet2_map. Map is published.");
}

lanelet::LaneletMapPtr Lanelet2MapLoaderNode::load_map(
  const std::string & lanelet2_filename,
  const autoware_map_msgs::msg::MapProjectorInfo & projector_info)
{
  lanelet::ErrorMessages errors{};
  if (projector_info.projector_type != autoware_map_msgs::msg::MapProjectorInfo::LOCAL) {
    std::unique_ptr<lanelet::Projector> projector =
      autoware::geography_utils::get_lanelet2_projector(projector_info);
    lanelet::LaneletMapPtr map = lanelet::load(lanelet2_filename, *projector, &errors);
    if (errors.empty()) {
      return map;
    }
  } else {
    const autoware::map_loader::LocalProjector projector;
    lanelet::LaneletMapPtr map = lanelet::load(lanelet2_filename, projector, &errors);

    if (!errors.empty()) {
      for (const auto & error : errors) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("map_loader"), error);
      }
    }

    // overwrite local_x, local_y
    for (lanelet::Point3d point : map->pointLayer) {
      if (point.hasAttribute("local_x")) {
        point.x() = point.attribute("local_x").asDouble().value();
      }
      if (point.hasAttribute("local_y")) {
        point.y() = point.attribute("local_y").asDouble().value();
      }
    }

    // realign lanelet borders using updated points
    for (lanelet::Lanelet lanelet : map->laneletLayer) {
      auto left = lanelet.leftBound();
      auto right = lanelet.rightBound();
      std::tie(left, right) = lanelet::geometry::align(left, right);
      lanelet.setLeftBound(left);
      lanelet.setRightBound(right);
    }

    return map;
  }

  for (const auto & error : errors) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("map_loader"), error);
  }
  return nullptr;
}

LaneletMapBin Lanelet2MapLoaderNode::create_map_bin_msg(
  const lanelet::LaneletMapPtr map, const std::string & lanelet2_filename, const rclcpp::Time & now)
{
  std::string format_version{};
  std::string map_version{};
  lanelet::io_handlers::AutowareOsmParser::parseVersions(
    lanelet2_filename, &format_version, &map_version);

  LaneletMapBin map_bin_msg;
  map_bin_msg.header.stamp = now;
  map_bin_msg.header.frame_id = "map";
  map_bin_msg.version_map_format = format_version;
  map_bin_msg.version_map = map_version;
  lanelet::utils::conversion::toBinMsg(map, &map_bin_msg);

  return map_bin_msg;
}
}  // namespace autoware::map_loader

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::map_loader::Lanelet2MapLoaderNode)
