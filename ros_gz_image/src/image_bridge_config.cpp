// Copyright 2026 Open Source Robotics Foundation, Inc.
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

#include "image_bridge_config.hpp"

#include <yaml-cpp/yaml.h>

#include <fstream>
#include <sstream>
#include <string>

#include <rclcpp/rclcpp.hpp>

namespace ros_gz_image
{
namespace
{
// YAML tag string constants
constexpr const char kTopicName[] = "topic_name";
constexpr const char kRosTopicName[] = "ros_topic_name";
constexpr const char kGzTopicName[] = "gz_topic_name";
constexpr const char kRosTypeName[] = "ros_type_name";
constexpr const char kGzTypeName[] = "gz_type_name";
constexpr const char kPublisherQueue[] = "publisher_queue";
constexpr const char kQosProfile[] = "qos_profile";
constexpr const char kLazy[] = "lazy";
constexpr const char kFrameId[] = "frame_id";
constexpr const char kCameraInfo[] = "camera_info";

// Default message types of image_bridge
constexpr const char kRosImageType[] = "sensor_msgs/msg/Image";
constexpr const char kGzImageType[] = "gz.msgs.Image";

// Suffix of camera_info topics derived from the image topic names
constexpr const char kCameraInfoSuffix[] = "/camera_info";

/// \brief Read an optional string value from a YAML map
/// \param[in] node YAML map node
/// \param[in] key Tag to read
/// \return The value, or an empty string when unset
std::string getValue(const YAML::Node & node, const char * key)
{
  if (!node[key]) {
    return "";
  }
  return node[key].as<std::string>();
}

/// \brief Resolve the image topic names of an entry
/// \param[in] node YAML map node of the entry
/// \param[out] ros_topic ROS image topic name
/// \param[out] gz_topic Gazebo image topic name
/// \return false when the names are invalid, with an error logged
bool resolveTopicNames(
  const YAML::Node & node,
  std::string & ros_topic,
  std::string & gz_topic)
{
  auto logger = rclcpp::get_logger("image_bridge_config");

  const auto topic_name = getValue(node, kTopicName);
  const auto ros_topic_name = getValue(node, kRosTopicName);
  const auto gz_topic_name = getValue(node, kGzTopicName);

  if (!topic_name.empty() && !ros_topic_name.empty()) {
    RCLCPP_ERROR(
      logger, "Could not parse entry: %s and %s are mutually exclusive",
      kTopicName, kRosTopicName);
    return false;
  }
  if (!topic_name.empty() && !gz_topic_name.empty()) {
    RCLCPP_ERROR(
      logger, "Could not parse entry: %s and %s are mutually exclusive",
      kTopicName, kGzTopicName);
    return false;
  }

  if (!topic_name.empty()) {
    // Only topic_name is set
    ros_topic = topic_name;
    gz_topic = topic_name;
  } else if (!ros_topic_name.empty() && gz_topic_name.empty()) {
    // Only ros_topic_name is set
    ros_topic = ros_topic_name;
    gz_topic = ros_topic_name;
  } else if (!gz_topic_name.empty() && ros_topic_name.empty()) {
    // Only gz_topic_name is set
    ros_topic = gz_topic_name;
    gz_topic = gz_topic_name;
  } else if (!ros_topic_name.empty() && !gz_topic_name.empty()) {
    // Both are set
    ros_topic = ros_topic_name;
    gz_topic = gz_topic_name;
  } else {
    RCLCPP_ERROR(
      logger, "Could not parse entry: one of %s, %s or %s must be set",
      kTopicName, kRosTopicName, kGzTopicName);
    return false;
  }
  return true;
}

/// \brief Parse the camera_info sub-configuration of an entry
/// \param[in] node YAML map node of the entry
/// \param[in] ros_topic ROS image topic name of the entry
/// \param[in] gz_topic Gazebo image topic name of the entry
/// \param[out] camera_info Camera info configuration, unset when the entry
/// does not bridge camera_info
/// \return false when the configuration is invalid, with an error logged
bool parseCameraInfo(
  const YAML::Node & node,
  const std::string & ros_topic,
  const std::string & gz_topic,
  std::optional<CameraInfoConfig> & camera_info)
{
  auto logger = rclcpp::get_logger("image_bridge_config");

  const YAML::Node ci_node = node[kCameraInfo];
  if (!ci_node) {
    // No camera_info bridge
    return true;
  }

  CameraInfoConfig config;
  // Defaults: camera_info topics derived from the image topics
  config.ros_topic_name = ros_topic + kCameraInfoSuffix;
  config.gz_topic_name = gz_topic + kCameraInfoSuffix;

  if (ci_node.IsNull() || (ci_node.IsMap() && ci_node.size() == 0)) {
    // Use the default topic names
    camera_info = config;
    return true;
  }

  if (!ci_node.IsMap()) {
    RCLCPP_ERROR(
      logger, "Could not parse entry: %s must be a YAML map", kCameraInfo);
    return false;
  }

  const auto topic_name = getValue(ci_node, kTopicName);
  const auto ros_topic_name = getValue(ci_node, kRosTopicName);
  const auto gz_topic_name = getValue(ci_node, kGzTopicName);

  if (!topic_name.empty() && !ros_topic_name.empty()) {
    RCLCPP_ERROR(
      logger, "Could not parse entry: %s and %s are mutually exclusive in %s",
      kTopicName, kRosTopicName, kCameraInfo);
    return false;
  }
  if (!topic_name.empty() && !gz_topic_name.empty()) {
    RCLCPP_ERROR(
      logger, "Could not parse entry: %s and %s are mutually exclusive in %s",
      kTopicName, kGzTopicName, kCameraInfo);
    return false;
  }

  if (!topic_name.empty()) {
    config.ros_topic_name = topic_name;
    config.gz_topic_name = topic_name;
  }
  if (!ros_topic_name.empty()) {
    config.ros_topic_name = ros_topic_name;
  }
  if (!gz_topic_name.empty()) {
    config.gz_topic_name = gz_topic_name;
  }

  camera_info = config;
  return true;
}

/// \brief Parse a single sequence entry into an ImageBridgeEntry
/// \param[in] node A node containing a map of bridge config params
/// \return ImageBridgeEntry on success, nullopt on failure
std::optional<ImageBridgeEntry> parseEntry(const YAML::Node & node)
{
  auto logger = rclcpp::get_logger("image_bridge_config");

  if (!node.IsMap()) {
    RCLCPP_ERROR(
      logger, "Could not parse entry: entry must be a YAML map");
    return {};
  }

  ImageBridgeEntry entry;

  if (!resolveTopicNames(node, entry.ros_topic_name, entry.gz_topic_name)) {
    return {};
  }

  // Message types are optional and default to image types; the bridge only
  // supports those, so anything else is a configuration error.
  const auto ros_type_name = getValue(node, kRosTypeName);
  const auto gz_type_name = getValue(node, kGzTypeName);
  if (!ros_type_name.empty() && ros_type_name != kRosImageType) {
    RCLCPP_ERROR(
      logger, "Could not parse entry: unsupported %s [%s], image_bridge only "
      "supports %s", kRosTypeName, ros_type_name.c_str(), kRosImageType);
    return {};
  }
  if (!gz_type_name.empty() && gz_type_name != kGzImageType) {
    RCLCPP_ERROR(
      logger, "Could not parse entry: unsupported %s [%s], image_bridge only "
      "supports %s", kGzTypeName, gz_type_name.c_str(), kGzImageType);
    return {};
  }

  if (node[kQosProfile]) {
    const auto qos_profile = getValue(node, kQosProfile);
    if (!qos_profile.empty()) {
      try {
        entry.qos_profile = ros_gz_bridge::parseQoS(qos_profile);
      } catch (const std::invalid_argument & e) {
        RCLCPP_ERROR(logger, "Could not parse entry: %s", e.what());
        return {};
      }
    }
  }
  if (node[kPublisherQueue]) {
    const auto queue_size_int = node[kPublisherQueue].as<int64_t>();
    if (queue_size_int >= 0) {
      entry.publisher_queue_size = static_cast<size_t>(queue_size_int);
    }
  }
  if (node[kLazy]) {
    entry.is_lazy = node[kLazy].as<bool>();
  }
  if (node[kFrameId]) {
    entry.frame_id = node[kFrameId].as<std::string>();
  }

  if (!parseCameraInfo(node, entry.ros_topic_name, entry.gz_topic_name, entry.camera_info)) {
    return {};
  }

  return entry;
}
}  // namespace

std::vector<ImageBridgeEntry> readFromYamlString(const std::string & data)
{
  std::vector<ImageBridgeEntry> entries;
  try {
    const auto yaml_node = YAML::Load(data);

    if (!yaml_node.IsSequence()) {
      RCLCPP_ERROR(
        rclcpp::get_logger("image_bridge_config"),
        "Could not parse config: top level must be a YAML sequence");
      return entries;
    }

    for (const auto & entry_node : yaml_node) {
      try {
        auto entry = parseEntry(entry_node);
        if (entry) {
          entries.push_back(entry.value());
        }
      } catch (const YAML::Exception & e) {
        RCLCPP_ERROR(
          rclcpp::get_logger("image_bridge_config"),
          "Could not parse entry: %s", e.what());
      }
    }
  } catch (const YAML::Exception & e) {
    RCLCPP_ERROR(
      rclcpp::get_logger("image_bridge_config"),
      "Could not parse config: %s", e.what());
  }
  return entries;
}

std::vector<ImageBridgeEntry> readFromYamlFile(const std::string & filename)
{
  std::vector<ImageBridgeEntry> entries;

  auto logger = rclcpp::get_logger("image_bridge_config");

  std::ifstream in(filename);
  if (!in.is_open()) {
    RCLCPP_ERROR(
      logger, "Could not parse config: failed to open file [%s]", filename.c_str());
    return entries;
  }

  // Compute file size to warn on empty configuration
  const auto fbegin = in.tellg();
  in.seekg(0, std::ios::end);
  const auto fend = in.tellg();
  const auto fsize = fend - fbegin;

  if (fsize == 0) {
    RCLCPP_ERROR(
      logger, "Could not parse config: file empty [%s]", filename.c_str());
    return entries;
  }

  in.seekg(0, std::ios::beg);

  std::stringstream ss;
  ss << in.rdbuf();
  return readFromYamlString(ss.str());
}
}  // namespace ros_gz_image
