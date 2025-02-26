// Copyright 2022 Open Source Robotics Foundation, Inc.
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

#include <yaml-cpp/yaml.h>

#include <fstream>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <ros_gz_bridge/bridge_config.hpp>

namespace ros_gz_bridge
{

// YAML tag string constants
constexpr const char kTopicName[] = "topic_name";
constexpr const char kRosTopicName[] = "ros_topic_name";
constexpr const char kGzTopicName[] = "gz_topic_name";
constexpr const char kRosTypeName[] = "ros_type_name";
constexpr const char kGzTypeName[] = "gz_type_name";
constexpr const char kDirection[] = "direction";
constexpr const char kPublisherQueue[] = "publisher_queue";
constexpr const char kSubscriberQueue[] = "subscriber_queue";
constexpr const char kLazy[] = "lazy";

// Comparison strings for bridge directions
constexpr const char kBidirectional[] = "BIDIRECTIONAL";
constexpr const char kGzToRos[] = "GZ_TO_ROS";
constexpr const char kRosToGz[] = "ROS_TO_GZ";

/// \brief Parse a single sequence entry into a BridgeConfig
/// \param[in] yaml_node A node containing a map of bridge config params
/// \return BridgeConfig on success, nullopt on failure
std::optional<BridgeConfig> parseEntry(const YAML::Node & yaml_node)
{
  auto logger = rclcpp::get_logger("BridgeConfig");

  if (!yaml_node.IsMap()) {
    RCLCPP_ERROR(
      logger,
      "Could not parse entry: entry must be a YAML map");
    return {};
  }

  if (yaml_node[kTopicName] && yaml_node[kRosTopicName]) {
    RCLCPP_ERROR(
      logger,
      "Could not parse entry: %s and %s are mutually exclusive", kTopicName, kRosTopicName);
    return {};
  }

  if (yaml_node[kTopicName] && yaml_node[kGzTopicName]) {
    RCLCPP_ERROR(
      logger,
      "Could not parse entry: %s and %s are mutually exclusive", kTopicName, kGzTopicName);
    return {};
  }

  if (!yaml_node[kRosTypeName] || !yaml_node[kGzTypeName]) {
    RCLCPP_ERROR(
      logger,
      "Could not parse entry: both %s and %s must be set", kRosTypeName, kGzTypeName);
    return {};
  }

  BridgeConfig ret;

  ret.direction = BridgeDirection::BIDIRECTIONAL;

  if (yaml_node[kDirection]) {
    auto dirStr = yaml_node[kDirection].as<std::string>();

    if (dirStr == kBidirectional) {
      ret.direction = BridgeDirection::BIDIRECTIONAL;
    } else if (dirStr == kGzToRos) {
      ret.direction = BridgeDirection::GZ_TO_ROS;
    } else if (dirStr == kRosToGz) {
      ret.direction = BridgeDirection::ROS_TO_GZ;
    } else {
      RCLCPP_ERROR(
        logger,
        "Could not parse entry: invalid direction [%s]", dirStr.c_str());
      return {};
    }
  }


  if (yaml_node[kTopicName]) {
    // Only "topic_name" is set
    ret.gz_topic_name = yaml_node[kTopicName].as<std::string>();
    ret.ros_topic_name = yaml_node[kTopicName].as<std::string>();
  } else if (yaml_node[kRosTopicName] && !yaml_node[kGzTopicName]) {
    // Only "ros_topic_name" is set
    ret.gz_topic_name = yaml_node[kRosTopicName].as<std::string>();
    ret.ros_topic_name = yaml_node[kRosTopicName].as<std::string>();
  } else if (yaml_node[kGzTopicName] && !yaml_node[kRosTopicName]) {
    // Only kGzTopicName is set
    ret.gz_topic_name = yaml_node[kGzTopicName].as<std::string>();
    ret.ros_topic_name = yaml_node[kGzTopicName].as<std::string>();
  } else {
    // Both are set
    ret.gz_topic_name = yaml_node[kGzTopicName].as<std::string>();
    ret.ros_topic_name = yaml_node[kRosTopicName].as<std::string>();
  }

  ret.gz_type_name = yaml_node[kGzTypeName].as<std::string>();
  ret.ros_type_name = yaml_node[kRosTypeName].as<std::string>();

  if (yaml_node[kPublisherQueue]) {
    ret.publisher_queue_size = yaml_node[kPublisherQueue].as<size_t>();
  }
  if (yaml_node[kSubscriberQueue]) {
    ret.subscriber_queue_size = yaml_node[kSubscriberQueue].as<size_t>();
  }
  if (yaml_node[kLazy]) {
    ret.is_lazy = yaml_node[kLazy].as<bool>();
  }

  return ret;
}

std::vector<BridgeConfig> readFromYaml(std::istream & in)
{
  auto ret = std::vector<BridgeConfig>();

  YAML::Node yaml_node;
  yaml_node = YAML::Load(in);

  auto logger = rclcpp::get_logger("readFromYaml");
  if (!yaml_node.IsSequence()) {
    RCLCPP_ERROR(
      logger,
      "Could not parse config, top level must be a YAML sequence");
    return ret;
  }

  for (auto it : yaml_node) {
    auto entry = parseEntry(it);
    if (entry) {
      ret.push_back(entry.value());
    }
  }

  return ret;
}

std::vector<BridgeConfig> readFromYamlFile(const std::string & filename)
{
  std::vector<BridgeConfig> ret;
  std::ifstream in(filename);
  return readFromYaml(in);
}

std::vector<BridgeConfig> readFromYamlString(const std::string & data)
{
  std::stringstream ss(data);
  return readFromYaml(ss);
}
}  // namespace ros_gz_bridge
