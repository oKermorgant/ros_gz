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
//
#ifndef IMAGE_BRIDGE_CONFIG_HPP_
#define IMAGE_BRIDGE_CONFIG_HPP_

#include <optional>
#include <string>
#include <vector>

#include <rclcpp/qos.hpp>

#include <ros_gz_bridge/bridge_config.hpp>

namespace ros_gz_image
{

/// \brief The camera_info bridge associated with an image entry.
struct CameraInfoConfig
{
  /// \brief ROS camera_info topic name
  std::string ros_topic_name;

  /// \brief Gazebo camera_info topic name
  std::string gz_topic_name;
};

/// \brief A single image bridge entry of a YAML config file.
///
/// The bridge is unidirectional (Gazebo to ROS) and the message types are
/// fixed (sensor_msgs/msg/Image and gz.msgs.Image), so most entries only
/// need topic names, and optionally QoS, lazy, frame_id, and a camera_info
/// sub-configuration.
struct ImageBridgeEntry
{
  /// \brief The ROS image topic name
  std::string ros_topic_name;

  /// \brief The Gazebo image topic name
  std::string gz_topic_name;

  /// \brief QoS profile of the ROS publisher (uppercase, e.g. "SENSOR_DATA").
  /// When std::nullopt, the node-level qos parameter is used.
  std::optional<rclcpp::QoS> qos_profile;

  /// \brief Depth of the publisher queue. When std::nullopt, the QoS profile
  /// or the node-level qos parameter applies.
  std::optional<size_t> publisher_queue_size;

  /// \brief Flag to change the "laziness" of the bridge.
  /// When std::nullopt, the node-level lazy parameter is used as the default.
  std::optional<bool> is_lazy;

  /// \brief The frame ID to inject into the ROS headers of both the image
  /// and camera_info messages. Empty keeps the received frame ID.
  std::string frame_id;

  /// \brief The camera_info bridge of this camera.
  /// When std::nullopt, no camera_info topic is bridged.
  std::optional<CameraInfoConfig> camera_info;
};

/// \brief Generate image bridge entries from a YAML string
/// \param[in] data string containing YAML of bridge configurations
/// \return Vector of bridge entries. Invalid entries are skipped with an error.
std::vector<ImageBridgeEntry> readFromYamlString(const std::string & data);

/// \brief Generate image bridge entries from a YAML file
/// \param[in] filename name of file containing YAML of bridge configurations
/// \return Vector of bridge entries. Invalid entries are skipped with an error.
std::vector<ImageBridgeEntry> readFromYamlFile(const std::string & filename);

}  // namespace ros_gz_image

#endif  // IMAGE_BRIDGE_CONFIG_HPP_
