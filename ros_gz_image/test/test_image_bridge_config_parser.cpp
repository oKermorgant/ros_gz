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

// Unit tests for the image_bridge YAML configuration parser.

#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "image_bridge_config.hpp"

namespace
{
std::vector<ros_gz_image::ImageBridgeEntry> parse(const std::string & data)
{
  return ros_gz_image::readFromYamlString(data);
}
}  // namespace

TEST(ImageBridgeConfigParser, MinimalEntryUsesTopicNameForBothSides)
{
  const auto entries = parse("- topic_name: /cam/image\n");
  ASSERT_EQ(1u, entries.size());
  EXPECT_EQ("/cam/image", entries[0].ros_topic_name);
  EXPECT_EQ("/cam/image", entries[0].gz_topic_name);
  EXPECT_FALSE(entries[0].qos_profile.has_value());
  EXPECT_FALSE(entries[0].publisher_queue_size.has_value());
  EXPECT_FALSE(entries[0].is_lazy.has_value());
  EXPECT_TRUE(entries[0].frame_id.empty());
  EXPECT_FALSE(entries[0].camera_info.has_value());
}

TEST(ImageBridgeConfigParser, ExplicitTypesAndFields)
{
  const auto entries = parse(
    "- ros_topic_name: \"cam/image\"\n"
    "  gz_topic_name: \"/gz/cam/image\"\n"
    "  ros_type_name: \"sensor_msgs/msg/Image\"\n"
    "  gz_type_name: \"gz.msgs.Image\"\n"
    "  qos_profile: SENSOR_DATA\n"
    "  publisher_queue: 5\n"
    "  lazy: true\n"
    "  frame_id: \"cam_optical\"\n");
  ASSERT_EQ(1u, entries.size());
  EXPECT_EQ("cam/image", entries[0].ros_topic_name);
  EXPECT_EQ("/gz/cam/image", entries[0].gz_topic_name);
  ASSERT_TRUE(entries[0].qos_profile.has_value());
  ASSERT_TRUE(entries[0].publisher_queue_size.has_value());
  EXPECT_EQ(5u, entries[0].publisher_queue_size.value());
  ASSERT_TRUE(entries[0].is_lazy.has_value());
  EXPECT_TRUE(entries[0].is_lazy.value());
  EXPECT_EQ("cam_optical", entries[0].frame_id);
}

TEST(ImageBridgeConfigParser, CameraInfoDefaultsToDerivedTopics)
{
  const auto entries = parse(
    "- ros_topic_name: \"cam/image\"\n"
    "  gz_topic_name: \"/gz/cam/image\"\n"
    "  camera_info:\n");
  ASSERT_EQ(1u, entries.size());
  ASSERT_TRUE(entries[0].camera_info.has_value());
  EXPECT_EQ("cam/image/camera_info", entries[0].camera_info->ros_topic_name);
  EXPECT_EQ("/gz/cam/image/camera_info", entries[0].camera_info->gz_topic_name);
}

TEST(ImageBridgeConfigParser, CameraInfoExplicitTopics)
{
  const auto entries = parse(
    "- topic_name: /cam/image\n"
    "  camera_info:\n"
    "    gz_topic_name: /gz/cam/info\n"
    "    ros_topic_name: cam/info\n");
  ASSERT_EQ(1u, entries.size());
  ASSERT_TRUE(entries[0].camera_info.has_value());
  EXPECT_EQ("cam/info", entries[0].camera_info->ros_topic_name);
  EXPECT_EQ("/gz/cam/info", entries[0].camera_info->gz_topic_name);
}

TEST(ImageBridgeConfigParser, CameraInfoTopicNameSetsBothSides)
{
  const auto entries = parse(
    "- topic_name: /cam/image\n"
    "  camera_info:\n"
    "    topic_name: /cam/info\n");
  ASSERT_EQ(1u, entries.size());
  ASSERT_TRUE(entries[0].camera_info.has_value());
  EXPECT_EQ("/cam/info", entries[0].camera_info->ros_topic_name);
  EXPECT_EQ("/cam/info", entries[0].camera_info->gz_topic_name);
}

TEST(ImageBridgeConfigParser, NoCameraInfoByDefault)
{
  const auto entries = parse("- topic_name: /cam/image\n");
  ASSERT_EQ(1u, entries.size());
  EXPECT_FALSE(entries[0].camera_info.has_value());
}

TEST(ImageBridgeConfigParser, UnsupportedTypesAreSkipped)
{
  EXPECT_TRUE(parse(
      "- topic_name: /cam/image\n"
      "  ros_type_name: \"std_msgs/msg/String\"\n").empty());
  EXPECT_TRUE(parse(
      "- topic_name: /cam/image\n"
      "  gz_type_name: \"gz.msgs.StringMsg\"\n").empty());
}

TEST(ImageBridgeConfigParser, InvalidQosProfileIsSkipped)
{
  EXPECT_TRUE(parse(
      "- topic_name: /cam/image\n"
      "  qos_profile: NOT_A_PROFILE\n").empty());
}

TEST(ImageBridgeConfigParser, ConflictingTopicNamesAreSkipped)
{
  EXPECT_TRUE(parse(
      "- topic_name: /cam/image\n"
      "  ros_topic_name: cam/image\n").empty());
  EXPECT_TRUE(parse(
      "- topic_name: /cam/image\n"
      "  gz_topic_name: /cam/image\n").empty());
}

TEST(ImageBridgeConfigParser, MissingTopicNameIsSkipped)
{
  EXPECT_TRUE(parse("- frame_id: \"cam_optical\"\n").empty());
}

TEST(ImageBridgeConfigParser, InvalidCameraInfoIsSkipped)
{
  EXPECT_TRUE(parse(
      "- topic_name: /cam/image\n"
      "  camera_info: not_a_map\n").empty());
}

TEST(ImageBridgeConfigParser, TopLevelMustBeASequence)
{
  EXPECT_TRUE(parse("ros_topic_name: cam/image\n").empty());
}

TEST(ImageBridgeConfigParser, EntryMustBeAMap)
{
  EXPECT_TRUE(parse("- just_a_string\n").empty());
}

TEST(ImageBridgeConfigParser, InvalidYamlIsSkipped)
{
  EXPECT_TRUE(parse("- [unclosed\n").empty());
}
