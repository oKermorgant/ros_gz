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

// Tests for gazebosim/ros_gz#701: image_bridge must accept YAML
// configuration files with optional message types, and bridge the
// camera_info topic of a camera alongside its image topic, sharing the
// frame_id override.

#include <gtest/gtest.h>

#include <signal.h>
#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>

#include <gz/msgs/camera_info.pb.h>
#include <gz/msgs/image.pb.h>
#include <gz/transport/Node.hh>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

extern char ** environ;

namespace
{
constexpr auto kPublishPeriod = std::chrono::milliseconds(50);
constexpr auto kTimeout = std::chrono::seconds(20);
constexpr uint32_t kImageWidth = 320;
constexpr uint32_t kImageHeight = 240;
constexpr uint32_t kInfoWidth = 640;
constexpr uint32_t kInfoHeight = 480;

constexpr const char kGzFrameId[] = "config_test_gz_frame";
constexpr const char kOverrideFrameId[] = "config_test_cam_optical_frame";

/// \brief Set the frame ID of a Gazebo message header.
/// The Gazebo header carries the frame ID as a key/value pair in data.
void SetGzFrameId(gz::msgs::Header * header, const std::string & frame_id)
{
  auto frame_pair = header->add_data();
  frame_pair->set_key("frame_id");
  frame_pair->add_value(frame_id);
}

/// \brief A 320x240 RGB frame published on the test topics.
gz::msgs::Image TestImage()
{
  gz::msgs::Image img;
  SetGzFrameId(img.mutable_header(), kGzFrameId);
  img.set_width(kImageWidth);
  img.set_height(kImageHeight);
  img.set_step(kImageWidth * 3);
  img.set_pixel_format_type(gz::msgs::PixelFormatType::RGB_INT8);
  img.set_data(std::string(kImageWidth * kImageHeight * 3, '\x7f'));
  return img;
}

/// \brief A camera_info message published on the test topics.
gz::msgs::CameraInfo TestCameraInfo()
{
  gz::msgs::CameraInfo info;
  SetGzFrameId(info.mutable_header(), kGzFrameId);
  info.set_width(kInfoWidth);
  info.set_height(kInfoHeight);
  return info;
}
}  // namespace

TEST(ImageBridgeConfig, BridgesEntriesFromYamlConfigFile)
{
  // Isolate this test's Gazebo traffic. The bridge inherits the environment.
  const std::string partition = "image_bridge_config_" + std::to_string(getpid());
  ASSERT_EQ(0, setenv("GZ_PARTITION", partition.c_str(), 1));

  const auto img = TestImage();
  const auto info = TestCameraInfo();
  gz::transport::Node gz_node;
  auto pub_full = gz_node.Advertise<gz::msgs::Image>("/config_test_cam/image");
  auto pub_min = gz_node.Advertise<gz::msgs::Image>("/config_test_cam_min/image");
  auto pub_info_full =
    gz_node.Advertise<gz::msgs::CameraInfo>("/config_test_cam/camera_info");
  auto pub_info_min =
    gz_node.Advertise<gz::msgs::CameraInfo>("/config_test_cam_min/image/camera_info");
  ASSERT_TRUE(pub_full.Valid());
  ASSERT_TRUE(pub_min.Valid());
  ASSERT_TRUE(pub_info_full.Valid());
  ASSERT_TRUE(pub_info_min.Valid());

  rclcpp::init(0, nullptr);
  auto node = rclcpp::Node::make_shared("test_image_bridge_config");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::atomic<bool> got_renamed{false};
  std::atomic<bool> got_shorthand{false};
  std::atomic<bool> got_renamed_info{false};
  std::atomic<bool> got_shorthand_info{false};
  std::atomic<uint32_t> renamed_width{0};
  std::atomic<uint32_t> renamed_info_width{0};
  std::string renamed_frame_id;
  std::string shorthand_frame_id;
  std::string renamed_info_frame_id;
  std::string shorthand_info_frame_id;

  // image_transport publishes the raw image on the base topic.
  auto sub_renamed = node->create_subscription<sensor_msgs::msg::Image>(
    "/config_test_cam/image", rclcpp::QoS(10),
    [&got_renamed, &renamed_width, &renamed_frame_id](sensor_msgs::msg::Image::ConstSharedPtr msg) {
      renamed_width = msg->width;
      renamed_frame_id = msg->header.frame_id;
      got_renamed = true;
    });
  auto sub_shorthand = node->create_subscription<sensor_msgs::msg::Image>(
    "/config_test_cam_min/image", rclcpp::QoS(10),
    [&got_shorthand, &shorthand_frame_id](sensor_msgs::msg::Image::ConstSharedPtr msg) {
      shorthand_frame_id = msg->header.frame_id;
      got_shorthand = true;
    });
  auto sub_renamed_info = node->create_subscription<sensor_msgs::msg::CameraInfo>(
    "/config_test_cam/camera_info", rclcpp::QoS(10),
    [&got_renamed_info, &renamed_info_width, &renamed_info_frame_id](
      sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
      renamed_info_width = msg->width;
      renamed_info_frame_id = msg->header.frame_id;
      got_renamed_info = true;
    });
  // The shorthand entry derives the camera_info topics from the image topic.
  auto sub_shorthand_info = node->create_subscription<sensor_msgs::msg::CameraInfo>(
    "/config_test_cam_min/image/camera_info", rclcpp::QoS(10),
    [&got_shorthand_info, &shorthand_info_frame_id](
      sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
      shorthand_info_frame_id = msg->header.frame_id;
      got_shorthand_info = true;
    });

  char arg0[] = IMAGE_BRIDGE_EXECUTABLE;
  char arg1[] = "--ros-args";
  char arg2[] = "-p";
  const std::string config_arg = "config_file:=" IMAGE_BRIDGE_CONFIG_FILE;
  char * config_argv = strdup(config_arg.c_str());
  char * argv[] = {arg0, arg1, arg2, config_argv, nullptr};

  pid_t pid;
  ASSERT_EQ(0, posix_spawn(&pid, IMAGE_BRIDGE_EXECUTABLE, nullptr, nullptr, argv, environ));

  const auto deadline = std::chrono::steady_clock::now() + kTimeout;
  while (
    (!got_renamed || !got_shorthand || !got_renamed_info || !got_shorthand_info) &&
    std::chrono::steady_clock::now() < deadline)
  {
    pub_full.Publish(img);
    pub_min.Publish(img);
    pub_info_full.Publish(info);
    pub_info_min.Publish(info);
    executor.spin_some();
    std::this_thread::sleep_for(kPublishPeriod);
  }

  // Stop the bridge before asserting, so it is reaped even if the test fails.
  ASSERT_EQ(0, kill(pid, SIGINT));
  int status = 0;
  ASSERT_EQ(pid, waitpid(pid, &status, 0));

  EXPECT_TRUE(got_renamed) << "no image arrived on the renamed ROS topic";
  EXPECT_TRUE(got_shorthand) << "no image arrived on the topic_name shorthand topic";
  EXPECT_TRUE(got_renamed_info) << "no camera_info arrived on the renamed ROS topic";
  EXPECT_TRUE(got_shorthand_info) <<
    "no camera_info arrived on the camera_info topic derived from the image topic";
  EXPECT_EQ(kImageWidth, renamed_width);
  EXPECT_EQ(kInfoWidth, renamed_info_width);
  // The frame_id entry overrides the frame IDs of both the image and its
  // camera_info; bridges without one keep the received frame IDs.
  EXPECT_EQ(kOverrideFrameId, renamed_frame_id);
  EXPECT_EQ(kGzFrameId, shorthand_frame_id);
  EXPECT_EQ(kOverrideFrameId, renamed_info_frame_id);
  EXPECT_EQ(kGzFrameId, shorthand_info_frame_id);

  // The bridge must still exit cleanly when interrupted.
  ASSERT_FALSE(WIFSIGNALED(status)) <<
    "image_bridge was killed by signal " << WTERMSIG(status);
  ASSERT_TRUE(WIFEXITED(status));
  EXPECT_EQ(0, WEXITSTATUS(status));

  rclcpp::shutdown();
}
