// Copyright 2019 Open Source Robotics Foundation, Inc.
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

#include <rmw/qos_profiles.h>

#include <atomic>
#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <gz/transport/Node.hh>
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <ros_gz_bridge/convert.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "image_bridge_config.hpp"

//////////////////////////////////////////////////
/// \brief A Gazebo to ROS bridge managed by image_bridge.
class Bridge
{
public:
  virtual ~Bridge() = default;

  /// \brief Manage the Gazebo subscription lifecycle based on the ROS
  /// subscriber count. Only active when lazy mode is enabled.
  virtual void CheckSubscribers() = 0;
};

//////////////////////////////////////////////////
/// \brief Base class bridging one Gazebo topic to one ROS topic.
/// \param[in] GzMsgT Gazebo message type
template<typename GzMsgT>
class HandlerBase : public Bridge
{
public:
  /// \brief Constructor
  /// \param[in] _gz_topic Gazebo topic
  /// \param[in] _gz_node Pointer to Gazebo node
  /// \param[in] _lazy Whether to lazily subscribe to Gazebo only when ROS subscribers are present
  /// \param[in] _frame_id Frame ID to set on the ROS header, overriding the one
  /// received from Gazebo. Empty keeps the received value.
  HandlerBase(
    const std::string & _gz_topic,
    std::shared_ptr<gz::transport::Node> _gz_node,
    bool _lazy,
    const std::string & _frame_id)
  : gz_topic_(_gz_topic),
    frame_id_(_frame_id),
    gz_node_(_gz_node),
    is_lazy_(_lazy)
  {
  }

  /// \brief Destructor. Blocks until any in-flight OnMessage call finishes,
  /// then prevents later callbacks from touching this handler.
  ~HandlerBase() override
  {
    std::lock_guard<std::mutex> lock(guard_->mutex);
    guard_->closed = true;
  }

  /// \brief Manage Gazebo subscription lifecycle based on ROS subscriber count.
  /// Only active when lazy mode is enabled.
  void CheckSubscribers() override
  {
    if (!is_lazy_) {
      return;
    }

    const size_t num_subs = GetNumRosSubscribers();

    if (has_subscriber_ && num_subs == 0) {
      StopSubscriber();
    } else if (!has_subscriber_ && num_subs > 0) {
      StartSubscriber();
    }
  }

protected:
  /// \brief Subscribe to the Gazebo topic. Called by the constructor unless
  /// lazy, and by CheckSubscribers in lazy mode.
  void StartSubscriber()
  {
    // Gazebo transport does not wait for in-flight callbacks on Unsubscribe or
    // ~Node, so a callback can still run after this handler is destroyed. The
    // callback co-owns the guard, which therefore outlives the handler.
    auto guard = guard_;
    std::function<void(const GzMsgT &)> cb =
      [this, guard](const GzMsgT & _gz_msg)
      {
        std::lock_guard<std::mutex> lock(guard->mutex);
        if (guard->closed) {
          return;
        }
        // The handler is alive here: ~HandlerBase blocks on guard->mutex.
        this->OnMessage(_gz_msg);
      };
    gz_node_->Subscribe(gz_topic_, cb);
    has_subscriber_ = true;
  }

  /// \brief Gazebo topic name
  std::string gz_topic_;

  /// \brief Frame ID overriding the one received from Gazebo. Empty keeps it.
  std::string frame_id_;

  /// \brief Gazebo transport node
  std::shared_ptr<gz::transport::Node> gz_node_;

  /// \brief Whether to use lazy subscription
  bool is_lazy_{false};

  /// \brief Whether currently subscribed to Gazebo topic
  std::atomic<bool> has_subscriber_{false};

  /// \brief State shared between the handler and its Gazebo callback.
  struct CallbackGuard
  {
    /// \brief Held while a callback runs and in ~HandlerBase.
    std::mutex mutex;

    /// \brief Set in ~HandlerBase; callbacks no-op once teardown has started.
    bool closed{false};
  };

  /// \brief Guard co-owned by the Gazebo callback so it outlives the handler.
  std::shared_ptr<CallbackGuard> guard_{std::make_shared<CallbackGuard>()};

private:
  void StopSubscriber()
  {
    // A callback still in flight after Unsubscribe is harmless: the handler
    // stays alive, and its destructor waits for the callback to finish.
    gz_node_->Unsubscribe(gz_topic_);
    has_subscriber_ = false;
  }

  /// \brief Callback when a Gazebo message is received
  /// \param[in] _gz_msg Gazebo message
  virtual void OnMessage(const GzMsgT & _gz_msg) = 0;

  /// \brief Number of ROS subscribers of this bridge
  virtual size_t GetNumRosSubscribers() = 0;
};

using ImageHandlerBase = HandlerBase<gz::msgs::Image>;

//////////////////////////////////////////////////
/// \brief Bridges one Gazebo image topic to ROS using image_transport
class ImageHandler : public ImageHandlerBase
{
public:
  /// \brief Constructor
  /// \param[in] _ros_topic ROS image base topic
  /// \param[in] _gz_topic Gazebo image topic
  /// \param[in] _qos QoS profile for the ROS publisher
  /// \param[in] _node Pointer to ROS node
  /// \param[in] _gz_node Pointer to Gazebo node
  /// \param[in] _lazy Whether to lazily subscribe to Gazebo only when ROS subscribers are present
  /// \param[in] _frame_id Frame ID to set on the ROS image header, overriding the one
  /// received from Gazebo. Empty keeps the received value.
  ImageHandler(
    const std::string & _ros_topic,
    const std::string & _gz_topic,
    const rclcpp::QoS & _qos,
    std::shared_ptr<rclcpp::Node> _node,
    std::shared_ptr<gz::transport::Node> _gz_node,
    bool _lazy = false,
    const std::string & _frame_id = "")
  : HandlerBase(_gz_topic, _gz_node, _lazy, _frame_id)
  {
    // Always create the ROS publisher
    this->ros_pub_ = image_transport::create_publisher(
      *_node, _ros_topic, _qos);

    // Subscribe to Gazebo immediately unless lazy
    if (!is_lazy_) {
      StartSubscriber();
    }
  }

private:
  void OnMessage(const gz::msgs::Image & _gz_msg) override
  {
    sensor_msgs::msg::Image ros_msg;
    ros_gz_bridge::convert_gz_to_ros(_gz_msg, ros_msg);
    if (!frame_id_.empty()) {
      ros_msg.header.frame_id = frame_id_;
    }
    this->ros_pub_.publish(ros_msg);
  }

  size_t GetNumRosSubscribers() override
  {
    return ros_pub_.getNumSubscribers();
  }

  /// \brief ROS image publisher
  image_transport::Publisher ros_pub_;
};

using CameraInfoHandlerBase = HandlerBase<gz::msgs::CameraInfo>;

//////////////////////////////////////////////////
/// \brief Bridges one Gazebo camera_info topic to ROS
class CameraInfoHandler : public CameraInfoHandlerBase
{
public:
  /// \brief Constructor
  /// \param[in] _ros_topic ROS camera_info topic
  /// \param[in] _gz_topic Gazebo camera_info topic
  /// \param[in] _qos QoS profile for the ROS publisher
  /// \param[in] _node Pointer to ROS node
  /// \param[in] _gz_node Pointer to Gazebo node
  /// \param[in] _lazy Whether to lazily subscribe to Gazebo only when ROS subscribers are present
  /// \param[in] _frame_id Frame ID to set on the ROS camera_info header, overriding
  /// the one received from Gazebo. Empty keeps the received value.
  CameraInfoHandler(
    const std::string & _ros_topic,
    const std::string & _gz_topic,
    const rclcpp::QoS & _qos,
    std::shared_ptr<rclcpp::Node> _node,
    std::shared_ptr<gz::transport::Node> _gz_node,
    bool _lazy = false,
    const std::string & _frame_id = "")
  : HandlerBase(_gz_topic, _gz_node, _lazy, _frame_id)
  {
    // Always create the ROS publisher
    this->ros_pub_ = _node->create_publisher<sensor_msgs::msg::CameraInfo>(_ros_topic, _qos);

    // Subscribe to Gazebo immediately unless lazy
    if (!is_lazy_) {
      StartSubscriber();
    }
  }

private:
  void OnMessage(const gz::msgs::CameraInfo & _gz_msg) override
  {
    sensor_msgs::msg::CameraInfo ros_msg;
    ros_gz_bridge::convert_gz_to_ros(_gz_msg, ros_msg);
    if (!frame_id_.empty()) {
      ros_msg.header.frame_id = frame_id_;
    }
    this->ros_pub_->publish(ros_msg);
  }

  size_t GetNumRosSubscribers() override
  {
    return ros_pub_->get_subscription_count();
  }

  /// \brief ROS camera_info publisher
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>> ros_pub_;
};

//////////////////////////////////////////////////
void usage()
{
  std::cerr << "Bridge a collection of Gazebo Transport image topics to ROS " <<
    "using image_transport.\n\n" <<
    "  image_bridge <topic> <topic> ..\n\n" <<
    "Topics may also be configured with a YAML file, via the config_file\n" <<
    "parameter. Entries default to sensor_msgs/msg/Image and gz.msgs.Image,\n" <<
    "so only topic names are required:\n" <<
    "  - ros_topic_name: \"camera/image\"\n" <<
    "    gz_topic_name: \"/world/.../camera/image\"\n" <<
    "    frame_id: \"camera_link\"\n" <<
    "    camera_info:            # optional, topics default to <topic>/camera_info\n\n" <<
    "Optional ROS parameters:\n" <<
    "  config_file:=<path>        YAML file with image bridge entries\n" <<
    "  frame_id (config file)     Per-bridge frame ID for the ROS headers\n" <<
    "  qos:=<profile>              QoS profile: default, sensor_data, system_default\n" <<
    "  lazy:=<true|false>          Only subscribe to Gazebo when ROS subscribers are present\n" <<
    "  subscription_heartbeat:=<ms> Interval (ms) to check ROS subscriber count (default: 1000)\n\n"
            <<
    "E.g.: image_bridge /camera/front/image_raw\n" <<
    "E.g.: image_bridge /camera/front/image_raw --ros-args -p lazy:=true\n" <<
    "E.g.: image_bridge --ros-args -p config_file:=bridges.yaml" << std::endl;
}

//////////////////////////////////////////////////
rclcpp::QoS QoSFromParameter(std::shared_ptr<rclcpp::Node> _node)
{
  const auto qos_str = _node->get_parameter("qos").get_parameter_value().get<std::string>();
  if (qos_str == "system_default") {
    return rclcpp::SystemDefaultsQoS();
  } else if (qos_str == "sensor_data") {
    return rclcpp::SensorDataQoS();
  } else if (qos_str != "default") {
    RCLCPP_ERROR(
      _node->get_logger(),
      "Invalid QoS profile %s specified. Using default profile.",
      qos_str.c_str());
  }
  return rclcpp::QoS(10);
}

//////////////////////////////////////////////////
/// \brief Resolve the QoS of a config file entry
/// \param[in] _entry Config file entry
/// \param[in] _default_qos QoS of the node-level qos parameter
rclcpp::QoS EntryQoS(
  const ros_gz_image::ImageBridgeEntry & _entry,
  const rclcpp::QoS & _default_qos)
{
  if (_entry.qos_profile.has_value()) {
    auto qos = *_entry.qos_profile;
    if (_entry.publisher_queue_size.has_value() && *_entry.publisher_queue_size > 0) {
      qos.keep_last(*_entry.publisher_queue_size);
    }
    return qos;
  }
  if (_entry.publisher_queue_size.has_value()) {
    return rclcpp::QoS(rclcpp::KeepLast(*_entry.publisher_queue_size));
  }
  return _default_qos;
}

//////////////////////////////////////////////////
int main(int argc, char * argv[])
{
  if (argc < 2) {
    usage();
    return -1;
  }

  rclcpp::init(argc, argv);

  // ROS node
  auto node_ = rclcpp::Node::make_shared("ros_gz_image");
  node_->declare_parameter("config_file", "");
  node_->declare_parameter("qos", "default");
  node_->declare_parameter("lazy", false);
  node_->declare_parameter("subscription_heartbeat", 1000);

  const bool lazy = node_->get_parameter("lazy").as_bool();
  const int heartbeat_ms = node_->get_parameter("subscription_heartbeat").as_int();

  if (lazy && heartbeat_ms <= 0) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "subscription_heartbeat must be a positive integer (got %d). Using default of 1000 ms.",
      heartbeat_ms);
    rclcpp::shutdown();
    return -1;
  }

  const rclcpp::QoS default_qos = QoSFromParameter(node_);

  // Gazebo node
  auto gz_node = std::make_shared<gz::transport::Node>();

  std::vector<std::shared_ptr<Bridge>> bridges;

  // skip the process name in argument processing
  ++argv;
  --argc;
  auto args = rclcpp::remove_ros_arguments(argc, argv);

  // Create bridges from the config file, if given
  const std::string config_file = node_->get_parameter("config_file").as_string();
  if (!config_file.empty()) {
    for (const auto & entry : ros_gz_image::readFromYamlFile(config_file)) {
      const rclcpp::QoS qos = EntryQoS(entry, default_qos);
      const bool entry_lazy = entry.is_lazy.value_or(lazy);

      bridges.push_back(
        std::make_shared<ImageHandler>(
          entry.ros_topic_name, entry.gz_topic_name, qos, node_, gz_node,
          entry_lazy, entry.frame_id));

      if (entry.camera_info.has_value()) {
        bridges.push_back(
          std::make_shared<CameraInfoHandler>(
            entry.camera_info->ros_topic_name, entry.camera_info->gz_topic_name, qos,
            node_, gz_node, entry_lazy, entry.frame_id));
      }
    }
  }

  // Create publishers (and subscribers if not lazy) for topics given as arguments
  for (const auto & topic : args) {
    bridges.push_back(
      std::make_shared<ImageHandler>(topic, topic, default_qos, node_, gz_node, lazy));
  }

  if (bridges.empty()) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "No image topics to bridge: pass topics as arguments or set the config_file parameter.");
    rclcpp::shutdown();
    return -1;
  }

  // When lazy, periodically check ROS subscriber count to start/stop Gz subscriptions
  rclcpp::TimerBase::SharedPtr heartbeat_timer;
  if (lazy) {
    heartbeat_timer = node_->create_wall_timer(
      std::chrono::milliseconds(heartbeat_ms),
      [&bridges]() {
        for (auto & bridge : bridges) {
          bridge->CheckSubscribers();
        }
      });
  }

  // Spin ROS and Gz until shutdown
  rclcpp::spin(node_);

  // Tear down explicitly: destroy the bridges, which stop their callbacks,
  // before the Gazebo node that delivers them.
  heartbeat_timer.reset();
  bridges.clear();
  gz_node.reset();

  return 0;
}
