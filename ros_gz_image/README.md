# Image utilities for using ROS and Gazebo Transport

This package provides a unidirectional bridge for images from Gazebo to ROS.
The bridge subscribes to Gazebo image messages (`gz::msgs::Image`)
and republishes them to ROS using [image_transport](http://wiki.ros.org/image_transport).

For compressed images, install
[compressed_image_transport](http://wiki.ros.org/compressed_image_transport)
and the bridge will publish `/compressed` images. The same goes for other
`image_transport` plugins.

To run the bridge from the command line:

```shell
ros2 run ros_gz_image image_bridge /topic1 /topic2
```

Topics can also be configured with a YAML file. Entries use the same field
names as [ros_gz_bridge](https://github.com/gazebosim/ros_gz/blob/ros2/ros_gz_bridge/README.md#configuration-yaml-file)
configuration files, but the message types are optional: every entry bridges
`gz.msgs.Image` messages to `sensor_msgs/msg/Image`, so only topic names are
required. Entries may rename the ROS topic relative to the Gazebo topic,
override the frame ID, and bridge the camera `camera_info` topic alongside the
image topic:

```yaml
- ros_topic_name: "navigation_camera/image"
  gz_topic_name: "/world/map/model/iris/link/navigation_camera_link/sensor/navigation_camera/image"
  frame_id: "navigation_camera_link"
  camera_info:
    # Optional; both default to the image topic names + "/camera_info"
    gz_topic_name: "/world/map/model/iris/link/navigation_camera_link/sensor/navigation_camera/camera_info"
    ros_topic_name: "navigation_camera/camera_info"

# topic_name applies to both sides
- topic_name: "/camera/front/image_raw"
```

```shell
ros2 run ros_gz_image image_bridge --ros-args -p config_file:=<path/to/config.yaml>
```

Because the bridge is unidirectional from Gazebo to ROS, no direction needs to
be set. Entries that set a message type other than `sensor_msgs/msg/Image`
and `gz.msgs.Image` are skipped with an error message.

A `frame_id` entry overrides the frame ID of the ROS headers of the image and
its `camera_info`; without it, the frame ID received in the Gazebo messages is
kept.

Topics passed as arguments and entries from a config file can be combined.

## Parameters

### `config_file` (string, default: `""`)

Path to a YAML file with image bridge entries, as shown above.

### `qos` (string, default: `"default"`)

Modify the [Quality of Service (QoS) policy](https://docs.ros.org/en/rolling/Concepts/About-Quality-of-Service-Settings.html#qos-policies) used to publish images. Accepted values: `default`, `sensor_data`, `system_default`.

```shell
ros2 run ros_gz_image image_bridge /topic1 /topic2 --ros-args -p qos:=sensor_data
```

### `lazy` (bool, default: `false`)

When set to `true`, the bridge will only subscribe to the Gazebo topic when there is at least one ROS subscriber. This avoids unnecessary message processing when no one is listening.

```shell
ros2 run ros_gz_image image_bridge /topic1 /topic2 --ros-args -p lazy:=true
```

### `subscription_heartbeat` (int, default: `1000`)

Interval in milliseconds at which the bridge checks the ROS subscriber count to start or stop the Gazebo subscription. Only relevant when `lazy:=true`.

```shell
ros2 run ros_gz_image image_bridge /topic1 /topic2 --ros-args -p lazy:=true -p subscription_heartbeat:=500
```
