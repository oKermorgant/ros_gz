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

Topics can also be configured with a YAML file using the same format as
[ros_gz_bridge](https://github.com/gazebosim/ros_gz/blob/ros2/ros_gz_bridge/README.md#configuration-yaml-file).
This allows renaming the ROS topic relative to the Gazebo topic and setting a
per-bridge QoS profile, lazy flag, or `frame_id`:

```yaml
- ros_topic_name: "navigation_camera/image"
  gz_topic_name: "/world/map/model/iris/link/navigation_camera_link/sensor/navigation_camera/image"
  ros_type_name: "sensor_msgs/msg/Image"
  gz_type_name: "gz.msgs.Image"
  direction: GZ_TO_ROS
  frame_id: "navigation_camera_link"
```

```shell
ros2 run ros_gz_image image_bridge --ros-args -p config_file:=<path/to/config.yaml>
```

Only image topics are supported: `ros_type_name` must be `sensor_msgs/msg/Image`
and `gz_type_name` must be `gz.msgs.Image`. Because the bridge is unidirectional,
`direction` must be `GZ_TO_ROS` (or omitted, in which case only the Gazebo to
ROS direction is bridged). Entries for other types, services, or `ROS_TO_GZ`
bridges are skipped with an error message.

A `frame_id` entry overrides the frame ID of the ROS image header; without it,
the frame ID received in the Gazebo message is kept.

Topics passed as arguments and entries from a config file can be combined.

## Parameters

### `config_file` (string, default: `""`)

Path to a YAML file with `ros_gz_bridge` style bridge entries, as shown above.

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
