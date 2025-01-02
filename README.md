# ros_dynamic_topic_synchronizer

This is a Robot Operating System (ROS) package for synchronizing multiple topics by approximately matching their message timestamps, similar to the [message_filters](https://wiki.ros.org/message_filters)-package.
Unlike the `message_filters` package however, the number of messages does not need to be set at compile time, but can be provided at runtimme instead.

This package therefore removes the limitation of having to know the number of topics at compile time, instead only the message types need to be provided.

This also enables the feature that is very useful for implementing redundant sensor fusion applications: A timeout that allows to receive the combined data from the remaining sensors in case some sensors fail.

# Requirements

- ROS 1 Noetic or ROS 2 Humble or later (tested with Humble and Jazzy)
- C++ 17 

# Example

An example for synchronizing multiple cameras and LiDAR-sensors (ROS 2):
```cpp
  /// First, provide every distinct ROS-message type. Unlike
  /// the message_filters-synchronizer, we do not need to specify the *number* of topics of a
  /// message type we want to subscribe to at compile time: This can be done at runtime.
  using SyncT = fsd::mf::ros2::TopicSynchronizer<sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo, sensor_msgs::msg::PointCloud2>;

  /// Create the approximate time policy with a 1s max message age and 0.2s timeout.
  auto policy = std::make_unique<fsd::mf::ApproxTimePolicy>(rclcpp::Duration::from_seconds(1.0), rclcpp::Duration::from_seconds(0.2));

  auto synchronizer = std::make_unique<SyncT>(
      node,
      [&](const auto &images, const auto &camera_infos, const auto &point_clouds) {
        /// Aaccess the message for the desired topic if they were received
        if (point_clouds.count("/point_cloud") > 0) {
          auto cloud_msg = point_clouds.at("/point_cloud");
          RCLCPP_INFO(node->get_logger(), "Received point cloud message with %u points.",
                      cloud_msg->width * cloud_msg->height);
        }
      },
      std::move(policy));

  /// Subscribe to all topics, grouped by the message types, i.e. first all Image topics, then all CameraInfo topics, and then all PointCloud2 topics.
  /// This topic list can be provided at runtime, re-subscription is possible as well.
  synchronizer->subscribe(
      {{{"/image1", "/image2"}, {"/camera_info1", "/camera_info2"}, {"/point_cloud"}}}, /*queue_size=*/1);
```

See also the [examples/](examples) folder for a talker-listener example that synchronizes three different topics.
It demonstrates the approximate time-stamp matching as well as the time-out behavior. After a topic "times out" (for example due to sensor malfunction), we still receive the synchronized messages of the remaining sensors.

# Building 

After building the package, just start the listener by 

```sh
rosrun fsd_topic_synchronizer fsd_topic_synchronizer_example_listener
```

and then the talker: 

```sh
rosrun fsd_topic_synchronizer fsd_topic_synchronizer_example_talker
```

which should give the expected output: 

```sh
TODO
```

# Run tests 

To build and execute the unit-tests for the synchronization policy, just run: 

```sh
catkin test fsd_topic_synchronizer
```

# License 

This code is licensed under Apache 2 license.




