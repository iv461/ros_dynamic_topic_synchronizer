/// Example node that shows how to use the synchronizer from ROS 2.
/// It subscribes to two topics, yielding images and point clouds.

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <ros_dynamic_topic_synchronizer/topic_synchronizer_ros2.hpp>
#include <memory>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("ros_dynamic_topic_synchronizer_listener");

  /// The type of the message synchronizer: Here we provide every distinct ROS-message type. Unlike
  /// the message_filters-synchronizer, we do not need to specify the *number* of topics of a
  /// message type we want to subscribe to at compile time: This can be done at runtime.
  using SyncT = fsd::mf::ros2::TopicSynchronizer<sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo,
                                           sensor_msgs::msg::PointCloud2>;

  /// Create the approximate time policy with a 1s max message age and 0.2s timeout.
  auto policy = std::make_unique<fsd::mf::ApproxTimePolicy>(rclcpp::Duration::from_seconds(1.0),
                                                            rclcpp::Duration::from_seconds(0.2));

  auto synchronizer = std::make_unique<SyncT>(
      node,
      [&](const auto &images, const auto &camera_infos, const auto &point_clouds) {
        /// HINT: The type of the arguments is a std::map from topic name to the messages for each
        /// message type, here we have four message types.
        size_t num_images_received = images.count("/image1") + images.count("/image2");
        size_t num_cam_infos_received =
            camera_infos.count("/camera_info1") + camera_infos.count("/camera_info2");
        size_t num_point_clouds_received = point_clouds.count("/point_cloud");
        RCLCPP_INFO(node->get_logger(), "Received %zu images, %zu camera infos and %zu point clouds.",
                    num_images_received, num_cam_infos_received, num_point_clouds_received);

        /// Then, access the message for the desired topic if they were received
        if (num_point_clouds_received > 0) {
          auto cloud_msg = point_clouds.at("/point_cloud");
          RCLCPP_INFO(node->get_logger(), "Received point cloud message with %u points.",
                      cloud_msg->width * cloud_msg->height);
        }
      },
      std::move(policy));

  /// Specify all the topics to subscribe on, grouped by the message types.
  /// I.e. first all Image topics, then all CameraInfo topics, and then all PointCloud2 topics.
  /// This topic list can be provided at runtime, re-subscription is possible as well.
  /// The last argument ("1") is the queue size. It is the same for all subscribers.
  synchronizer->subscribe(
      {{{"/image1", "/image2"}, {"/camera_info1", "/camera_info2"}, {"/point_cloud"}}}, 1);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
