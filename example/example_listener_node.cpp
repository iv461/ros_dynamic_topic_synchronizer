#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <fsd_topic_synchronizer/topic_synchronizer.hpp>
#include <memory>

void on_combined_messagess(const CombinedMessages &combined_messages) {}

int main(int argc, char **argv) {
  ros::init(argc, argv, "fsd_topic_synchronizer_test_listener");
  ros::NodeHandle nh;
  ros::NodeHandle private_node_handle("~");

  /// The type of the message synchronizer: Here we provide every distinct ROS-message type. Unlike
  /// to the message_filters-synchronizer, we do not need to specify the *number* of topics of a
  /// message type we want to subscribe to at compile time: This can be done at runtime.
  using SyncT = fsd::mf::TopicSynchronizer<sensor_msgs::Image, sensor_msgs::CameraInfo,
                                           sensor_msgs::PointCloud2>;

  /// Create the approximate time policy with a 1s max message age and .2s timeout.
  auto policy = std::make_unique<fsd::mf::ApproxTimePolicy>(ros::Duration(1.), ros::Duration(0.2));

  auto synchronizer = std::make_unique<SyncT>(
      private_node_handle_,

      [&](const auto &images, const auto &camera_infos, const auto &point_clouds) {
        /// HINT: The type of the arguments is a std::map from topic name to the messages for each
        /// message type, here we have four message types.
        size_t num_images_received = images.count("image1") + images.count("image1");
        size_t num_cam_infos_received =
            camera_infos.count("camera_info0") + camera_infos.count("camera_info1");
        size_t num_point_clouds_received = point_clouds.count("point_cloud");
        ROS_INFO_STREAM("Received " << num_images_received << " images, " << num_cam_infos_received
                                    << " camera infos and " << num_point_clouds_received
                                    << " point clouds.");

        /// Then, access the message for the desired topic if they were received
        if (num_point_clouds_received > 0) {
          sensor_msgs::PointCloud2::ConstPtr cloud_msg = point_clouds.at("point_cloud");
          ROS_INFO_STREAM("Received point cloud message with "
                          << cloud_msg->width * cloud_msg->height << " points.");
        }
      },
      policy);

  /// Specify all the topics to subscribe on, grouped by the message types.
  /// I.e. first all Image topics, then all CameraInfo topics, and then all PointCloud2 topics.
  /// This topic list can be provided at runtime, re-subscription is possible as well.
  synchronizer->subscribe(
      {{{"/image1", "/image2"}, {"/camera_info0", "/camera_info1"}, {"/point_cloud"}}}, 1);

  return 0;
}
