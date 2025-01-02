#!/usr/bin/env python

"""
Copyright © 2024 StarkStrom Augsburg
Author: Ivo Ivanov
Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
"""

import rospy
import random
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from std_msgs.msg import Header

def create_image():
    header = Header()
    header.stamp = rospy.Time.now() + rospy.Duration.from_sec(random.uniform(-0.01, 0.01))
    header.frame_id = "camera_frame"
    return Image(header=header, height=480, width=640, encoding='rgb8', is_bigendian=0, step=1920, data=[0] * (480 * 640 * 3))


def create_camera_info():
    header = Header()
    header.stamp = rospy.Time.now() + rospy.Duration.from_sec(random.uniform(-0.01, 0.01))
    header.frame_id = "camera_frame"
    return CameraInfo(header=header, height=480, width=640, distortion_model='plumb_bob', D=[0.0] * 5, K=[0.0] * 9, R=[0.0] * 9, P=[0.0] * 12, binning_x=0, binning_y=0)


def create_pointcloud():
    header = Header()
    header.stamp = rospy.Time.now() + rospy.Duration.from_sec(random.uniform(-0.01, 0.01))
    header.frame_id = "pointcloud_frame"
    pointcloud = PointCloud2(header=header)
    return pointcloud


def publisher_node():
    rospy.init_node('example_talker_node', anonymous=True)

    image_pub_1 = rospy.Publisher('/image1', Image, queue_size=10)
    image_pub_2 = rospy.Publisher('/image2', Image, queue_size=10)
    cam_info_pub_1 = rospy.Publisher(
        '/camera_info1', CameraInfo, queue_size=10)
    cam_info_pub_2 = rospy.Publisher(
        '/camera_info2', CameraInfo, queue_size=10)
    pointcloud_pub = rospy.Publisher(
        '/point_cloud', PointCloud2, queue_size=10)

    rate = rospy.Rate(10)  # 10 Hz

    rospy.loginfo("First, publishing all messages for 5 seconds ..")
    for _ in range(50):
        image_msg_1 = create_image()
        image_msg_2 = create_image()
        cam_info_msg_1 = create_camera_info()
        cam_info_msg_2 = create_camera_info()
        pointcloud_msg = create_pointcloud()

        image_pub_1.publish(image_msg_1)
        image_pub_2.publish(image_msg_2)
        cam_info_pub_1.publish(cam_info_msg_1)
        cam_info_pub_2.publish(cam_info_msg_2)
        pointcloud_pub.publish(pointcloud_msg)

        #rospy.loginfo("Published messages at time: %s", rospy.Time.now())

        rate.sleep()

    rospy.loginfo("Now, simulating a sensor failure: Publishing only one image ..")

    for _ in range(50):
        image_msg_1 = create_image()
        cam_info_msg_1 = create_camera_info()
        cam_info_msg_2 = create_camera_info()
        pointcloud_msg = create_pointcloud()

        image_pub_1.publish(image_msg_1)
        cam_info_pub_1.publish(cam_info_msg_1)
        cam_info_pub_2.publish(cam_info_msg_2)
        pointcloud_pub.publish(pointcloud_msg)

        #rospy.loginfo("Published messages at time: %s", rospy.Time.now())

        rate.sleep()

if __name__ == '__main__':
    try:
        publisher_node()
    except rospy.ROSInterruptException:
        pass
