#! /usr/bin/env python3

import rclpy
import sys

from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from tf2_ros.transform_broadcaster import TransformBroadcaster
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from turtlesim.msg import Pose
from scipy.spatial.transform import Rotation
import numpy as np


class Camera_Stick_broadcaster(Node):

    def __init__(self):
        super().__init__("Camera_Stick_broadcaster")
        self.get_logger().info("Broadcasting pose of : camera and stick")
        # self.tfb_ = TransformBroadcaster(self)
        self.timer = self.create_timer(1.0, self.on_timer)
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        # self.sub_pose = self.create_subscription(
        #     Pose, "{}/pose".format(self.name_), self.handle_pose, 10
        # )

    def on_timer(self):
        self.get_logger().info("Broadcasting pose of : camera and stick")
        quaternion = Rotation.from_matrix(np.eye(3, 3)).as_quat()
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "rx200/fingers_link"
        t.child_frame_id = "stick_end"
        t.transform.translation.x = 0.09398
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = quaternion[0]
        t.transform.rotation.y = quaternion[1]
        t.transform.rotation.z = quaternion[2]
        t.transform.rotation.w = quaternion[3]

        self.tf_static_broadcaster.sendTransform(t)

        tfs = TransformStamped()
        tfs.header.stamp = self.get_clock().now().to_msg()
        tfs.header.frame_id = "stick_end"
        tfs.child_frame_id = "camera_link"
        tfs.transform.translation.x = -0.08128
        tfs.transform.translation.y = 0.01778
        tfs.transform.translation.z = 0.0381
        tfs.transform.rotation.x = quaternion[0]
        tfs.transform.rotation.y = quaternion[1]
        tfs.transform.rotation.z = quaternion[2]
        tfs.transform.rotation.w = quaternion[3]

        self.tf_static_broadcaster.sendTransform(tfs)


def main():
    rclpy.init()
    node = Camera_Stick_broadcaster()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()
