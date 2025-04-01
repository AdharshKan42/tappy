#! /usr/bin/env python3

import rclpy
import sys

from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from tf2_ros.transform_broadcaster import TransformBroadcaster
from turtlesim.msg import Pose


class DynamicBroadcaster(Node):

    def __init__(self):
        super().__init__("dynamic_broadcaster")
        self.name_ = "fake_key"
        self.get_logger().info("Broadcasting pose of : {}".format(self.name_))
        self.tfb_ = TransformBroadcaster(self)
        self.timer = self.create_timer(1.0, self.on_timer)
        # self.sub_pose = self.create_subscription(
        #     Pose, "{}/pose".format(self.name_), self.handle_pose, 10
        # )

    def on_timer(self):
        self.get_logger().info("Broadcasting pose of : {}".format(self.name_))
        tfs = TransformStamped()
        tfs.header.stamp = self.get_clock().now().to_msg()
        tfs.header.frame_id = "rx200/base_link"
        tfs._child_frame_id = self.name_
        tfs.transform.translation.x = 0.2
        tfs.transform.translation.y = 0.1
        tfs.transform.translation.z = 0.2

        # r = R.from_euler("xyz", [0, 0, msg.theta])

        tfs.transform.rotation.x = 1.0
        tfs.transform.rotation.y = 1.5
        tfs.transform.rotation.z = 0.0
        tfs.transform.rotation.w = 0.0

        self.tfb_.sendTransform(tfs)


def main():
    rclpy.init()
    node = DynamicBroadcaster()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()
