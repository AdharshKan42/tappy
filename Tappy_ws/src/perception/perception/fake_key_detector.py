#! /usr/bin/env python3

import rclpy
import sys

from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from tf2_ros.transform_broadcaster import TransformBroadcaster
from turtlesim.msg import Pose
import numpy as np


class DynamicBroadcaster(Node):

    def __init__(self):
        super().__init__("dynamic_broadcaster")
        self.name_ = "key_T"
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
        tfs.transform.translation.x = 0.25
        tfs.transform.translation.y = 0.1
        tfs.transform.translation.z = 0.03

        # r = R.from_euler("xyz", [0, 0, msg.theta])
        
        quaternion = R.from_euler("xyz", [0, float(np.pi/2), 0]).as_quat()


        tfs.transform.rotation.x = quaternion[0]
        tfs.transform.rotation.y = quaternion[1]    
        tfs.transform.rotation.z = quaternion[2]
        tfs.transform.rotation.w = quaternion[3]

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
