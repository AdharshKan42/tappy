#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from interbotix_common_modules.common_robot.robot import robot_shutdown, robot_startup
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS


class MyNode(Node):

    def __init__(self):
        super().__init__("first_node")
        self.get_logger().info("first node!")
        self.bot = InterbotixManipulatorXS(
            robot_model="rx200",
            group_name="arm",
            gripper_name="gripper",
        )
        self.get_logger().info("startup!")
        robot_startup()
        self.get_logger().info("moving!")
        self.move()
        self.get_logger().info("done moving!")
        robot_shutdown()
        self.get_logger().info("shutting down!")

    def move(self):
        self.bot.arm.go_to_home_pose()
        self.bot.arm.set_ee_pose_components(x=0.2, y=0.1, z=0.2, roll=1.0, pitch=1.5)
        self.bot.arm.go_to_home_pose()
        self.bot.arm.go_to_sleep_pose()


def main(args=None):
    rclpy.init(args=args)
    print("hi")

    # code here
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


# if __name__ == "__main__":
#     main()
