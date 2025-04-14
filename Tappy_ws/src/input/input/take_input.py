#! /usr/bin/env python3

import rclpy
import sys

from rclpy.node import Node
from turtlesim.msg import Pose
from std_msgs.msg import String


class CollectInput(Node):

    def __init__(self):
        super().__init__("input_collector")

        self.sentence_publisher = self.create_publisher(String, "Sentence_to_type", 1)

        self.create_subscription(String, "type_status", self.get_next_input, 1)

        self.prompt_user()
        # self.name_ = "fake_key"
        # self.get_logger().info("Broadcasting pose of : {}".format(self.name_))
        # self.tfb_ = TransformBroadcaster(self)
        # self.timer = self.create_timer(1.0, self.on_timer)
        # self.sub_pose = self.create_subscription(
        #     Pose, "{}/pose".format(self.name_), self.handle_pose, 10
        # )

    def get_next_input(self, status):
        if status.data == "Done typing":
            self.prompt_user()

    def prompt_user(self):
        """Continuously prompts the user for input sentences."""
        while True:
            sentence = input("Enter sentence to be typed (or type 'exit' to quit): ")

            if sentence.lower() == "exit":
                self.get_logger().info("Exiting input collection.")
                break

            if sentence.isalnum() or sentence.replace(" ", "").isalnum():
                self.sentence_publisher.publish(String(data=sentence))
                self.get_logger().info(f"Published sentence: {sentence}")
            else:
                self.get_logger().warning("Invalid input. Please enter alphanumeric characters only.")


def main():
    rclpy.init()
    node = CollectInput()

    # try:
    rclpy.spin(node)
    # except KeyboardInterrupt:
    #     pass

    node.destroy_node()
    rclpy.shutdown()
