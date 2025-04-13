import rclpy

from rclpy.node import Node
from std_msgs.msg import String


class KeyboardListener(Node):
    def __init__(self):
        super().__init__("keyboard_listener")
        self.subscription = self.create_subscription(
            String, "external_inp", self.listener_callback, 10
        )
        self.get_logger().info("Listener started listening!")
        self.subscription

    def listener_callback(self, msg):
        self.get_logger().info(
            f"Im a subscriber and the letter pressed was published {msg.data}"
        )


def main(args=None):
    rclpy.init(args=args)
    subscriber_node = KeyboardListener()
    rclpy.spin(subscriber_node)
    rclpy.shutdown()
