import rclpy

from rclpy.node import Node
from std_msgs.msg import String
from inputs import get_key


class KeyboardPublisher(Node):
    def __init__(self):
        super().__init__("keyboard_publisher")
        self.publisher = self.create_publisher(String, "keyboard_inp", 10)
        self.get_logger().info("Publisher is about to start publishing")

    def publish_keys(self):
        try:
            while rclpy.ok():
                keys_pressed = get_key()
                for key in keys_pressed:
                    if key.ev_type == "Key" and key.state == 1:
                        key_pushed = key.code.lower().split("_")
                        key_name = key_pushed[1]
                        output_message = String()
                        output_message.data = f"{key_name}"
                        self.publisher.publish(output_message)
                        self.get_logger().info("it should be published by now")

                rclpy.spin_once(self, timeout_sec=0.1)

        except KeyboardInterrupt:
            self.get_logger().info("Program exited")


def main(args=None):
    rclpy.init(args=args)
    publisher_node = KeyboardPublisher()

    publisher_node.get_logger().info("before publish key call")
    publisher_node.publish_keys()
    publisher_node.get_logger().info("after publish key call")

    rclpy.shutdown()
