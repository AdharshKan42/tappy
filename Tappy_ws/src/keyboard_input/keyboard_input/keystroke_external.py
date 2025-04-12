import evdev
from evdev import InputDevice, categorize, ecodes
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class externalKeyboardPublisher(Node):
    def __init__(self):
        super().__init__("ext_key_publisher")
        self.publisher = self.create_publisher(String, "external_inp", 10)
        self.get_logger().info("external keyboard publisher initialized")
        self.device = InputDevice(
            "/dev/input/by-id/usb-SINO_WEALTH_Gaming_KB-event-kbd"
        )
        self.device.grab()

    def publish_keys(self):
        try:
            while rclpy.ok():
                for key in self.device.read():
                    if key.type == ecodes.EV_KEY:
                        key_event = categorize(key)
                        if key_event.keystate == key_event.key_down:
                            output_m = String()
                            output_m.data = key_event.key_code
                            self.publisher.publish(output_m)
                            self.get_logger().info("it should be published by now")

            rclpy.spin_once(self, timeout_sec=0.1)

        except KeyboardInterrupt:
            self.get_logger().info("Program exited")
        except OSError as e:
            self.get_logger().info(f"error reading keyboard, {e}")


def main(args=None):
    rclpy.init(args=args)
    publisher_node = externalKeyboardPublisher()

    publisher_node.get_logger().info("before publish key call")
    publisher_node.publish_keys()
    publisher_node.get_logger().info("after publish key call")

    rclpy.shutdown()
