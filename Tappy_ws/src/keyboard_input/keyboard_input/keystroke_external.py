#! /usr/bin/env python3

import rclpy
import sys

from evdev import InputDevice, categorize, ecodes
from rclpy.node import Node
from std_msgs.msg import String


class ExternalKeyboardInput(Node):

    def __init__(self):
        super().__init__("External_keyboard_Input")
        # self.name_ = "key_a"
        self.get_logger().info("Node started")
        # self.tfb_ = TransformBroadcaster(self)
        self.publisher = self.create_publisher(String, "keyboard_inp", 10)
        self.timer = self.create_timer(0.3, self.read_key)
        # self.sub_pose = self.create_subscription(
        #     Pose, "{}/pose".format(self.name_), self.handle_pose, 10
        # )
        self.device = InputDevice(
            "/dev/input/by-id/usb-SINO_WEALTH_Gaming_KB-event-kbd"
        )
        self.device.grab()

    def read_key(self):
        try:
            for key in self.device.read():
                if key.type == ecodes.EV_KEY:
                    key_event = categorize(key)
                    if key_event.keystate == key_event.key_down:
                        output_m = String()
                        key = key_event.keycode.split("_")
                        key = key[-1].lower()
                        output_m.data = key
                        self.publisher.publish(output_m)
                        self.get_logger().info(str(key_event.keycode))
        except OSError:
            self.get_logger().info(f"Nothing pushed")


def main():
    rclpy.init()
    node = ExternalKeyboardInput()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()
