import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class TfListener(Node):

    def __init__(self):
        super.__init__("tf_listener_node")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(1.0, self.on_timer)

    def on_timer(self):
        try:
            # the name of the keyboard key frame has to be whatever you define it as
            transform = self.tf_buffer.lookup_transform(
                "keyboard_key", "end_effector", rclpy.time.Time()
            )
            self.get_logger().info(
                transform, "this is the transform from base to sensor frame"
            )
        except TransformException as e:
            self.get_logger().info("Error", e)


def main():
    rclpy.init()
    node = TfListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()
