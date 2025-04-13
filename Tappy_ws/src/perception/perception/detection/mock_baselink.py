import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from tf2_ros.buffer import Buffer
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros.transform_listener import TransformListener
from typing import Optional

class Mock_Baselink(Node):
    """
    A mock node that publishes a static transform between the baselink and camera_link frames.
    This is used for testing purposes to simulate the presence of a camera on the robot.
    """
    def __init__(self) -> None:
        super().__init__("mock_baselink")

        # TF magic
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Create a timer to periodically publish the baselink transform
        self.timer = self.create_timer(0.1, self.publish_baselink)

    def publish_baselink(self) -> None:
        # Create a static transform between the baselink and camera_link
        # This transform is static and does not change over time
        # Create a TransformStamped message
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "rx200/base_link"
        transform.child_frame_id = "camera_link"

        # Set the translation (x, y, z)
        transform.transform.translation.x = 0.0
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.0

        # Set the rotation (quaternion)
        rotation = Rotation.from_euler("xyz", [0.0, 0.0, 0.0])
        q = rotation.as_quat()
        transform.transform.rotation.x = q[0]
        transform.transform.rotation.y = q[1]
        transform.transform.rotation.z = q[2]
        transform.transform.rotation.w = q[3]

        # Publish the static transform
        self.tf_static_broadcaster.sendTransform(transform)



def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    object_tf_node = Mock_Baselink()
    rclpy.spin(object_tf_node)
    object_tf_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()