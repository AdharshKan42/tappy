import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from geometry_msgs.msg import TransformStamped
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import pdb
from std_msgs.msg import String
from interbotix_common_modules.common_robot.robot import robot_shutdown, robot_startup
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS


class DynamicMoveArm(Node):

    def __init__(self):
        super().__init__("Dynamic_move_arm")
        self.stick_length = 0.05
        self.push_dist = 0.01
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # self.timer = self.create_timer(1.0, self.on_timer)
        self.create_subscription(String, "next_arm_instruction", self.type_character, 1)

        self.get_logger().info("Dynamic_move_arm node started!")
        self.bot = InterbotixManipulatorXS(
            robot_model="rx200",
            group_name="arm",
            gripper_name="gripper",
        )
        self.get_logger().info("startup!")
        robot_startup()
        # self.bot.arm.go_to_home_pose()

    def type_character(self, char):
        char_tf_name = "key_" + char
        try:
            # the name of the keyboard key frame has to be whatever you define it as
            transform = self.tf_buffer.lookup_transform(
                "rx200/base_link",
                char_tf_name,
                rclpy.time.Time(),
            )
            self.get_logger().info("this is the transform from base to sensor frame")
            # self.get_logger().info(transform.transform.translation.x)

        except TransformException as e:
            # self.get_logger().info("blash ")
            self.get_logger().error(e)

        # pdb.set_trace()
        x = transform.transform.translation.x - self.stick_length
        y = transform.transform.translation.y
        z = transform.transform.translation.z
        self.bot.arm.set_ee_pose_components(x=x, y=y, z=z, roll=0, pitch=0)

        x = transform.transform.translation.x - self.stick_length + self.push_dist
        self.bot.arm.set_ee_pose_components(x=x, y=y, z=z, roll=0, pitch=0)

        x = transform.transform.translation.x - self.stick_length
        self.bot.arm.set_ee_pose_components(x=x, y=y, z=z, roll=0, pitch=0)

        msg = char + " typed"
        self.status_publisher.publish(String(data=msg))


def main():
    rclpy.init()
    node = DynamicMoveArm()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()
