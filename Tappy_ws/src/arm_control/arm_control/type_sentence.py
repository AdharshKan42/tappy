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
import math
from scipy.spatial.transform import Rotation


class DynamicMoveArm(Node):

    def __init__(self):
        super().__init__("Dynamic_move_arm")
        self.stick_length = 0.09398
        self.push_dist = 0.005
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.existing_key = ""

        # self.timer = self.create_timer(1.0, self.on_timer)
        self.create_subscription(String, "next_arm_instruction", self.type_character, 1)
        self.create_subscription(
            String, "perception/object_tf/existing_key", self.update_existing_key, 1
        )

        self.existing_key = "t"
        self.get_logger().info("Dynamic_move_arm node started!")
        self.bot = InterbotixManipulatorXS(
            robot_model="rx200",
            group_name="arm",
            gripper_name="gripper",
        )
        self.get_logger().info("startup!")
        robot_startup()
        # self.bot.arm.go_to_home_pose()

    def type_character(self, msg):
        print("im in tpakdslj;f")
        if self.existing_key != "":
            char = msg.data
            existing_key_tf = "key_" + self.existing_key.upper()
            try:
                self.get_logger().info("looking up transform")
                # the name of the keyboard key frame has to be whatever you define it as
                transform = self.tf_buffer.lookup_transform(
                    "rx200/base_link",
                    existing_key_tf,
                    rclpy.time.Time(),
                )
                self.get_logger().info(
                    f"this is the transform from {existing_key_tf} to base link : {transform}"
                )
                tf_diff = [0, 0]
                if self.existing_key != char:
                    tf_diff = self.map_keyboard_keys(self.existing_key, char)
                    self.get_logger().info(
                        f"this is the difference between {self.existing_key} and {char} : {tf_diff}")
                self.get_logger().info("this is the transform from base to sensor frame")

            except TransformException as e:
                self.get_logger().error(e)
                return

            # Extract roll, pitch, yaw from the transform's quaternion
            rotation = Rotation.from_quat([
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w,
            ])
            roll, pitch, yaw = rotation.as_euler('xyz')

            deg_90 = math.radians(90)
            # Move to the target position
            x = transform.transform.translation.x 
            y = transform.transform.translation.y + tf_diff[0]
            z = transform.transform.translation.z + tf_diff[1] + self.stick_length
            self.bot.arm.set_ee_pose_components(x=x, y=y, z=z, roll=roll, pitch=deg_90, yaw=yaw)

            # Push the key
            z = transform.transform.translation.z + self.stick_length - self.push_dist
            self.bot.arm.set_ee_pose_components(x=x, y=y, z=z, roll=roll, pitch=deg_90, yaw=yaw)

            # Return to the original position
            z = transform.transform.translation.x + self.stick_length
            self.bot.arm.set_ee_pose_components(x=x, y=y, z=z, roll=roll, pitch=deg_90, yaw=yaw)

    def update_existing_key(self, msg):
        """Callback to update the existing key from the topic."""
        self.existing_key = msg.data.lower()
        self.get_logger().info(f"Updated existing key: {self.existing_key}")

    def transform_coordinates(self, key_1, key_2):
        y1, z1 = key_1
        y2, z2 = key_2
        return (y1 - y2, z1 - z2)

    def map_keyboard_keys(self, key1, key2):
        key_dict = {
            "1": [0.0194, 0],
            "2": [0.0386, 0],
            "3": [0.0586, 0],
            "4": [0.0775, 0],
            "5": [0.098, 0],
            "6": [0.117, 0],
            "7": [0.1363, 0],
            "8": [0.1559, 0],
            "9": [0.1746, 0],
            "0": [0.1939, 0],
            "-": [0.2138, 0],
            "=": [0.2327, 0],
            "delete": [0.2668, 0],
            "tab": [0.0191, 0.02],
            "q": [0.029, 0.02],
            "w": [0.0494, 0.02],
            "e": [0.068, 0.02],
            "r": [0.088, 0.02],
            "t": [0.1073, 0.02],
            "y": [0.1269, 0.02],
            "u": [0.1459, 0.02],
            "i": [0.1646, 0.02],
            "o": [0.1844, 0.02],
            "p": [0.2029, 0.02],
            "[": [0.2222, 0.02],
            "]": [0.2428, 0.02],
            "capslock": [0.0254, 0.02],
            "a": [0.0345, 0.0386],
            "s": [0.0526, 0.0386],
            "d": [0.0724, 0.0386],
            "f": [0.0922, 0.0386],
            "g": [0.1153, 0.0386],
            "h": [0.1311, 0.0386],
            "j": [0.1507, 0.0386],
            "k": [0.1694, 0.0386],
            "l": [0.1889, 0.0386],
            ";": [0.2079, 0.0386],
            "enter": [0.1042, 0.0386],
            "shift": [0.0158, 0.0583],
            "z": [0.0414, 0.0583],
            "x": [0.0622, 0.0583],
            "c": [0.0816, 0.0583],
            "v": [0.1021, 0.0583],
            "b": [0.1215, 0.0583],
            "n": [0.1407, 0.0583],
            "m": [0.1601, 0.0583],
            ",": [0.1799, 0.0583],
            ".": [0.1978, 0.0583],
            "/": [0.2184, 0.0583],
            "leftctrl": [0.0013, 0.078],
            "space": [0.1483, 0.078],
        }

        key_one = key_dict[key1]
        key_two = key_dict[key2]
        # print(key_one, key_two)
        return self.transform_coordinates(key_one, key_two)


def main():
    rclpy.init()
    node = DynamicMoveArm()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()
