import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from geometry_msgs.msg import TransformStamped, PoseArray
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
        fingers_to_ee_gripper = 0.027575
        self.stick_length = 0.13 - fingers_to_ee_gripper
        self.push_dist = 0.004 # 4mm key press travel
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info("Dynamic_move_arm node started!")
        self.bot = InterbotixManipulatorXS(
            robot_model="rx200",
            group_name="arm",
            gripper_name="gripper",
        )
        self.get_logger().info("startup!")
        robot_startup()

        self.existing_key = None
        self.timer_period = 0.5  # seconds
        self.timer = self.create_timer(self.timer_period, self.update_existing_key)

        self.search_algo()

        # self.timer = self.create_timer(1.0, self.on_timer)
        self.create_subscription(String, "next_arm_instruction", self.type_character, 1)
        
        
        # self.bot.arm.go_to_home_pose()
    def search_algo(self):
        # draw an exploratory rectangle
        # pdb.set_trace()
        self.bot.arm.set_ee_pose_components(
                x=0.10, y=0.25, z=0.22, pitch=math.radians(90),
            )
        
        # pdb.set_trace()
        self.bot.arm.set_ee_pose_components(
                x=0.20, y=0.2, z=0.22, pitch=math.radians(90),
            )
        
        self.bot.arm.set_ee_pose_components(
                x=0.20, y=0.1, z=0.22, pitch=math.radians(90),
            )
        self.bot.arm.set_ee_pose_components(
                x=0.20, y=0.0, z=0.22, pitch=math.radians(90),
            )
        self.bot.arm.set_ee_pose_components(
                x=0.20, y=-0.1, z=0.22, pitch=math.radians(90),
            )
        self.bot.arm.set_ee_pose_components(
                x=0.20, y=-0.2, z=0.22, pitch=math.radians(90),
            )
        
        self.bot.arm.set_ee_pose_components(
                x=0.10, y=-0.25, z=0.22, pitch=math.radians(90),
            )

    def type_character(self, msg):
        if self.existing_key_tf is not None:

            char = msg.data
            transform = self.existing_key_tf

            tf_diff = self.map_keyboard_keys("aruco", char)
            self.get_logger().info(
                f"this is the difference between aruco and {char} : {tf_diff}"
            )
            
            # print("transform_x: ", transform.transform.translation.x)
            # print("transform_y: ", transform.transform.translation.y)
            # print("transform_z: ", transform.transform.translation.z)

            # pdb.set_trace()

            # tf_diff represents the difference in width and height coordinates between the two keys
            # + width = -y in robot frame
            # + height = -x in robot frame

            mapping_width, mapping_height = tf_diff
            mapping_width += (0.009899999999999992 + 0.005)
            mapping_height += (0.019699999999999995 + 0.005)
            # Extract roll, pitch, yaw from the transform's quaternion
            rotation = Rotation.from_quat(
                [
                    transform.transform.rotation.x,
                    transform.transform.rotation.y,
                    transform.transform.rotation.z,
                    transform.transform.rotation.w,
                ]
            )
            roll, pitch, yaw = rotation.as_euler("xyz")
            # roll, yaw = 0, 0

            deg_90 = math.radians(90)
            # Move to the target position
            # pdb.set_trace()

            self.get_logger().info(
                f"Moving to target position above key {char}")
            
            x = transform.transform.translation.x + mapping_height
            y = transform.transform.translation.y + mapping_width 
            z = transform.transform.translation.z + self.stick_length + 0.08
            self.bot.arm.set_ee_pose_components(
                x=x, y=y, z=z, pitch=deg_90, 
            )

            

            # pdb.set_trace()

            self.get_logger().info(
                f"Pushing key {char}")

            # Push the key
            z = transform.transform.translation.z + self.stick_length + 0.04
            self.bot.arm.set_ee_pose_components(
                x=x, y=y, z=z, pitch=deg_90, 
            )


            # Return to the original position

            self.get_logger().info(
                f"returning to original target key {char} position")
            
            z = transform.transform.translation.z  + self.stick_length + 0.08
            self.bot.arm.set_ee_pose_components(
                x=x, y=y, z=z, pitch=deg_90,
            )

    def update_existing_key(self):
        """Callback to update the existing key from the topic."""
        try:
            transform = self.tf_buffer.lookup_transform(
                "rx200/base_link",
                "aruco_id_4",
                rclpy.time.Time(),
            )
            self.get_logger().info(f"aruco tag found!")

            self.existing_key_tf = transform

        except:
            # self.get_logger().error(e)
            return


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
            "backspace": [0.2668, 0],
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
            "aruco": [0.3718, -0.023]
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
