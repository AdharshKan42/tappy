from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="arm_control",
                namespace="dynamic_move_arm",
                executable="camera_to_stick",
                name="camera_to_stick",
            ),
            Node(
                package="arm_control",
                namespace="type_sentence",
                executable="type_sentence",
                name="type_sentence",
            ),
        ]
    )
