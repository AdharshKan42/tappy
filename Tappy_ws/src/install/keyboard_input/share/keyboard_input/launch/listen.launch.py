import launch
import launch_ros.actions


def generate_launch_description():
    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(
                package="keyboard_input",
                executable="listen_keystroke",
                output="screen",
                name="listen_keystroke",
                parameters=[{"exit_on_esc": True}],
            ),
        ]
    )
