from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                executable="controller",
                package="joint_state_can_controller",
                output="screen",
                arguments=[
                    "--ros-args",
                    "--disable-external-lib-logs",
                ],
                parameters=[
                    {
                        "interface": "virtual",
                        "plugin": "joint_state_can_controller.mock.CAN",
                        "params": ["interface=virtual"],
                    }
                ],
            ),
        ]
    )
