import pytest
import launch
import launch_ros
import launch_pytest


@pytest.fixture
def proc_pub():
    return launch.actions.ExecuteProcess(
        cmd=[
            "ros2",
            "topic",
            "pub",
            "/joint_state_plugin_controller/joint_commands",
            "sensor_msgs/msg/JointState",
            "{name: ['mock'], position: [0.0], velocity: [1.0], effort: [0.0]}",
        ],
        cached_output=True,
    )


@pytest.fixture
def proc_sub():
    return launch.actions.ExecuteProcess(
        cmd=[
            "ros2",
            "topic",
            "echo",
            "/joint_state_plugin_controller/joint_states",
        ],
        cached_output=True,
    )


@launch_pytest.fixture
def launch_description(proc_pub, proc_sub):
    return launch.LaunchDescription(
        [
            proc_pub,
            launch.actions.RegisterEventHandler(
                event_handler=launch.event_handlers.OnProcessStart(
                    target_action=proc_pub,
                    on_start=[
                        launch.actions.TimerAction(
                            period=2.0,
                            actions=[
                                launch.actions.LogInfo(
                                    msg=f"Sub waited 2 seconds; start"
                                ),
                                proc_sub,
                            ],
                        ),
                    ],
                )
            ),
            launch_ros.actions.Node(
                executable="mock",
                package="joint_state_plugin_controller",
                output="screen",
            ),
            launch_ros.actions.Node(
                executable="controller",
                package="joint_state_plugin_controller",
                output="screen",
                parameters=[
                    {
                        "frequency": 10.0,
                        "plugin": "joint_state_plugin_controller.mock.CAN",
                        "params": ["interface=udp_multicast"],
                    }
                ],
            ),
        ]
    )


@pytest.mark.launch(fixture=launch_description)
def test_read_stdout(proc_sub, launch_context):
    def validate_output(output):
        true = """
header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: ''
name:
- mock
position:
- 0.0
velocity: []
effort: []
"""
        assert true in output, output

    launch_pytest.tools.process.assert_output_sync(
        launch_context, proc_sub, validate_output, timeout=5
    )
    yield
