import pytest
import launch
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
        output="screen",
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
        output="screen",
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
            launch.actions.ExecuteProcess(
                cmd=[
                    "ros2",
                    "launch",
                    "joint_state_plugin_controller",
                    "launch.mock.py",
                ],
                output="screen",
            ),
        ]
    )


@pytest.mark.launch(fixture=launch_description)
def test_read_stdout(proc_sub, launch_context):
    def validate_output(output):
        true = """
  frame_id: ''
name:
- mock
position:
- 0.0
velocity: []
effort: []
---
header:
  stamp:
"""
        assert true in output, output

    launch_pytest.tools.process.assert_output_sync(
        launch_context, proc_sub, validate_output, timeout=5
    )
    yield
