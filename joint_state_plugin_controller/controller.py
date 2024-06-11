import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState

import importlib


class ControllerNode(Node):
    def __init__(self):
        super().__init__("joint_state_plguin_controller")

        self.declare_parameter("joint_states", "~/joint_states")
        self.declare_parameter("joint_commands", "~/joint_commands")
        self.declare_parameter("plugin", rclpy.Parameter.Type.STRING)
        self.declare_parameter("params", rclpy.Parameter.Type.STRING_ARRAY)

        self.joint_states_ = (
            self.get_parameter("joint_states").get_parameter_value().string_value
        )
        self.joint_commands_ = (
            self.get_parameter("joint_commands").get_parameter_value().string_value
        )

        plugin = self.get_parameter("plugin").get_parameter_value().string_value
        params = dict(
            map(
                lambda e: tuple(e.split("=", maxsplit=1)),
                self.get_parameter("params").get_parameter_value().string_array_value,
            )
        )

        self.get_logger().info(
            "parameters: joint_states={} joint_commands={} plugin={} params={}".format(
                self.joint_states_,
                self.joint_commands_,
                plugin,
                params,
            )
        )
        plugin_module, plugin_class = plugin.rsplit(".", maxsplit=1)
        self.plugin_ = getattr(importlib.import_module(plugin_module), plugin_class)(
            **params
        )

        self.publisher_ = self.create_publisher(JointState, self.joint_states_, 10)
        self.subscription_ = self.create_subscription(
            JointState, self.joint_commands_, self.subscription_callback, 10
        )

    def __enter__(self):
        if self.plugin_:
            self.plugin_.__enter__()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        if self.plugin_:
            self.plugin_.__exit__(exc_type, exc_value, traceback)
        return None

    def subscription_callback(self, msg):
        self.get_logger().info(f"subscription: joint_commands={msg}")
        assert (
            len(msg.name) == len(msg.position) or not msg.position
        ), f"name({msg.name}) vs. position({msg.position})"
        assert (
            len(msg.name) == len(msg.velocity) or not msg.velocity
        ), f"name({msg.name}) vs. position({msg.velocity})"
        assert (
            len(msg.name) == len(msg.effort) or not msg.effort
        ), f"name({msg.name}) vs. position({msg.effort})"

        for index, name in enumerate(msg.name):
            position = msg.position[index] if msg.position else None
            velocity = msg.velocity[index] if msg.velocity else None
            effort = msg.effort[index] if msg.effort else None
            self.plugin_.send_command(name, position, velocity, effort)


def main(args=None):
    rclpy.init(args=args)

    with ControllerNode() as node:
        try:
            rclpy.spin(node)
        finally:
            node.destroy_node()
            rclpy.try_shutdown()


if __name__ == "__main__":
    main()
