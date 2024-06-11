import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState


class ControllerNode(Node):
    def __init__(self):
        super().__init__("joint_state_can_controller")

        self.declare_parameter("joint_states", "~/joint_states")
        self.declare_parameter("joint_commands", "~/joint_commands")

        self.joint_states = (
            self.get_parameter("joint_states").get_parameter_value().string_value
        )
        self.joint_commands = (
            self.get_parameter("joint_commands").get_parameter_value().string_value
        )

        self.get_logger().info(
            f"parameters: joint_states={self.joint_states} joint_commands={self.joint_commands}"
        )

        self.publisher = self.create_publisher(JointState, self.joint_states, 10)
        self.subscription = self.create_subscription(
            JointState, self.joint_commands, self.subscription_callback, 10
        )

    def subscription_callback(self, msg):
        self.get_logger().info(f"subscription: joint_states={msg}")


def main(args=None):
    rclpy.init(args=args)

    node = ControllerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
