import time
import struct

from . import plugin

import rclpy
import rclpy.node
import rclpy.logging
import can


class CAN(plugin.CAN):
    def __init__(self, **params):
        super().__init__(**params)
        rclpy.logging.get_logger("{}".format("mock")).info("params={}".format(params))

        self.joint = tuple(sorted(params["joint"].split(",")))

        rclpy.logging.get_logger("{}".format("mock")).info(
            "joint={}".format(self.joint)
        )

    def send_command(self, name, position, velocity, effort):
        e = self.joint.index(name)

        rclpy.logging.get_logger("{}".format("mock")).info(
            "{}: position={}, velocity={}, effort={}".format(
                name, position, velocity, effort
            )
        )
        msg = can.Message(
            arbitration_id=0x123 + e, data=bytearray(struct.pack("d", float(position)))
        )
        self.bus.send(msg)

    def recv_state(self, msg):
        rclpy.logging.get_logger("{}".format("mock")).info("msg={}".format(msg))
        if 0x321 <= msg.arbitration_id and msg.arbitration_id < 0x321 + len(self.joint):
            e = msg.arbitration_id - 0x321
            (position,) = struct.unpack("d", msg.data)
            return self.joint[e], position, None, None
        return None, None, None, None

    def query_state(self):
        msg = can.Message(arbitration_id=0x100, data=[])
        self.bus.send(msg)


class Mock(rclpy.node.Node):
    def __init__(self):
        super().__init__("mock")
        self.declare_parameter("joint", rclpy.Parameter.Type.STRING)
        self.joint = tuple(
            sorted(
                self.get_parameter("joint")
                .get_parameter_value()
                .string_value.split(",")
            )
        )
        self.get_logger().info("joint={}".format(self.joint))


def main(args=None):
    rclpy.init(args=args)

    joint = Mock().joint
    rclpy.logging.get_logger("{}".format("mock")).info("joint={}".format(joint))

    joint_state = list(None for e in joint)
    with can.Bus(interface="udp_multicast") as bus:
        rclpy.logging.get_logger("can_mock").info("Start")
        for msg in bus:
            rclpy.logging.get_logger("can_mock").info(
                "arbitration_id={}, data={}".format(msg.arbitration_id, msg.data)
            )
            if 0x123 <= msg.arbitration_id and msg.arbitration_id < 0x123 + len(joint):
                e = msg.arbitration_id - 0x123
                (position,) = struct.unpack("d", msg.data)
                rclpy.logging.get_logger("can_mock").info(
                    "joint={} joint_command={}".format(joint[e], position)
                )
                if joint_state[e] != position:
                    time.sleep(1.0)
                    joint_state[e] = position
            elif 0x100 <= msg.arbitration_id and msg.arbitration_id < 0x100 + len(
                joint
            ):
                e = msg.arbitration_id - 0x100
                if joint_state[e] is not None:
                    position = joint_state[e]
                    bus.send(
                        can.Message(
                            arbitration_id=0x321 + e,
                            data=bytearray(struct.pack("d", float(position))),
                        )
                    )

        rclpy.logging.get_logger("can_mock").info("Final")


if __name__ == "__main__":
    main()
