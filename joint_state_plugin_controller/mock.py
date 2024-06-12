import time
import struct

from . import plugin

import rclpy
import rclpy.logging
import can


class CAN(plugin.CAN):
    def __init__(self, **params):
        super().__init__(**params)

    def send_command(self, name, position, velocity, effort):
        rclpy.logging.get_logger("{}".format("mock")).info(
            "{}: position={}, velocity={}, effort={}".format(
                name, position, velocity, effort
            )
        )
        msg = can.Message(
            arbitration_id=0x123, data=bytearray(struct.pack("d", float(position)))
        )
        self.bus.send(msg)

    def recv_state(self, msg):
        rclpy.logging.get_logger("{}".format("mock")).info("msg={}".format(msg))
        if msg.arbitration_id == 0x321:
            (position,) = struct.unpack("d", msg.data)
            return "mock", position, None, None
        return None, None, None, None

    def query_state(self):
        msg = can.Message(arbitration_id=0x100, data=[])
        self.bus.send(msg)


def main(args=None):
    joint_state = None
    with can.Bus(interface="udp_multicast") as bus:
        rclpy.logging.get_logger("can_mock").info("Start")
        for msg in bus:
            rclpy.logging.get_logger("can_mock").info(
                "arbitration_id={}, data={}".format(msg.arbitration_id, msg.data)
            )
            if msg.arbitration_id == 0x123:
                (position,) = struct.unpack("d", msg.data)
                rclpy.logging.get_logger("can_mock").info(
                    "joint_command={}".format(position)
                )
                if joint_state != position:
                    time.sleep(1.0)
                    joint_state = position
            elif msg.arbitration_id == 0x100:
                if joint_state is not None:
                    position = joint_state
                    bus.send(
                        can.Message(
                            arbitration_id=0x321,
                            data=bytearray(struct.pack("d", float(position))),
                        )
                    )

        rclpy.logging.get_logger("can_mock").info("Final")


if __name__ == "__main__":
    main()
