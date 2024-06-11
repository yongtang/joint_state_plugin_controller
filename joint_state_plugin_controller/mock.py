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


def main(args=None):
    with can.Bus(interface="udp_multicast") as bus:
        rclpy.logging.get_logger("can_mock").info("Start")
        for msg in bus:
            rclpy.logging.get_logger("can_mock").info("{}".format(msg.data))
            (position,) = struct.unpack("d", msg.data)
            rclpy.logging.get_logger("can_mock").info("{}".format(position))
        rclpy.logging.get_logger("can_mock").info("Final")


if __name__ == "__main__":
    main()
