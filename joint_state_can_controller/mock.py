import rclpy
from . import plugin


class CAN(plugin.CAN):
    def __init__(self, **params):
        super().__init__(**params)

    def send_command(self, name, position, velocity, effort):
        rclpy.logging.get_logger("{}".format("mock")).info(
            "{}: position={}, velocity={}, effort={}".format(
                name, position, velocity, effort
            )
        )
        raise NotImplementedError
