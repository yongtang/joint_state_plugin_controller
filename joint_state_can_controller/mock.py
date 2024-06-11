import rclpy


def send_command(bus, name, position, velocity, effort):
    rclpy.logging.get_logger("{}".format("mock")).info("[{}] {}: position={}, velocity={}, effort={}".format(bus, name, position, velocity, effort))
