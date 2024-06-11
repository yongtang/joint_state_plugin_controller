import abc
import contextlib

import can


class Plugin(contextlib.AbstractContextManager):
    @abc.abstractmethod
    def send_command(self, name, position, velocity, effort):
        raise NotImplementedError


class CAN(Plugin):
    def __init__(self, **params):
        self._bus_ = can.Bus(**params)

    def __enter__(self):
        if self.bus:
            self.bus.__enter__()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        if self.bus:
            self.bus.__exit__(exc_type, exc_value, traceback)
        return None

    @property
    def bus(self):
        return self._bus_
