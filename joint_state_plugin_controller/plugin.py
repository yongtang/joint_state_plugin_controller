import abc
import contextlib

import can


class Plugin(contextlib.AbstractContextManager):
    @abc.abstractmethod
    def send_command(self, name, position, velocity, effort):
        raise NotImplementedError

    @abc.abstractmethod
    def notify_state(self, callback):
        raise NotImplementedError

    @abc.abstractmethod
    def query_state(self):
        raise NotImplementedError


class CAN(Plugin):
    def __init__(self, **params):
        self._bus_ = can.Bus(**params)

    @abc.abstractmethod
    def send_command(self, name, position, velocity, effort):
        raise NotImplementedError

    @abc.abstractmethod
    def recv_state(self, msg):
        raise NotImplementedError

    @abc.abstractmethod
    def query_state(self):
        raise NotImplementedError

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

    def notify_state(self, callback):
        can.Notifier(self.bus, [CAN.Listener(callback, self.recv_state)])

    class Listener(can.Listener):
        def __init__(self, callback, recv_state):
            self.callback = callback
            self.recv_state = recv_state

        def on_message_received(self, msg):
            name, position, velocity, effort = self.recv_state(msg)
            if name is not None:
                self.callback(name, position, velocity, effort)

        def stop(self):
            pass
