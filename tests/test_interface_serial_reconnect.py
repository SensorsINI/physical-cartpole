import struct
import sys
import termios
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "Driver"))

from DriverFunctions import interface as interface_module
from DriverFunctions.interface import CMD_STATE, SERIAL_SOF, STATE_MESSAGE_LEN, Interface


class BrokenFlushDevice:
    def __init__(self):
        self.closed = False

    def reset_input_buffer(self):
        raise termios.error(5, "Input/output error")

    def close(self):
        self.closed = True


class StreamingStateDevice:
    def __init__(self, packet):
        self.packet = bytes(packet)
        self.index = 0
        self.timeout = None
        self.writes = []
        self.closed = False
        self.reset_count = 0

    def reset_input_buffer(self):
        self.reset_count += 1

    def write(self, data):
        self.writes.append(bytes(data))

    def read(self):
        if self.index >= len(self.packet):
            return b""
        value = self.packet[self.index]
        self.index += 1
        return bytes([value])

    def close(self):
        self.closed = True


def _state_packet(interface):
    payload = struct.pack(
        "=hfhfhBIQ2H",
        -958,
        0.25,
        232,
        0.0,
        28,
        0,
        10000,
        123456,
        660,
        0,
    )
    packet = [SERIAL_SOF, CMD_STATE, STATE_MESSAGE_LEN]
    packet.extend(payload)
    packet.append(interface._crc(packet))
    return packet


def test_read_state_reconnects_when_input_buffer_flush_fails(monkeypatch):
    interface = Interface()
    interface.port = "/dev/ttyUSB-test"
    interface.baud = 230400
    broken_device = BrokenFlushDevice()
    interface.device = broken_device

    replacement_device = StreamingStateDevice(_state_packet(interface))
    monkeypatch.setattr(interface_module.time, "sleep", lambda _: None)
    monkeypatch.setattr(interface_module.serial, "Serial", lambda *_, **__: replacement_device)

    state = interface.read_state()

    assert broken_device.closed
    assert replacement_device.writes
    assert replacement_device.writes[0][1] == interface_module.CMD_STREAM_ON
    assert state == (-958, 0.25, 232, 0.0, 28, 0, 0.01, 0.123456, 0.0066, 0)
