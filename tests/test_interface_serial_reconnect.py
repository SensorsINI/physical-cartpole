import serial
import struct
import sys
import termios
from pathlib import Path

import pytest


REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "Driver"))

from DriverFunctions import interface as interface_module
from DriverFunctions.interface import (
    CMD_COLLECT_RAW_ANGLE,
    CMD_STATE,
    MAX_RAW_ANGLE_SAMPLES_PER_PACKET,
    SERIAL_SOF,
    STATE_MESSAGE_LEN,
    Interface,
    UartHealth,
)


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
        "=hfhfhBIQ2HBf",
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
        0,
        1.0,
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
    assert state == (-958, 0.25, 232, 0.0, 28, 0, 0.01, 0.123456, 0.0066, 0, 0, 0, 0, 0, 1.0)


def test_collect_raw_angle_chunks_requests_to_fit_firmware_buffer(monkeypatch):
    interface = Interface()
    requested_chunks = []
    next_sample = 0

    monkeypatch.setattr(interface, "_write_message", lambda msg: requested_chunks.append(msg[3]))

    def receive_reply(command, reply_len, crc, timeout):
        nonlocal next_sample
        assert command == CMD_COLLECT_RAW_ANGLE
        chunk = (reply_len - 4) // 2
        values = range(next_sample, next_sample + chunk)
        next_sample += chunk
        return [SERIAL_SOF, command, reply_len] + list(struct.pack(f"{chunk}H", *values)) + [0]

    monkeypatch.setattr(interface, "_receive_reply", receive_reply)

    count = 2 * MAX_RAW_ANGLE_SAMPLES_PER_PACKET + 9
    samples = interface.collect_raw_angle(lenght=count, interval_us=100)

    assert requested_chunks == [98, 98, 9]
    assert samples == tuple(range(count))


class RaisingReadDevice:
    def __init__(self, error):
        self.error = error
        self.timeout = None
        self.closed = False

    def reset_input_buffer(self):
        pass

    def write(self, data):
        pass

    def read(self):
        raise self.error

    def close(self):
        self.closed = True


class TimeoutDevice:
    def __init__(self):
        self.timeout = None
        self.writes = []
        self.closed = False

    def reset_input_buffer(self):
        pass

    def write(self, data):
        self.writes.append(bytes(data))

    def read(self):
        return b""

    def close(self):
        self.closed = True


def test_read_state_survives_brief_io_error_and_records_incident(monkeypatch):
    interface = Interface()
    interface.port = "/dev/ttyUSB-test"
    interface.baud = 230400
    broken_device = RaisingReadDevice(OSError(5, "Input/output error"))
    interface.device = broken_device

    replacement_device = StreamingStateDevice(_state_packet(interface))
    clock = {"t": 0.0}
    monkeypatch.setattr(interface_module.time, "monotonic", lambda: clock["t"])
    monkeypatch.setattr(interface_module.time, "sleep", lambda seconds: clock.__setitem__("t", clock["t"] + seconds))
    monkeypatch.setattr(interface_module.serial, "Serial", lambda *_, **__: replacement_device)

    state = interface.read_state()

    assert state == (-958, 0.25, 232, 0.0, 28, 0, 0.01, 0.123456, 0.0066, 0, 0, 0, 0, 0, 1.0)
    assert broken_device.closed
    assert interface.uart.incident_count == 1
    assert not interface.uart.disconnected
    assert interface.uart.last_duration == 1.0
    assert interface.uart.total_downtime == 1.0
    assert interface.uart.reconnect_successes == 1
    assert interface.uart.last_error_message == "Input/output error"
    assert interface.uart.last_error_errno == 5
    assert "OK after 1 drops (last 1.0s, total 1.0s) last=Input/output error" in interface.uart.status_line()
    line = interface.uart.status_line()
    assert "wait min=" in line
    assert "p1=" in line
    assert "p50=" in line
    assert "skipped=" in line
    motor_writes = [w for w in replacement_device.writes if w[1] == interface_module.CMD_SET_MOTOR]
    assert motor_writes
    assert struct.unpack("i", motor_writes[0][3:7])[0] == 0


def test_read_state_repeated_timeout_does_not_raise(monkeypatch):
    interface = Interface()
    interface.port = "/dev/ttyUSB-test"
    interface.baud = 230400
    interface.device = TimeoutDevice()

    opens = {"n": 0}

    def fake_serial(*_, **__):
        opens["n"] += 1
        if opens["n"] > 2:
            raise serial.SerialException("could not open port")
        return TimeoutDevice()

    monkeypatch.setattr(interface_module.time, "sleep", lambda _: None)
    monkeypatch.setattr(interface_module.serial, "Serial", fake_serial)

    assert interface.read_state() is None
    assert interface.uart.disconnected
    assert interface.uart.incident_count == 1
    assert interface.uart.reconnect_successes == 0
    assert interface.uart.status_line().startswith("UART: DOWN")
    assert "(drop 1)" in interface.uart.status_line()

    assert interface.read_state() is None
    assert interface.uart.disconnected
    assert interface.uart.incident_count == 1

    assert interface.read_state() is None
    assert interface.uart.disconnected
    assert interface.uart.incident_count == 1
    assert interface.uart.reconnect_failures >= 1
    assert interface.uart.last_error_type is not None


def test_framing_rejects_are_counted_not_printed(capsys):
    interface = Interface()
    junk = [SERIAL_SOF, 0x00, STATE_MESSAGE_LEN]
    junk.extend([0] * (STATE_MESSAGE_LEN - 3))
    bad_crc = [SERIAL_SOF, CMD_STATE, STATE_MESSAGE_LEN]
    bad_crc.extend([0] * (STATE_MESSAGE_LEN - 4))
    bad_crc.append(0)
    stream = junk + bad_crc + _state_packet(interface)
    interface.device = StreamingStateDevice(stream)
    interface.port = "/dev/ttyUSB-test"
    interface.baud = 230400

    state = interface.read_state()

    assert state[0] == -958
    assert interface.uart.missed_cmd >= 1
    assert interface.uart.crc_failed >= 1
    assert interface.uart.framing_count() >= 2
    assert "framing crc=" in interface.uart.status_line()
    captured = capsys.readouterr().out
    assert "CRC Failed" not in captured
    assert "Missed CMD" not in captured


def test_reconnect_rediscovers_uart_when_old_path_is_gone(monkeypatch):
    interface = Interface()
    interface.port = "/dev/ttyUSB1"
    interface.baud = 230400
    broken_device = RaisingReadDevice(OSError(2, "No such file or directory"))
    interface.device = broken_device

    opens = []

    def fake_serial(port, *_, **__):
        opens.append(port)
        if port == "/dev/ttyUSB1":
            raise serial.SerialException("could not open port /dev/ttyUSB1")
        if port == "/dev/ttyUSB3":
            return StreamingStateDevice(_state_packet(interface))
        raise serial.SerialException(f"unexpected port {port}")

    monkeypatch.setattr(interface_module.time, "sleep", lambda _: None)
    monkeypatch.setattr(interface_module.serial, "Serial", fake_serial)
    monkeypatch.setattr(interface, "_candidate_reconnect_ports", lambda: ["/dev/ttyUSB1", "/dev/ttyUSB3"])

    state = interface.read_state()

    assert state[0] == -958
    assert opens == ["/dev/ttyUSB1", "/dev/ttyUSB3"]
    assert interface.port == "/dev/ttyUSB3"
    assert interface.uart.incident_count == 1
    assert not interface.uart.disconnected
    assert interface.uart.reconnect_successes == 1


def test_reconnect_records_failure_when_no_uart_path_exists(monkeypatch):
    interface = Interface()
    interface.port = "/dev/ttyUSB1"
    interface.baud = 230400
    interface.device = None
    interface.uart.begin_incident(OSError(2, "No such file or directory"))

    def fake_serial(*_, **__):
        raise serial.SerialException("could not open port /dev/ttyUSB1")

    monkeypatch.setattr(interface_module.time, "sleep", lambda _: None)
    monkeypatch.setattr(interface_module.serial, "Serial", fake_serial)
    monkeypatch.setattr(interface, "_candidate_reconnect_ports", lambda: ["/dev/ttyUSB1"])

    assert interface.read_state() is None
    assert interface.uart.disconnected
    assert interface.uart.incident_count == 1
    assert interface.uart.reconnect_failures >= 1
    assert "No such file" in interface.uart.status_line() or "could not open" in interface.uart.status_line()


def test_uart_health_skipped_and_lost_cycles():
    health = UartHealth()
    health.poll_period_s = 0.01
    health.record_read(0.008, lost=False, now=1.0)
    health.record_read(0.0102, lost=False, now=1.0102)  # 10.2 ms is not a missed tick
    health.record_read(0.025, lost=False, resync=True, now=1.035)
    health.record_read(1.0, lost=True, now=2.035)

    assert health.good_reads == 3
    assert health.lost_count == 1
    assert health.skipped_count == 2
    assert health.resync_reads == 1
    assert health.min_wait_s == pytest.approx(0.008)
    assert health.max_wait_s == pytest.approx(1.0)
    line = health.status_line()
    assert "skipped=2(>15ms)" in line
    assert "wait min=8ms p1=8ms" in line
    assert "max=1000ms" in line
    assert "lost=1" in line
    assert "resync=1" in line
