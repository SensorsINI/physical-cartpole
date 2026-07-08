import serial
import struct
import termios
import time
import pandas as pd

PING_TIMEOUT = 1.0  # Seconds
CALIBRATE_TIMEOUT = 20.0  # Seconds
HARDWARE_EXPERIMENT_TIMEOUT = 30.0  # Seconds
READ_STATE_TIMEOUT = 1.0  # Seconds
SERIAL_SOF = 0xAA
CMD_PING = 0xC0
CMD_STREAM_ON = 0xC1
CMD_CALIBRATE = 0xC2
CMD_CONTROL_MODE = 0xC3
CMD_SET_PID_CONFIG = 0xC4
CMD_GET_PID_CONFIG = 0xC5
CMD_SET_CONTROL_CONFIG = 0xC6
CMD_GET_CONTROL_CONFIG = 0xC7
CMD_SET_MOTOR = 0xC8
CMD_SET_TARGET_POSITION = 0xC9
CMD_COLLECT_RAW_ANGLE = 0xCA
CMD_PC_CONTROL_MODE = 0xCB
CMD_STATE = 0xCC
CMD_SET_TARGET_EQUILIBRIUM = 0xCD
CMD_RUN_HARDWARE_EXPERIMENT = 0xCE
CMD_TRANSFER_BUFFERS = 0xD1
CMD_SET_ANGLE_FILTER = 0xD2
CMD_GET_DEAD_ZONE = 0xD3

# Hardware XADC filter modes; must match Firmware/Src/Zynq/goniometer_zynq.h
# and FPGA/CustomIPs/median_filter_hls/median_functions.h.
ANGLE_FILTER_MODE_RAW = 0
ANGLE_FILTER_MODE_MEDIAN = 1
ANGLE_FILTER_MODE_TRIMMED_MEAN = 2
# Must match firmware STATE_MESSAGE_LEN; CMD_STATE carries an 8-byte chip timestamp.
STATE_MESSAGE_LEN = 35
SERIAL_IO_ERRORS = (OSError, serial.SerialException, termios.error)


class Interface:
    def __init__(self):
        self.device = None
        self.msg = []
        self.prevPktNum = 1000
        self.start = None
        self.end = None

        self.encoderDirection = None
        self.calibration_in_progress = False
        self.calibration_completed = False

        self.hardware_experiment_length = 0

    def open(self, port, baud):
        self.port = port
        self.baud = baud
        self.device = serial.Serial(port, baudrate=baud, timeout=None)
        self.device.reset_input_buffer()

    def close(self):
        if self.device:
            self.pc_control_mode(False)
            self.control_mode(False)
            self.set_motor(0)
            time.sleep(2)
            self.device.close()
            self.device = None

    def clear_read_buffer(self):
        self.device.reset_input_buffer()
        self.prevPktNum = 1000

    def _write_message(self, msg):
        self.device.write(bytearray(msg))

    def _send_stream_output_request(self, en):
        msg = [SERIAL_SOF, CMD_STREAM_ON, 5, en]
        msg.append(self._crc(msg))
        self._write_message(msg)

    def _reconnect(self, timeout=None, restart_stream=False):
        print('\nSerial I/O error; reconnecting.')
        try:
            if self.device:
                self.device.close()
        except SERIAL_IO_ERRORS:
            pass

        time.sleep(1)
        self.device = serial.Serial(self.port, baudrate=self.baud, timeout=timeout)
        self.msg = []
        self.start = False
        self.prevPktNum = 1000

        if restart_stream:
            self._send_stream_output_request(True)
            try:
                self.clear_read_buffer()
            except SERIAL_IO_ERRORS:
                # If the newly opened port cannot be flushed either, let the
                # following read timeout/error drive the next reconnect attempt.
                self.msg = []
                self.prevPktNum = 1000

    def ping(self):
        msg = [SERIAL_SOF, CMD_PING, 4]
        msg.append(self._crc(msg))
        self._write_message(msg)
        self.prevPktNum = 1000
        return self._receive_reply(CMD_PING, 4, PING_TIMEOUT) == msg

    def stream_output(self, en):
        self._send_stream_output_request(en)
        self.clear_read_buffer()

    def start_calibration(self):
        if self.calibration_in_progress:
            return False

        msg = [SERIAL_SOF, CMD_CALIBRATE, 4]
        msg.append(self._crc(msg))

        self.clear_read_buffer()
        self.encoderDirection = None
        self.calibration_completed = False
        self.calibration_in_progress = True
        self._write_message(msg)

        return True

    def calibrate(self):
        if not self.start_calibration():
            return False

        reply = self._receive_reply(CMD_CALIBRATE, 5, CALIBRATE_TIMEOUT)
        self._handle_calibration_reply(reply)

        return True

    def run_hardware_experiment(self):
        msg = [SERIAL_SOF, CMD_RUN_HARDWARE_EXPERIMENT, 4]
        msg.append(self._crc(msg))
        self._write_message(msg)

        self.clear_read_buffer()

        reply = self._receive_reply(CMD_RUN_HARDWARE_EXPERIMENT, 6, HARDWARE_EXPERIMENT_TIMEOUT,
                                    reconnect_at_timeout=False)
        self.hardware_experiment_length = struct.unpack('H', bytes(reply[3:5]))[0]
        print(f'Hardware experiment finished with length {self.hardware_experiment_length}')

        msg = [SERIAL_SOF, CMD_TRANSFER_BUFFERS, 4]
        msg.append(self._crc(msg))
        self._write_message(msg)

        self.clear_read_buffer()
        variables_bytes = []
        message_length = 4 * self.hardware_experiment_length + 7
        for i in range(7):  # There are seven floats to receive
            c = self._receive_reply(CMD_TRANSFER_BUFFERS, message_length, HARDWARE_EXPERIMENT_TIMEOUT,
                                    reconnect_at_timeout=False)
            variables_bytes.append(c)

        message_length = self.hardware_experiment_length + 7  # target equilibrium,
        c = self._receive_reply(CMD_TRANSFER_BUFFERS, message_length, HARDWARE_EXPERIMENT_TIMEOUT,
                                reconnect_at_timeout=False)
        variables_bytes.append(c)

        variables = []
        unpack_string = f'<{self.hardware_experiment_length}f'
        for i in range(len(variables_bytes) - 1):
            variable_byte = variables_bytes[i]
            variable = struct.unpack(unpack_string, bytes(variable_byte[6:-1]))
            variables.append(variable)

        unpack_string = f'<{self.hardware_experiment_length}b'
        variable_byte = variables_bytes[7]
        variable = struct.unpack(unpack_string, bytes(variable_byte[6:-1]))
        variables.append(variable)

        # Creating a DataFrame
        df = pd.DataFrame({
            'time': variables[0],
            'angle': variables[1],
            'angleD': variables[2],
            'position': variables[3],
            'positionD': variables[4],
            'target_equilibrium': variables[7],
            'target_position': variables[5],
            'Q': variables[6],
        })

        # Saving to CSV without index
        df.to_csv('hardware_experiment_recording.csv', index=False)

        return True

    def control_mode(self, en):
        msg = [SERIAL_SOF, CMD_CONTROL_MODE, 5, 1 if en else 0]
        msg.append(self._crc(msg))
        self._write_message(msg)

    def pc_control_mode(self, en):
        msg = [SERIAL_SOF, CMD_PC_CONTROL_MODE, 5, 1 if en else 0]
        msg.append(self._crc(msg))
        self._write_message(msg)

    def set_config_PID(self, setPoint, smoothing, position_KP, position_KI, position_KD, angle_KP, angle_KI, angle_KD):
        msg = [SERIAL_SOF, CMD_SET_PID_CONFIG, 28]

        msg += list(struct.pack('f', position_KP))
        msg += list(struct.pack('f', position_KI))
        msg += list(struct.pack('f', position_KD))

        msg += list(struct.pack('f', angle_KP))
        msg += list(struct.pack('f', angle_KI))
        msg += list(struct.pack('f', angle_KD))

        msg.append(self._crc(msg))
        self._write_message(msg)

    def get_config_PID(self):
        msg = [SERIAL_SOF, CMD_GET_PID_CONFIG, 4]
        msg.append(self._crc(msg))
        self._write_message(msg)
        reply = self._receive_reply(CMD_GET_PID_CONFIG, 28)
        (setPoint, smoothing, position_KP, position_KI, position_KD, angle_KP, angle_KI, angle_KD) = struct.unpack(
            'h7f', bytes(reply[3:27]))
        return setPoint, smoothing, position_KP, position_KI, position_KD, angle_KP, angle_KI, angle_KD

    def set_config_control(self, controlLoopPeriodMs, controlSync, angle_hanging, avgLen, correct_motor_dynamics,
                           timesteps_for_derivative=1):
        """timesteps_for_derivative: on-chip derivative window in polling periods
        (firmware clamps to 1..20 and restarts its history buffers)."""
        msg = [SERIAL_SOF, CMD_SET_CONTROL_CONFIG, 16]
        msg += list(struct.pack('H', controlLoopPeriodMs))
        msg += list(struct.pack('?', controlSync))
        msg += list(struct.pack('f', angle_hanging))
        msg += list(struct.pack('H', avgLen))
        msg += list(struct.pack('?', correct_motor_dynamics))
        msg += list(struct.pack('H', timesteps_for_derivative))
        msg.append(self._crc(msg))
        self._write_message(msg)

    def get_config_control(self):
        msg = [SERIAL_SOF, CMD_GET_CONTROL_CONFIG, 4]
        msg.append(self._crc(msg))
        self._write_message(msg)
        reply = self._receive_reply(CMD_GET_CONTROL_CONFIG, 16)
        (controlLoopPeriodMs, controlSync, angle_hanging, avgLen, correct_motor_dynamics,
         timesteps_for_derivative) = struct.unpack('=H?fH?H', bytes(reply[3:15]))
        return controlLoopPeriodMs, controlSync, angle_hanging, avgLen, correct_motor_dynamics, timesteps_for_derivative

    def set_motor(self, speed):
        msg = [SERIAL_SOF, CMD_SET_MOTOR, 8]
        msg += list(struct.pack('i', speed))
        msg.append(self._crc(msg))
        self._write_message(msg)

    def set_target_position(self, target_position):
        msg = [SERIAL_SOF, CMD_SET_TARGET_POSITION, 8]
        msg += list(struct.pack('f', target_position))
        msg.append(self._crc(msg))
        self._write_message(msg)

    def set_target_equilibrium(self, target_equilibrium):
        msg = [SERIAL_SOF, CMD_SET_TARGET_EQUILIBRIUM, 8]
        msg += list(struct.pack('f', target_equilibrium))
        msg.append(self._crc(msg))
        self._write_message(msg)

    def collect_raw_angle(self, lenght=100, interval_us=100):
        msg = [SERIAL_SOF, CMD_COLLECT_RAW_ANGLE, 8, lenght % 256, lenght // 256, interval_us % 256, interval_us // 256]
        msg.append(self._crc(msg))
        self._write_message(msg)
        reply = self._receive_reply(CMD_COLLECT_RAW_ANGLE, 4 + 2 * lenght, crc=False, timeout=100)
        return struct.unpack(str(lenght) + 'H', bytes(reply[3:3 + 2 * lenght]))

    def collect_angle_pairs(self, length=1000, interval_us=100):
        """Collect (filtered16, raw16) pairs from the FPGA angle filter block.

        Both values are full 16-bit register reads (12-bit ADC code left-aligned,
        i.e. multiplied by 16; the filtered value carries 4 fractional bits from
        averaging). Motor is stopped and the control interrupt is suspended for
        the duration of the collection. Firmware caps length at 16384.
        """
        msg = [SERIAL_SOF, CMD_COLLECT_RAW_ANGLE, 9,
               length % 256, length // 256,
               interval_us % 256, interval_us // 256,
               1]  # format 1 = paired full-16-bit filtered+raw
        msg.append(self._crc(msg))
        self._write_message(msg)
        timeout = max(100.0, 2 * length * interval_us * 1e-6 + 4 * length / (self.baud / 10) + 10)
        reply = self._receive_reply(CMD_COLLECT_RAW_ANGLE, 4 + 4 * length, crc=False, timeout=timeout)
        values = struct.unpack(str(2 * length) + 'H', bytes(reply[3:3 + 4 * length]))
        filtered = values[0::2]
        raw = values[1::2]
        return filtered, raw

    def collect_angle_deadzone(self, length=1000, interval_us=100):
        """Collect (filtered16, raw16, dz_window, dz_status, dz_age) tuples.

        dz_window: number of near-rail samples in the hardware filter window.
        dz_status: bit0 = latest sample at low rail, bit1 = at high rail.
        dz_age: XADC samples (~2.2 us each) since last rail contact, saturating
        at 0xFFFF; captures rail touches that occur between recorded samples.
        Firmware caps length at 16384.
        """
        msg = [SERIAL_SOF, CMD_COLLECT_RAW_ANGLE, 9,
               length % 256, length // 256,
               interval_us % 256, interval_us // 256,
               2]  # format 2 = pairs + dead-zone tracking
        msg.append(self._crc(msg))
        self._write_message(msg)
        timeout = max(100.0, 2 * length * interval_us * 1e-6 + 8 * length / (self.baud / 10) + 10)
        reply = self._receive_reply(CMD_COLLECT_RAW_ANGLE, 4 + 8 * length, crc=False, timeout=timeout)
        records = struct.unpack('<' + 'HHBBH' * length, bytes(reply[3:3 + 8 * length]))
        filtered = records[0::5]
        raw = records[1::5]
        dz_window = records[2::5]
        dz_status = records[3::5]
        dz_age = records[4::5]
        return filtered, raw, dz_window, dz_status, dz_age

    def get_dead_zone(self):
        """Snapshot of the hardware dead-zone registers.

        Returns (status, window, age, low_count, high_count); the counts are
        cumulative near-rail sample counters since firmware boot.
        """
        msg = [SERIAL_SOF, CMD_GET_DEAD_ZONE, 4]
        msg.append(self._crc(msg))
        self._write_message(msg)
        reply = self._receive_reply(CMD_GET_DEAD_ZONE, 18, timeout=5.0, reconnect_at_timeout=False)
        return struct.unpack('<HHHII', bytes(reply[3:17]))

    def set_angle_filter(self, window_size, trim_count, filter_mode):
        """Reconfigure the FPGA angle filter block at runtime (Zynq only).

        filter_mode: 0 = raw passthrough, 1 = median, 2 = trimmed mean
        (trim_count = 0 gives a pure average). window_size is capped at 64 in
        hardware. The firmware echoes the packet back as confirmation.
        """
        msg = [SERIAL_SOF, CMD_SET_ANGLE_FILTER, 8,
               window_size % 256, window_size // 256,
               trim_count, filter_mode]
        msg.append(self._crc(msg))
        self._write_message(msg)
        reply = self._receive_reply(CMD_SET_ANGLE_FILTER, 8, timeout=5.0, reconnect_at_timeout=False)
        (echoed_window, echoed_trim, echoed_mode) = struct.unpack('HBB', bytes(reply[3:7]))
        if (echoed_window, echoed_trim, echoed_mode) != (window_size, trim_count, filter_mode):
            raise RuntimeError(
                f"Filter config echo mismatch: sent {(window_size, trim_count, filter_mode)}, "
                f"got {(echoed_window, echoed_trim, echoed_mode)}")

    def read_state(self):
        if not self.calibration_in_progress:
            try:
                self.clear_read_buffer()
            except SERIAL_IO_ERRORS:
                self._reconnect(timeout=READ_STATE_TIMEOUT, restart_stream=True)
        message_length = STATE_MESSAGE_LEN
        timeout = CALIBRATE_TIMEOUT if self.calibration_in_progress else READ_STATE_TIMEOUT
        reply = self._receive_reply(
            CMD_STATE,
            message_length,
            timeout,
            reconnect_at_timeout=not self.calibration_in_progress,
        )

        (angle, angleD, position, target_position, command, invalid_steps, time_difference,
         time_current_measurement_chip, latency, latency_violation) = struct.unpack('=hfhfhBIQ2H',
                                                                                    bytes(reply[3:message_length - 1]))

        return angle, angleD, position, target_position, command, invalid_steps, time_difference / 1e6, time_current_measurement_chip / 1e6, latency / 1e5, latency_violation

    def _receive_reply(self, cmd, cmdLen, timeout=None, crc=True, reconnect_at_timeout=True):
        self.device.timeout = timeout
        self.start = False

        while True:
            try:
                c = self.device.read()
            except SERIAL_IO_ERRORS:
                if reconnect_at_timeout:
                    self._reconnect(timeout=timeout, restart_stream=True)
                    continue
                raise
            # Timeout: reopen device, start stream, reset msg and try again
            if len(c) == 0:
                if reconnect_at_timeout:
                    print('\n_receive_reply: no response; reconnecting.')
                    self._reconnect(timeout=timeout, restart_stream=True)
            else:
                self.msg.append(ord(c))
                if self.start == False:
                    self.start = time.time()

            while True:
                # print('I am looping! Hurra!')
                # Message must start with SOF character
                if len(self.msg) < 3:
                    break

                if self.msg[0] != SERIAL_SOF:
                    # print('\nMissed SERIAL_SOF')
                    del self.msg[0]
                    continue

                packet_len = self.msg[2] if cmdLen < 256 else cmdLen
                if len(self.msg) < packet_len:
                    break

                if cmd != CMD_CALIBRATE and self.msg[1] == CMD_CALIBRATE and packet_len == 5:
                    if self.msg[4] == self._crc(self.msg[:4]):
                        self._handle_calibration_reply(self.msg[:5])
                        del self.msg[:5]
                        continue

                    print('\nCRC Failed.')
                    del self.msg[0]
                    continue

                # Check command
                if self.msg[1] != cmd:
                    print('\nMissed CMD.')
                    del self.msg[0]
                    continue

                # Check message packet length
                if self.msg[2] != cmdLen and cmdLen < 256:
                    print('\nWrong Packet Length.')
                    del self.msg[0]
                    continue

                # Verify integrity of message
                if crc and self.msg[cmdLen - 1] != self._crc(self.msg[:cmdLen - 1]):
                    print('\nCRC Failed.')
                    del self.msg[0]
                    continue

                self.device.timeout = None
                reply = self.msg[:cmdLen]
                del self.msg[:cmdLen]
                return reply

    def _handle_calibration_reply(self, reply):
        self.encoderDirection = struct.unpack('b', bytes(reply[3:4]))[0]
        self.calibration_in_progress = False
        self.calibration_completed = True

    def _crc(self, msg):
        crc8 = 0x00

        for i in range(len(msg)):
            val = msg[i]
            for b in range(8):
                sum = (crc8 ^ val) & 0x01
                crc8 >>= 1
                if sum > 0:
                    crc8 ^= 0x8C
                val >>= 1

        return crc8
