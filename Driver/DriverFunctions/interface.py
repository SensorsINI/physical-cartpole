import serial
import struct
import termios
import time
import pandas as pd

from DriverFunctions.zynq_serial import prefer_zynq_uart_port

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
CMD_SET_SECLOC_CONFIG = 0xD3
CMD_GET_SECLOC_INFO = 0xD4

SECLOC_BACKEND_NAMES = {0: 'SW', 1: 'PL', 2: 'PL-shadow'}

# Hardware XADC filter modes; must match Firmware/Src/Zynq/goniometer_zynq.h
# and FPGA/CustomIPs/median_filter_hls/median_functions.h.
ANGLE_FILTER_MODE_RAW = 0
ANGLE_FILTER_MODE_MEDIAN = 1
ANGLE_FILTER_MODE_TRIMMED_MEAN = 2
# Must match firmware STATE_MESSAGE_LEN; CMD_STATE carries an 8-byte chip timestamp
# and 1 SecLoc telemetry byte (bit 0 = skipped_update, bit 1 = gate_skipped,
# bit 2 = step computed by the PL backend, bit 3 = PL fault: PL backend selected but
# the PL block is absent or the transaction failed; the step output zero force,
# no SW fallback) when the on-chip SecLoc wrapper is active, plus target_equilibrium.
STATE_MESSAGE_LEN = 40
# Firmware uses a 200-byte reply buffer: 3-byte header + 2 bytes/sample + CRC.
MAX_RAW_ANGLE_SAMPLES_PER_PACKET = 98
SERIAL_IO_ERRORS = (OSError, serial.SerialException, termios.error)


class UartHealth:
    """UART link health for the cartpole FTDI/STATE stream.

    wait (min/p1/p50/p99/max) is the wall time of one read_state() call:
    flush RX, then block until a STATE frame is parsed (or the read fails).
    It is not set_motor→STATE, not a command RTT, and not the full control
    period.

    In a listen-only loop the wait is about one chip period T. After compute
    and set_motor, read_state starts later in the period, so the wait is the
    remainder until the next chip edge. min/p1 is that leftover slack: how
    much later you could have started the wait and still caught that tick.
    Near 0 means the next bit of extra compute would miss and flush. Faster
    compute only makes the wait longer (toward T); it does not threaten
    catching. p99/max is the other tail (early start or a stall).

    skipped: that wait was longer than SKIPPED_WAIT_MULTIPLIER * T. That is a
    missed chip tick (or a USB stall), not “slightly over T”. A 10.3 ms wait
    on a 10 ms tick does not accumulate into a dropped packet every 33 calls;
    each read_state flushes and phase-locks to the next edge.

    lost: no STATE (timeout / port gone). Chip firmware_latency in STATE is
    the motor-command-vs-tick figure; these host waits are not that.
    """

    SKIPPED_WAIT_MULTIPLIER = 1.5

    def __init__(self):
        self.incident_count = 0
        self.last_start = None
        self.last_end = None
        self.last_duration = 0.0
        self.total_downtime = 0.0
        self.disconnected = False
        self.last_error_type = None
        self.last_error_errno = None
        self.last_error_message = None
        self.reconnect_successes = 0
        self.reconnect_failures = 0
        self._recovered_since_check = False
        self.crc_failed = 0
        self.missed_cmd = 0
        self.wrong_len = 0
        self.framing_bursts = 0
        self.last_framing_kind = None
        self._last_framing_at = None
        self.poll_period_s = 0.01
        self.read_attempts = 0
        self.good_reads = 0
        self.lost_count = 0
        self.skipped_count = 0
        self.resync_reads = 0
        self.min_wait_s = None
        self.max_wait_s = 0.0
        self._first_read_at = None
        self._last_read_at = None
        self._wait_ring = []
        self._wait_ring_cap = 512

    def record_error(self, error):
        if error is None:
            return
        self.last_error_type = type(error).__name__
        self.last_error_errno = getattr(error, "errno", None)
        if isinstance(error, OSError) and error.strerror:
            self.last_error_message = error.strerror
        else:
            self.last_error_message = str(error) or self.last_error_type

    def begin_incident(self, error=None):
        if error is not None:
            self.record_error(error)
        if self.disconnected:
            return False
        self.disconnected = True
        self.incident_count += 1
        self.last_start = time.monotonic()
        self.last_end = None
        self._recovered_since_check = False
        return True

    def end_incident(self):
        if not self.disconnected:
            return
        now = time.monotonic()
        self.last_end = now
        if self.last_start is not None:
            self.last_duration = now - self.last_start
            self.total_downtime += self.last_duration
        self.disconnected = False
        self.reconnect_successes += 1
        self._recovered_since_check = True

    def consume_recovered(self):
        recovered = self._recovered_since_check
        self._recovered_since_check = False
        return recovered

    def record_framing(self, kind):
        """Count a parse reject. kind is crc, missed_cmd, or wrong_len."""
        now = time.monotonic()
        if self._last_framing_at is None or (now - self._last_framing_at) > 0.5:
            self.framing_bursts += 1
        self._last_framing_at = now
        self.last_framing_kind = kind
        if kind == "crc":
            self.crc_failed += 1
        elif kind == "missed_cmd":
            self.missed_cmd += 1
        elif kind == "wrong_len":
            self.wrong_len += 1

    def framing_count(self):
        return self.crc_failed + self.missed_cmd + self.wrong_len

    def chip_period_s(self):
        return self.poll_period_s if self.poll_period_s > 0 else 0.01

    def skipped_wait_s(self):
        return self.SKIPPED_WAIT_MULTIPLIER * self.chip_period_s()

    def record_read(self, wait_s, lost=False, resync=False, now=None):
        """Record one read_state() wait (flush + receive), not a send→recv RTT."""
        if now is None:
            now = time.monotonic()
        if self._first_read_at is None:
            self._first_read_at = now
        self._last_read_at = now
        self.read_attempts += 1
        wait_s = max(0.0, float(wait_s))
        self.min_wait_s = wait_s if self.min_wait_s is None else min(self.min_wait_s, wait_s)
        self.max_wait_s = max(self.max_wait_s, wait_s)
        if wait_s > self.skipped_wait_s():
            self.skipped_count += 1
        if lost:
            self.lost_count += 1
        else:
            self.good_reads += 1
        if resync:
            self.resync_reads += 1
        ring = self._wait_ring
        ring.append(wait_s)
        if len(ring) > self._wait_ring_cap:
            del ring[0]

    def _percentile_ms(self, q):
        if not self._wait_ring:
            return 0.0
        ordered = sorted(self._wait_ring)
        index = min(len(ordered) - 1, int(round((q / 100.0) * (len(ordered) - 1))))
        return 1000.0 * ordered[index]

    def good_hz(self):
        if self._first_read_at is None or self._last_read_at is None:
            return 0.0
        elapsed = self._last_read_at - self._first_read_at
        if elapsed <= 0 or self.good_reads <= 1:
            return 0.0
        return (self.good_reads - 1) / elapsed

    def status_line(self, now=None):
        parts = []
        if self.disconnected:
            started = self.last_start
            elapsed = 0.0 if started is None else (now if now is not None else time.monotonic()) - started
            last = self.last_error_message or "unknown"
            parts.append(
                f"DOWN {elapsed:.1f}s (drop {self.incident_count}) last={last}"
            )
        elif self.incident_count:
            last = self.last_error_message or "unknown"
            parts.append(
                f"OK after {self.incident_count} drops "
                f"(last {self.last_duration:.1f}s, total {self.total_downtime:.1f}s) "
                f"last={last}"
            )
        if self.framing_count():
            parts.append(
                f"framing crc={self.crc_failed} miss={self.missed_cmd} "
                f"bad_len={self.wrong_len} bursts={self.framing_bursts}"
            )
        if self.read_attempts:
            skipped_ms = 1000.0 * self.skipped_wait_s()
            min_ms = 0.0 if self.min_wait_s is None else 1000.0 * self.min_wait_s
            parts.append(
                f"io {self.good_hz():.1f}Hz lost={self.lost_count} "
                f"skipped={self.skipped_count}(>{skipped_ms:.0f}ms) "
                f"wait min={min_ms:.0f}ms p1={self._percentile_ms(1):.0f}ms "
                f"p50={self._percentile_ms(50):.0f}ms "
                f"p99={self._percentile_ms(99):.0f}ms "
                f"max={1000.0 * self.max_wait_s:.0f}ms "
                f"resync={self.resync_reads}"
            )
        if not parts:
            return None
        return "UART: " + " | ".join(parts)


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
        self.uart = UartHealth()
        # Setup still fails loudly; the run loop opts into surviving I/O errors.
        self._survive_io_errors = False

    def open(self, port, baud):
        self.port = port
        self.baud = baud
        self.device = serial.Serial(port, baudrate=baud, timeout=None)
        self.device.reset_input_buffer()

    def close(self):
        if not self.device:
            return
        try:
            self.pc_control_mode(False)
            self.control_mode(False)
            self.set_motor(0)
            if not self.uart.disconnected:
                time.sleep(2)
        except SERIAL_IO_ERRORS:
            pass
        try:
            self.device.close()
        except SERIAL_IO_ERRORS:
            pass
        self.device = None

    def clear_read_buffer(self):
        # Drop already-queued bytes, including a STATE that arrived during
        # compute. The next read then waits for the following chip edge
        # (phase lock). If a packet was already in the buffer, that sample
        # is skipped; the wait itself does not accumulate slip.
        self.device.reset_input_buffer()
        self.prevPktNum = 1000

    def enable_runtime_io_tolerance(self):
        self._survive_io_errors = True

    def note_io_error(self, error):
        if self.uart.begin_incident(error):
            detail = self.uart.last_error_message or "serial error"
            print(f'\nUART lost ({detail}); reconnecting.', flush=True)

    def _mark_reconnected(self):
        if not self.uart.disconnected:
            return
        self.uart.end_incident()
        print(
            f'\nUART restored after {self.uart.last_duration:.1f}s '
            f'(drop {self.uart.incident_count}).',
            flush=True,
        )

    def _write_message(self, msg):
        if not self.device:
            err = serial.SerialException("UART disconnected")
            self.note_io_error(err)
            if not self._survive_io_errors:
                raise err
            return False
        try:
            self.device.write(bytearray(msg))
            return True
        except SERIAL_IO_ERRORS as e:
            self.note_io_error(e)
            if not self._survive_io_errors:
                raise
            return False

    def _send_stream_output_request(self, en):
        msg = [SERIAL_SOF, CMD_STREAM_ON, 5, en]
        msg.append(self._crc(msg))
        self._write_message(msg)

    def _reconnect(self, timeout=None, restart_stream=False):
        if not self.uart.disconnected:
            self.note_io_error(serial.SerialException("Serial I/O error"))
        try:
            if self.device:
                self.device.close()
        except SERIAL_IO_ERRORS:
            pass
        self.device = None

        time.sleep(1)
        last_error = None
        opened_port = None
        for port in self._candidate_reconnect_ports():
            try:
                self.device = serial.Serial(port, baudrate=self.baud, timeout=timeout)
                opened_port = port
                last_error = None
                break
            except SERIAL_IO_ERRORS as e:
                last_error = e
                self.device = None

        if self.device is None:
            self.uart.reconnect_failures += 1
            if last_error is not None:
                self.note_io_error(last_error)
            return False

        if opened_port != self.port:
            print(
                f'UART reconnected on {opened_port} (was {self.port}).',
                flush=True,
            )
            self.port = opened_port

        self.msg = []
        self.start = False
        self.prevPktNum = 1000

        if restart_stream:
            self._send_stream_output_request(True)
            try:
                self.clear_read_buffer()
            except SERIAL_IO_ERRORS as e:
                # If the newly opened port cannot be flushed either, let the
                # following read timeout/error drive the next reconnect attempt.
                self.note_io_error(e)
                self.msg = []
                self.prevPktNum = 1000

        return True

    def _candidate_reconnect_ports(self):
        """Last path first, then Digilent FTDI UART if it re-enumerated."""
        ports = []
        if getattr(self, "port", None):
            ports.append(self.port)
        discovered = prefer_zynq_uart_port(None)
        if discovered and discovered not in ports:
            ports.append(discovered)
        return ports

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
                           timesteps_for_derivative=1, force_angle_hanging=False):
        """timesteps_for_derivative: on-chip derivative window in polling periods
        (firmware clamps to 1..20 and restarts its history buffers).

        force_angle_hanging: apply hanging even if the chip locked it (BTN0 / QSPI).
        Default False uses the 16-byte packet; True sends a 17th force byte.
        """
        payload = struct.pack(
            '=H?fH?H',
            controlLoopPeriodMs, controlSync, angle_hanging, avgLen,
            correct_motor_dynamics, timesteps_for_derivative,
        )
        if force_angle_hanging:
            payload += struct.pack('=?', True)
            pkt_len = 17
        else:
            pkt_len = 16
        msg = [SERIAL_SOF, CMD_SET_CONTROL_CONFIG, pkt_len]
        msg += list(payload)
        msg.append(self._crc(msg))
        self._write_message(msg)

    def get_config_control(self):
        msg = [SERIAL_SOF, CMD_GET_CONTROL_CONFIG, 4]
        msg.append(self._crc(msg))
        self._write_message(msg)
        reply = self._receive_reply(CMD_GET_CONTROL_CONFIG, 17)
        (controlLoopPeriodMs, controlSync, angle_hanging, avgLen, correct_motor_dynamics,
         timesteps_for_derivative, hanging_on_chip) = struct.unpack('=H?fH?H?', bytes(reply[3:16]))
        return (controlLoopPeriodMs, controlSync, angle_hanging, avgLen, correct_motor_dynamics,
                timesteps_for_derivative, hanging_on_chip)

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

    def set_secloc_config(self, log_base, ref_period_ticks, dead_ang, dead_pos):
        """Overwrite the on-chip SecLoc gate parameters (see secloc_defaults.h).

        ref_period_ticks is an integer number of control loop iterations
        (POLLING_PERIOD_MS each) between accepted gate updates."""
        msg = [SERIAL_SOF, CMD_SET_SECLOC_CONFIG, 20]
        msg += list(struct.pack('=fi2f', log_base, int(ref_period_ticks), dead_ang, dead_pos))
        msg.append(self._crc(msg))
        self._write_message(msg)

    def get_secloc_info(self):
        """Query the on-chip SecLoc execution backend diagnostics.

        Returns a dict with:
          backend            - 'SW', 'PL' or 'PL-shadow'; the requested backend,
                               never silently degraded: a PL backend without the
                               FPGA SecLoc frontend faults every step (zero
                               force) instead of falling back to SW
          pl_available       - True when the PL frontend answered the boot probe
          shadow_mismatches  - SW/PL gate decision disagreements in shadow mode
                               (expected 0)
          pl_update_count    - NN evaluations in the PL since the last gate reset
          pl_nn_wait_cycles  - PL clock cycles the frontend waited for the
                               network on the most recent computed step
          pl_faults          - steps where a PL backend was selected but the PL
                               block was absent or the transaction failed (each
                               output zero force; no SW computation was
                               substituted; expected 0)
        """
        msg = [SERIAL_SOF, CMD_GET_SECLOC_INFO, 4]
        msg.append(self._crc(msg))
        self._write_message(msg)
        reply = self._receive_reply(CMD_GET_SECLOC_INFO, 22, timeout=2.0, reconnect_at_timeout=False)
        (backend, pl_available, shadow_mismatches,
         pl_update_count, pl_nn_wait_cycles, pl_faults) = struct.unpack('=BB4I', bytes(reply[3:21]))
        return {
            'backend': SECLOC_BACKEND_NAMES.get(backend, f'unknown({backend})'),
            'pl_available': bool(pl_available),
            'shadow_mismatches': shadow_mismatches,
            'pl_update_count': pl_update_count,
            'pl_nn_wait_cycles': pl_nn_wait_cycles,
            'pl_faults': pl_faults,
        }

    def collect_raw_angle(self, lenght=100, interval_us=100):
        if lenght < 0:
            raise ValueError("lenght must be non-negative")

        samples = []
        remaining = lenght
        while remaining:
            chunk = min(remaining, MAX_RAW_ANGLE_SAMPLES_PER_PACKET)
            msg = [SERIAL_SOF, CMD_COLLECT_RAW_ANGLE, 8,
                   chunk % 256, chunk // 256,
                   interval_us % 256, interval_us // 256]
            msg.append(self._crc(msg))
            self._write_message(msg)
            reply = self._receive_reply(
                CMD_COLLECT_RAW_ANGLE, 4 + 2 * chunk, crc=False, timeout=100)
            samples.extend(struct.unpack(
                str(chunk) + 'H', bytes(reply[3:3 + 2 * chunk])))
            remaining -= chunk
        return tuple(samples)

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
        """Flush RX, then block until the next STATE (or fail).

        The wait recorded on UartHealth is this call only. set_motor happens
        after io_step compute; it is not part of the wait. Chip
        firmware_latency is the motor-vs-tick number.
        """
        self.enable_runtime_io_tolerance()
        framing0 = self.uart.framing_count()
        t0 = time.monotonic()
        result = self._read_state_once()
        self.uart.record_read(
            time.monotonic() - t0,
            lost=result is None,
            resync=self.uart.framing_count() > framing0,
        )
        return result

    def _read_state_once(self):
        if self.device is None:
            if not self._reconnect(timeout=READ_STATE_TIMEOUT, restart_stream=True):
                return None
        elif not self.calibration_in_progress:
            try:
                self.clear_read_buffer()
            except SERIAL_IO_ERRORS as e:
                self.note_io_error(e)
                if not self._reconnect(timeout=READ_STATE_TIMEOUT, restart_stream=True):
                    return None
        message_length = STATE_MESSAGE_LEN
        timeout = CALIBRATE_TIMEOUT if self.calibration_in_progress else READ_STATE_TIMEOUT
        reply = self._receive_reply(
            CMD_STATE,
            message_length,
            timeout,
            reconnect_at_timeout=not self.calibration_in_progress,
        )
        if reply is None:
            return None

        if self.uart.disconnected:
            self._mark_reconnected()
            self.set_motor(0)
            if self.uart.disconnected:
                return None

        (angle, angleD, position, target_position, command, invalid_steps, time_difference,
         time_current_measurement_chip, latency, latency_violation,
         secloc_flags, target_equilibrium) = struct.unpack(
            '=hfhfhBIQ2HBf', bytes(reply[3:message_length - 1]))

        return (angle, angleD, position, target_position, command, invalid_steps,
                time_difference / 1e6, time_current_measurement_chip / 1e6,
                latency / 1e5, latency_violation,
                secloc_flags & 1, (secloc_flags >> 1) & 1, (secloc_flags >> 2) & 1,
                (secloc_flags >> 3) & 1, target_equilibrium)

    def _receive_reply(self, cmd, cmdLen, timeout=None, crc=True, reconnect_at_timeout=True):
        survive = reconnect_at_timeout and cmd == CMD_STATE
        reconnects = 0
        self.start = False

        while True:
            if self.device is None:
                if reconnect_at_timeout and reconnects == 0:
                    self.note_io_error(serial.SerialException("UART disconnected"))
                    if self._reconnect(timeout=timeout, restart_stream=True):
                        reconnects += 1
                        continue
                if survive:
                    return None
                raise serial.SerialException("UART disconnected")

            self.device.timeout = timeout
            try:
                c = self.device.read()
            except SERIAL_IO_ERRORS as e:
                self.note_io_error(e)
                if reconnect_at_timeout and reconnects == 0:
                    if self._reconnect(timeout=timeout, restart_stream=True):
                        reconnects += 1
                        continue
                if survive:
                    return None
                raise

            # Timeout: reopen device, start stream, reset msg and try again
            if len(c) == 0:
                if reconnect_at_timeout:
                    self.note_io_error(serial.SerialException("Timeout"))
                    if survive:
                        if reconnects == 0:
                            if self._reconnect(timeout=timeout, restart_stream=True):
                                reconnects += 1
                                continue
                        return None
                    self._reconnect(timeout=timeout, restart_stream=True)
                    reconnects += 1
                    continue
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

                    self.uart.record_framing("crc")
                    del self.msg[0]
                    continue

                if cmd != CMD_STATE and self.msg[1] == CMD_STATE:
                    del self.msg[:packet_len]
                    continue

                # Check command
                if self.msg[1] != cmd:
                    self.uart.record_framing("missed_cmd")
                    del self.msg[0]
                    continue

                # Check message packet length
                if self.msg[2] != cmdLen and cmdLen < 256:
                    self.uart.record_framing("wrong_len")
                    del self.msg[0]
                    continue

                # Verify integrity of message
                if crc and self.msg[cmdLen - 1] != self._crc(self.msg[:cmdLen - 1]):
                    self.uart.record_framing("crc")
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
