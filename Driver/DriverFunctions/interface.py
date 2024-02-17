import serial
import struct
import time

PING_TIMEOUT            = 1.0       # Seconds
CALIBRATE_TIMEOUT       = 10.0      # Seconds
READ_STATE_TIMEOUT      = 1.0      # Seconds
SERIAL_SOF              = 0xAA
CMD_PING                = 0xC0
CMD_STREAM_ON           = 0xC1
CMD_CALIBRATE           = 0xC2
CMD_CONTROL_MODE        = 0xC3
CMD_SET_PID_CONFIG      = 0xC4
CMD_GET_PID_CONFIG      = 0xC5
CMD_SET_CONTROL_CONFIG  = 0xC6
CMD_GET_CONTROL_CONFIG  = 0xC7
CMD_SET_MOTOR           = 0xC8
CMD_SET_TARGET_POSITION = 0xC9
CMD_COLLECT_RAW_ANGLE   = 0xCA
CMD_STATE               = 0xCC

def get_serial_port(chip_type="STM", serial_port_number=None):

    """
    Finds the cartpole serial port, or throws exception if not present
    :param chip_type: "ZYNQ" or "STM" depending on which one you use
    :param serial_port_number: Only used if serial port not found using chip type, can be left None, for normal operation
    :returns:  the string name of the COM port
    """

    from serial.tools import list_ports
    ports = list(serial.tools.list_ports.comports())
    serial_ports_names = []
    print('\nAvailable serial ports:')
    for index, port in enumerate(ports):
        serial_ports_names.append(port.device)
        print(f'{index}: port={port.device}; description={port.description}')
    print()

    if chip_type == "STM":
        expected_description = 'USB Serial'
    elif chip_type == "ZYNQ":
        expected_description = 'Digilent Adept USB Device - Digilent Adept USB Device'
    else:
        raise ValueError(f'Unknown chip type: {chip_type}')

    SERIAL_PORT = None
    for port in ports:
        if port.description == expected_description:
            SERIAL_PORT = port.device
            break
    if SERIAL_PORT is None:
        message = f"Searching serial port by its expected description - {expected_description} - not successful."
        if serial_port_number is not None:
            print(message)
        else:
            raise Exception(message)

    if SERIAL_PORT is None and serial_port_number is not None:
        print(f"Trying to connect to a serial port with requested number ({serial_port_number})\n")
        SERIAL_PORT = serial_ports_names[serial_port_number]


    return SERIAL_PORT

class Interface:
    def __init__(self):
        self.device         = None
        self.msg            = []
        self.prevPktNum     = 1000
        self.start = None
        self.end = None

        self.encoderDirection = None

    def open(self, port, baud):
        self.port = port
        self.baud = baud
        self.device = serial.Serial(port, baudrate=baud, timeout=None)
        self.device.reset_input_buffer()

    def close(self):
        if self.device:
            self.control_mode(False)
            self.set_motor(0)
            time.sleep(2)
            self.device.close()
            self.device = None

    def clear_read_buffer(self):
        self.device.reset_input_buffer()
        self.prevPktNum = 1000

    def ping(self):
        msg = [SERIAL_SOF, CMD_PING, 4]
        msg.append(self._crc(msg))
        self.device.write(bytearray(msg))
        self.prevPktNum = 1000
        return self._receive_reply(CMD_PING, 4, PING_TIMEOUT) == msg

    def stream_output(self, en):
        msg = [SERIAL_SOF, CMD_STREAM_ON, 5, en]
        msg.append(self._crc(msg))
        self.device.write(bytearray(msg))
        self.clear_read_buffer()

    def calibrate(self):
        msg = [SERIAL_SOF, CMD_CALIBRATE, 4]
        msg.append(self._crc(msg))
        self.device.write(bytearray(msg))

        self.clear_read_buffer()

        reply = self._receive_reply(CMD_CALIBRATE, 5, CALIBRATE_TIMEOUT)
        self.encoderDirection = struct.unpack('b', bytes(reply[3:4]))[0]

        return True

    def control_mode(self, en):
        msg = [SERIAL_SOF, CMD_CONTROL_MODE, 5, 1 if en else 0]
        msg.append(self._crc(msg))
        self.device.write(bytearray(msg))

    def set_config_PID(self, setPoint, smoothing, position_KP, position_KI, position_KD, angle_KP, angle_KI, angle_KD):
        msg = [SERIAL_SOF, CMD_SET_PID_CONFIG, 28]

        msg += list(struct.pack('f', position_KP))
        msg += list(struct.pack('f', position_KI))
        msg += list(struct.pack('f', position_KD))
        
        msg += list(struct.pack('f', angle_KP))
        msg += list(struct.pack('f', angle_KI))
        msg += list(struct.pack('f', angle_KD))

        msg.append(self._crc(msg))
        self.device.write(bytearray(msg))

    def get_config_PID(self):
        msg = [SERIAL_SOF, CMD_GET_PID_CONFIG, 4]
        msg.append(self._crc(msg))
        self.device.write(bytearray(msg))
        reply = self._receive_reply(CMD_GET_PID_CONFIG, 28)
        (setPoint, smoothing, position_KP, position_KI, position_KD, angle_KP, angle_KI, angle_KD) = struct.unpack('h7f', bytes(reply[3:27]))
        return setPoint, smoothing, position_KP, position_KI, position_KD, angle_KP, angle_KI, angle_KD

    def set_config_control(self, controlLoopPeriodMs, controlSync, angle_deviation, avgLen, correct_motor_dynamics):
        msg = [SERIAL_SOF, CMD_SET_CONTROL_CONFIG, 14]
        msg += list(struct.pack('H', controlLoopPeriodMs))
        msg += list(struct.pack('?', controlSync))
        msg += list(struct.pack('f', angle_deviation))
        msg += list(struct.pack('H', avgLen))
        msg += list(struct.pack('?', correct_motor_dynamics))
        msg.append(self._crc(msg))
        self.device.write(bytearray(msg))

    def get_config_control(self):
        msg = [SERIAL_SOF, CMD_GET_CONTROL_CONFIG, 4]
        msg.append(self._crc(msg))
        self.device.write(bytearray(msg))
        reply = self._receive_reply(CMD_GET_CONTROL_CONFIG, 14)
        (controlLoopPeriodMs, controlSync, angle_deviation, avgLen, correct_motor_dynamics) = struct.unpack('H?fH', bytes(reply[3:12]))
        return controlLoopPeriodMs, controlSync, angle_deviation, avgLen, correct_motor_dynamics

    def set_motor(self, speed):
        msg  = [SERIAL_SOF, CMD_SET_MOTOR, 8]
        msg += list(struct.pack('i', speed))
        msg.append(self._crc(msg))
        self.device.write(bytearray(msg))

    def set_target_position(self, target_position):
        msg  = [SERIAL_SOF, CMD_SET_TARGET_POSITION, 8]
        msg += list(struct.pack('f', target_position))
        msg.append(self._crc(msg))
        self.device.write(bytearray(msg))

    def collect_raw_angle(self, lenght=100, interval_us=100):
        msg = [SERIAL_SOF, CMD_COLLECT_RAW_ANGLE, 8,  lenght % 256, lenght // 256, interval_us % 256, interval_us // 256]
        msg.append(self._crc(msg))
        self.device.write(bytearray(msg))
        reply = self._receive_reply(CMD_COLLECT_RAW_ANGLE, 4 + 2*lenght, crc=False, timeout=100)
        return struct.unpack(str(lenght)+'H', bytes(reply[3:3+2*lenght]))

    def read_state(self):
        self.clear_read_buffer()
        message_length = 27
        reply = self._receive_reply(CMD_STATE, message_length, READ_STATE_TIMEOUT)

        (angle, position, target_position, command, invalid_steps, time_difference, sent, latency, latency_violation) = struct.unpack('=2hfhB2I2H', bytes(reply[3:message_length-1]))

        return angle, position, target_position, command, invalid_steps, time_difference/1e6, sent/1e6, latency/1e5, latency_violation

    def _receive_reply(self, cmd, cmdLen, timeout=None, crc=True):
        self.device.timeout = timeout
        self.start = False

        while True:
            c = self.device.read()
            # Timeout: reopen device, start stream, reset msg and try again
            if len(c) == 0:
                print('\nReconnecting.')
                self.device.close()
                self.device = serial.Serial(self.port, baudrate=self.baud, timeout=timeout)
                self.clear_read_buffer()
                time.sleep(1)
                self.stream_output(True)
                self.msg = []
                self.start = False
            else:
                self.msg.append(ord(c))
                if self.start == False:
                    self.start = time.time()

            while len(self.msg) >= cmdLen:
                # print('I am looping! Hurra!')
                # Message must start with SOF character
                if self.msg[0] != SERIAL_SOF:
                    #print('\nMissed SERIAL_SOF')
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
                if crc and self.msg[cmdLen-1] != self._crc(self.msg[:cmdLen-1]):
                    print('\nCRC Failed.')
                    del self.msg[0]
                    continue

                self.device.timeout = None
                reply = self.msg[:cmdLen]
                del self.msg[:cmdLen]
                return reply

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

import subprocess
def set_ftdi_latency_timer(SERIAL_PORT):
    serial_port = SERIAL_PORT.split('/')[-1]
    print('\nSetting FTDI latency timer')
    ftdi_timer_latency_requested_value = 1
    command_ftdi_timer_latency_set = f"sh -c 'echo {ftdi_timer_latency_requested_value} > /sys/bus/usb-serial/devices/{serial_port}/latency_timer'"
    command_ftdi_timer_latency_check = f'cat /sys/bus/usb-serial/devices/{serial_port}/latency_timer'
    try:
        subprocess.run(command_ftdi_timer_latency_set, shell=True, check=True, capture_output=True, text=True)
    except subprocess.CalledProcessError as e:
        print(e.stderr)
        if "Permission denied" in e.stderr:
            print("Trying with sudo...")
            command_ftdi_timer_latency_set = "sudo " + command_ftdi_timer_latency_set
            try:
                subprocess.run(command_ftdi_timer_latency_set, shell=True, check=True, capture_output=True, text=True)
            except subprocess.CalledProcessError as e:
                print(e.stderr)

    ftdi_latency_timer_value = subprocess.run(command_ftdi_timer_latency_check, shell=True, capture_output=True, text=True).stdout.rstrip()
    print(f'FTDI latency timer value (tested only for FTDI with Zybo and with Linux on PC side): {ftdi_latency_timer_value} ms  \n')
