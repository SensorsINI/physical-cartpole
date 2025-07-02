import serial
import struct
import time
import pandas as pd

# at the very top, after your other imports:
import platform

# On macOS we want to force PyUSB to use Homebrew’s libusb so that
# we can control the FTDI latency timer. Then we pull in pyftdi’s
# serial extension which honors the “latency” query parameter.
if platform.system() == 'Darwin':
    import usb.backend.libusb1
    import usb.core
    from pyftdi.serialext import serial_for_url

    # Point PyUSB at the Homebrew‐installed libusb rather than Apple's
    backend = usb.backend.libusb1.get_backend(
        find_library=lambda x: "/opt/homebrew/lib/libusb-1.0.dylib"
    )
    usb.core.backend = backend

PING_TIMEOUT            = 1.0       # Seconds
CALIBRATE_TIMEOUT       = 10.0      # Seconds
HARDWARE_EXPERIMENT_TIMEOUT = 30.0      # Seconds
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
CMD_SET_TARGET_EQUILIBRIUM = 0xCD
CMD_RUN_HARDWARE_EXPERIMENT = 0xCE
CMD_TRANSFER_BUFFERS    = 0xD1

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
        expected_descriptions = ['USB Serial']
    elif chip_type == "ZYNQ":
        expected_descriptions = ['Digilent Adept USB Device - Digilent Adept USB Device', 'Digilent Adept USB Device']
    else:
        raise ValueError(f'Unknown chip type: {chip_type}')

    possible_ports = []
    for port in ports:
        if port.description in expected_descriptions:
            possible_ports.append(port.device)

    SERIAL_PORT = None
    if not possible_ports:
        message = f"Searching serial port by its expected descriptions - {expected_descriptions} - not successful."
        if serial_port_number is not None:
            print(message)
        else:
            raise Exception(message)
    else:
        if serial_port_number < len(possible_ports):
            SERIAL_PORT = possible_ports[serial_port_number]
        else:
            print(f"Requested serial port number {serial_port_number} is out of range. Available ports: {len(possible_ports)}")
            print(f"Using the first available port: {possible_ports[0]}")
            SERIAL_PORT = possible_ports[0]

    if SERIAL_PORT is None and serial_port_number is not None:
        if len(serial_ports_names)==0:
            print(f'No serial ports')
        else:
            print(f"Setting serial port with requested number ({serial_port_number})\n")
            SERIAL_PORT = serial_ports_names[serial_port_number]

    if platform.system() == 'Darwin' and SERIAL_PORT:
        # On macOS, build a PyFtdi URL including the serial number so
        # create_from_url() can locate the exact device and apply latency.
        port_info = next((p for p in ports if p.device == SERIAL_PORT), None)
        if port_info is None:
            raise Exception(f"Couldn't retrieve port_info for {SERIAL_PORT}")
        from pyftdi.ftdi import Ftdi

        # —— DEBUG: list detected FTDI devices — useful if multiple VID/PID exist
        print("Detected FTDI devices (PyFtdi.list_devices()):")
        for desc, iface in Ftdi.list_devices():
            print(f"  VID={desc.vid:04x}, PID={desc.pid:04x}, SN={desc.sn!r}, IFACE={iface}")

        # —— find first device matching VID/PID ——
        found = next(
            ((desc, iface) for desc, iface in Ftdi.list_devices()
             if desc.vid == port_info.vid and desc.pid == port_info.pid),
            None
        )

        # Determine serial number to use: prefer pyserial-provided, else descriptor
        port_serial = getattr(port_info, 'serial_number', None) or (found[0].sn if found else None)
        if port_serial is None:
            raise Exception(f"Could not determine serial number for {SERIAL_PORT}")

        # Determine chip token: '2232h' for FT2232H (PID 0x6010), else '232r'
        pid_value = found[0].pid if found else port_info.pid
        chip_token = '2232h' if pid_value == 0x6010 else '232r'

        # Interface index: from found tuple or default to 1
        interface = found[1] if found else 1

        # Build and return the PyFtdi URL with the latency parameter
        return f"ftdi://ftdi:{chip_token}/{interface}?latency=1"


    return SERIAL_PORT

class Interface:
    def __init__(self):
        self.device         = None
        self.msg            = []
        self.prevPktNum     = 1000
        self.start = None
        self.end = None

        self.encoderDirection = None

        self.hardware_experiment_length = 0

    def open(self, port, baud):
        self.port = port
        self.baud = baud

        if isinstance(port, str) and port.startswith("ftdi://"):
            try:
                self.device = serial_for_url(port, baudrate=baud, timeout=None)
            except serial.SerialException as e:
                # Hard to debug otherwise, so let us know what went wrong…
                print(f"⚠️  PyFtdi open failed ({e}); falling back to pyserial on {self.port}")
                self.device = serial.Serial(self.port, baudrate=self.baud, timeout=None)
        else:
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

    def run_hardware_experiment(self):
        msg = [SERIAL_SOF, CMD_RUN_HARDWARE_EXPERIMENT, 4]
        msg.append(self._crc(msg))
        self.device.write(bytearray(msg))

        self.clear_read_buffer()

        reply = self._receive_reply(CMD_RUN_HARDWARE_EXPERIMENT, 6, HARDWARE_EXPERIMENT_TIMEOUT,
                                    reconnect_at_timeout=False)
        self.hardware_experiment_length = struct.unpack('H', bytes(reply[3:5]))[0]
        print(f'Hardware experiment finished with length {self.hardware_experiment_length}')

        msg = [SERIAL_SOF, CMD_TRANSFER_BUFFERS, 4]
        msg.append(self._crc(msg))
        self.device.write(bytearray(msg))

        self.clear_read_buffer()
        variables_bytes = []
        message_length = 4 * self.hardware_experiment_length + 7
        for i in range(7):  # There are seven floats to receive
            c = self._receive_reply(CMD_TRANSFER_BUFFERS, message_length, HARDWARE_EXPERIMENT_TIMEOUT, reconnect_at_timeout=False)
            variables_bytes.append(c)

        message_length = self.hardware_experiment_length + 7  # target equilibrium,
        c = self._receive_reply(CMD_TRANSFER_BUFFERS, message_length, HARDWARE_EXPERIMENT_TIMEOUT, reconnect_at_timeout=False)
        variables_bytes.append(c)

        variables = []
        unpack_string = f'<{self.hardware_experiment_length}f'
        for i in range(len(variables_bytes)-1):
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

    def set_config_control(self, controlLoopPeriodMs, controlSync, angle_hanging, avgLen, correct_motor_dynamics):
        msg = [SERIAL_SOF, CMD_SET_CONTROL_CONFIG, 14]
        msg += list(struct.pack('H', controlLoopPeriodMs))
        msg += list(struct.pack('?', controlSync))
        msg += list(struct.pack('f', angle_hanging))
        msg += list(struct.pack('H', avgLen))
        msg += list(struct.pack('?', correct_motor_dynamics))
        msg.append(self._crc(msg))
        self.device.write(bytearray(msg))

    def get_config_control(self):
        msg = [SERIAL_SOF, CMD_GET_CONTROL_CONFIG, 4]
        msg.append(self._crc(msg))
        self.device.write(bytearray(msg))
        reply = self._receive_reply(CMD_GET_CONTROL_CONFIG, 14)
        (controlLoopPeriodMs, controlSync, angle_hanging, avgLen, correct_motor_dynamics) = struct.unpack('H?fH', bytes(reply[3:12]))
        return controlLoopPeriodMs, controlSync, angle_hanging, avgLen, correct_motor_dynamics

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

    def set_target_equilibrium(self, target_equilibrium):
        msg = [SERIAL_SOF, CMD_SET_TARGET_EQUILIBRIUM, 8]
        msg += list(struct.pack('f', target_equilibrium))
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
        message_length = 31
        reply = self._receive_reply(CMD_STATE, message_length, READ_STATE_TIMEOUT)

        (angle, angleD, position, target_position, command, invalid_steps, time_difference, time_current_measurement_chip, latency, latency_violation) = struct.unpack('=hfhfhB2I2H', bytes(reply[3:message_length-1]))

        return angle, angleD, position, target_position, command, invalid_steps, time_difference/1e6, time_current_measurement_chip/1e6, latency/1e5, latency_violation

    def _receive_reply(self, cmd, cmdLen, timeout=None, crc=True, reconnect_at_timeout=True):
        self.device.timeout = timeout
        self.start = False

        while True:
            c = self.device.read()
            # Timeout: reopen device, start stream, reset msg and try again
            if len(c) == 0:
                if reconnect_at_timeout:
                    print('\n_receive_reply: no response; reconnecting.')
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


SUDO_PASSWORD = None
import subprocess
import getpass
import platform
# This is probably wrong port now.
def set_ftdi_latency_timer_linux(SERIAL_PORT):
    print('\nSetting FTDI latency timer for Linux...')
    requested_value = 1  # in ms

    if platform.system() == 'Linux':
        # check for hardcoded sudo password or prompt the user
        if SUDO_PASSWORD:
            password = SUDO_PASSWORD
        else:
            password = getpass.getpass('Enter sudo password: ')

        serial_port = SERIAL_PORT.split('/')[-1]
        ftdi_timer_latency_requested_value = 1
        command_ftdi_timer_latency_set = f"sh -c 'echo {ftdi_timer_latency_requested_value} > /sys/bus/usb-serial/devices/{serial_port}/latency_timer'"
        command_ftdi_timer_latency_check = f'cat /sys/bus/usb-serial/devices/{serial_port}/latency_timer'
        try:
            subprocess.run(command_ftdi_timer_latency_set, shell=True, check=True, capture_output=True, text=True)
        except subprocess.CalledProcessError as e:
            print(e.stderr)
            if "Permission denied" in e.stderr:
                print("Trying with sudo...")
                command_ftdi_timer_latency_set = f"echo {password} | sudo -S {command_ftdi_timer_latency_set}"
                try:
                    subprocess.run(command_ftdi_timer_latency_set, shell=True, check=True, capture_output=True, text=True)
                except subprocess.CalledProcessError as e:
                    print(e.stderr)

        ftdi_latency_timer_value = subprocess.run(command_ftdi_timer_latency_check, shell=True, capture_output=True, text=True).stdout.rstrip()
        print(f'FTDI latency timer value (tested only for FTDI with Zybo and with Linux on PC side): {ftdi_latency_timer_value} ms  \n')



def set_ftdi_latency_all_mac(latency_ms=1):
    print("\nSetting FTDI latency timer for all FTDI devices on macOS...")
    import usb.backend.libusb1
    import usb.core
    from pyftdi.ftdi import Ftdi

    # Force PyUSB to use Homebrew’s libusb rather than the Apple one
    backend = usb.backend.libusb1.get_backend(
        find_library=lambda x: "/opt/homebrew/lib/libusb-1.0.dylib"
    )
    usb.core.backend = backend

    # list_devices() yields (UsbDeviceDescriptor, interface) tuples,
    # not actual PyUSB Device objects
    dev_list = Ftdi.list_devices()
    if not dev_list:
        print("No FTDI devices found.")
        return

    for usb_desc, iface in dev_list:
        # usb_desc has vid, pid, sn, bus, address, etc.
        # Locate the real PyUSB device by vendor/product/serial
        pyusb_dev = usb.core.find(
            idVendor=usb_desc.vid,
            idProduct=usb_desc.pid,
            serial_number=usb_desc.sn
        )
        if pyusb_dev is None:
            print(f"Could not open {usb_desc}: no matching PyUSB device")
            continue

        # Now we can open it properly by passing the PyUSB Device + interface
        ftdi_dev = Ftdi()
        ftdi_dev.open_from_device(pyusb_dev, iface)
        ftdi_dev.set_latency_timer(latency_ms)  # persists until next power-cycle
        ftdi_dev.close()
        print(f"→ Set latency to {latency_ms} ms on SN={usb_desc.sn}, iface={iface}")
