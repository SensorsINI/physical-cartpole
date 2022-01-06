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
CMD_SET_ANGLE_CONFIG    = 0xC4
CMD_GET_ANGLE_CONFIG    = 0xC5
CMD_SET_POSITION_CONFIG = 0xC6
CMD_GET_POSITION_CONFIG = 0xC7
CMD_SET_MOTOR           = 0xC8
CMD_SET_CONTROL_CONFIG  = 0xC9
CMD_STATE               = 0xCC

class Interface:
    def __init__(self):
        self.device         = None
        self.msg            = []
        self.prevPktNum     = 1000
        self.start = None
        self.end = None
        self.angle_raw_prev = 0
        self.angleD = 0

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
        self.device.flush()
        self.prevPktNum = 1000
        return self._receive_reply(CMD_PING, 4, PING_TIMEOUT) == msg

    def stream_output(self, en):
        msg = [SERIAL_SOF, CMD_STREAM_ON, 5, en]
        msg.append(self._crc(msg))
        self.device.write(bytearray(msg))
        self.device.flush()
        self.clear_read_buffer()

    def calibrate(self):
        msg = [SERIAL_SOF, CMD_CALIBRATE, 4]
        msg.append(self._crc(msg))
        self.device.write(bytearray(msg))
        self.device.flush()
        self.prevPktNum = 1000

        reply = self._receive_reply(CMD_CALIBRATE, 5, CALIBRATE_TIMEOUT)
        self.encoderDirection = struct.unpack('b', bytes(reply[3:4]))[0]

        return True

    def control_mode(self, en):
        msg = [SERIAL_SOF, CMD_CONTROL_MODE, 5, 1 if en else 0]
        msg.append(self._crc(msg))
        self.device.write(bytearray(msg))
        self.device.flush()

    def set_angle_config(self, setPoint, avgLen, smoothing, KP, KI, KD):
        msg  = [SERIAL_SOF, CMD_SET_ANGLE_CONFIG, 24]
        msg += list(struct.pack('h', setPoint))
        msg += list(struct.pack('H', avgLen))
        msg += list(struct.pack('f', smoothing))
        msg += list(struct.pack('f', KP))
        msg += list(struct.pack('f', KI))
        msg += list(struct.pack('f', KD))
        msg.append(self._crc(msg))
        self.device.write(bytearray(msg))
        self.device.flush()

    def get_angle_config(self):
        msg = [SERIAL_SOF, CMD_GET_ANGLE_CONFIG, 4]
        msg.append(self._crc(msg))
        self.device.write(bytearray(msg))
        self.device.flush()
        reply = self._receive_reply(CMD_GET_ANGLE_CONFIG, 29)
        (setPoint, avgLen, smoothing, KP, KI, KD) = struct.unpack('hHffff', bytes(reply[3:20]))
        return setPoint, avgLen, smoothing, KP, KI, KD

    def set_position_config(self, setPoint, ctrlPeriod_ms, smoothing, KP, KD):
        msg = [SERIAL_SOF, CMD_SET_POSITION_CONFIG, 20]
        msg += list(struct.pack('h', setPoint))
        msg += list(struct.pack('H', ctrlPeriod_ms))
        msg += list(struct.pack('f', smoothing))
        msg += list(struct.pack('f', KP))
        msg += list(struct.pack('f', KD))
        msg.append(self._crc(msg))
        self.device.write(bytearray(msg))
        self.device.flush()

    def get_position_config(self):
        msg = [SERIAL_SOF, CMD_GET_POSITION_CONFIG, 4]
        msg.append(self._crc(msg))
        self.device.write(bytearray(msg))
        self.device.flush()
        reply = self._receive_reply(CMD_GET_POSITION_CONFIG, 20)
        (setPoint, ctrlPeriod_ms, smoothing, KP, KD) = struct.unpack('hHfff', bytes(reply[3:19]))
        return setPoint, ctrlPeriod_ms, smoothing, KP, KD

    def set_motor(self, speed):
        msg  = [SERIAL_SOF, CMD_SET_MOTOR, 6, speed & 0xFF, (speed >> 8) & 0xFF]
        msg.append(self._crc(msg))
        self.device.write(bytearray(msg))
        self.device.flush()
        self.end = time.time()

    def set_control_config(self, controlLoopPeriodMs, controlSync, controlLatencyUs):
        msg = [SERIAL_SOF, CMD_SET_CONTROL_CONFIG, 11]
        msg += list(struct.pack('H', controlLoopPeriodMs))
        msg += list(struct.pack('?', controlSync))
        msg += list(struct.pack('i', controlLatencyUs))
        msg.append(self._crc(msg))
        self.device.write(bytearray(msg))
        self.device.flush()

    def wrap_local(self, angle):
        ADC_RANGE = 4096
        if angle >= ADC_RANGE / 2:
            return angle - ADC_RANGE
        elif angle <= -ADC_RANGE / 2:
            return angle + ADC_RANGE
        else:
            return angle

    def read_state(self):
        reply = self._receive_reply(CMD_STATE, 17, READ_STATE_TIMEOUT)

        # Verify packet sequence 
        #if self.prevPktNum != 1000:
        #    if ((reply[3] - self.prevPktNum) & 0xFF) > 1:
        #        print("WARNING -- Skipped packets [prev={0} now={1}]".format(self.prevPktNum, reply[3]))
        #self.prevPktNum = reply[3]

        (angle, position, command, frozen, sent, latency) = struct.unpack('=3hBIH', bytes(reply[3:16]))

        if frozen == 0:
            self.angleD = self.wrap_local(angle - self.angle_raw_prev)
        self.angle_raw_prev = angle

        return angle, self.angleD, position, command, frozen, sent/1e6, latency/1e6

    def _receive_reply(self, cmd, cmdLen, timeout=None):
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
                    del self.msg[0]
                    continue

                # Check command
                if self.msg[1] != cmd:
                    del self.msg[0]
                    continue

                # Check message packet length
                if self.msg[2] != cmdLen:
                    del self.msg[0]
                    continue

                # Verify integrity of message
                if self.msg[cmdLen-1] != self._crc(self.msg[:cmdLen-1]):
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