import serial
import struct
import sys
import time

PING_TIMEOUT            = 1.0       # Seconds
CALIBRATE_TIMEOUT       = 10.0      # Seconds
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
CMD_STATE               = 0xCC

class Pendulum:
    def __init__(self):
        self.device         = None
        self.msg            = []
        self.prevPktNum     = 1000

    def open(self, port, baud):
        self.device         = serial.Serial(port, baudrate=baud, timeout=None)
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
        return self._receive_reply(CMD_CALIBRATE, 4, CALIBRATE_TIMEOUT) == msg

    def control_mode(self, en):
        msg = [SERIAL_SOF, CMD_CONTROL_MODE, 5, en]
        msg.append(self._crc(msg))
        self.device.write(bytearray(msg))
        self.device.flush()

    def set_angle_config(self, setPoint, avgLen, smoothing, KP, KD):
        msg  = [SERIAL_SOF, CMD_SET_ANGLE_CONFIG, 20]
        msg += list(struct.pack('h', setPoint))
        msg += list(struct.pack('H', avgLen))
        msg += list(struct.pack('f', smoothing))
        msg += list(struct.pack('f', KP))
        msg += list(struct.pack('f', KD))
        msg.append(self._crc(msg))
        self.device.write(bytearray(msg))
        self.device.flush()

    def get_angle_config(self):
        msg = [SERIAL_SOF, CMD_GET_ANGLE_CONFIG, 4]
        msg.append(self._crc(msg))
        self.device.write(bytearray(msg))
        self.device.flush()
        reply = self._receive_reply(CMD_GET_ANGLE_CONFIG, 20)
        (setPoint, avgLen, smoothing, KP, KD) = struct.unpack('hHfff', bytes(reply[3:19]))
        return (setPoint, avgLen, smoothing, KP, KD)

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
        return (setPoint, ctrlPeriod_ms, smoothing, KP, KD)

    def set_motor(self, speed):
        msg  = [SERIAL_SOF, CMD_SET_MOTOR, 6, speed & 0xFF, (speed >> 8) & 0xFF]
        msg.append(self._crc(msg))
        self.device.write(bytearray(msg))
        self.device.flush()

    def read_state(self):
        reply = self._receive_reply(CMD_STATE, 11)

        # Verify packet sequence 
        if self.prevPktNum != 1000:
            if ((reply[3] - self.prevPktNum) & 0xFF) > 1:
                print("WARNING -- Skipped packets [prev={0} now={1}]".format(self.prevPktNum, reply[3]))
        self.prevPktNum = reply[3]

        (angle, position, command) = struct.unpack('hhh', bytes(reply[4:10]))
        return (angle, position, command)

    def _receive_reply(self, cmd, cmdLen, timeout=None):
        self.device.timeout = timeout
        while True:
            c = self.device.read()
            if len(c) == 0:
                self.device.timeout = None
                return []

            self.msg.append(ord(c))

            while len(self.msg) >= cmdLen:
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