import serial

SERIAL_PORT = 'COM4'
SERIAL_BAUD = 230400
device = serial.Serial(SERIAL_PORT, baudrate=SERIAL_BAUD, timeout=None)
device.reset_input_buffer()
device.close()

