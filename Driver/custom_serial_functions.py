"""
TODO: It would be great to have at some point functions related to serial communication extracted to a separate place.
    Having it done, it would be not to much work to run physical cartpole directly from CartPole Simulator GUI
"""
import glob
import sys
import serial  # conda install pyserial

def serial_ports():  # from https://stackoverflow.com/questions/12090503/listing-available-com-ports-with-python
    """ Lists serial port names

        :raises EnvironmentError:
            On unsupported or unknown platforms
        :returns:
            A list of the serial ports available on the system
    """
    if sys.platform.startswith('win'):
        ports = ['COM%s' % (i + 1) for i in range(256)]
    elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
        # this excludes your current terminal "/dev/tty"
        # if cannot open, check permissions
        ports = glob.glob('/dev/ttyUSB[0-9]*')
    elif sys.platform.startswith('darwin'):
        ports = glob.glob('/dev/tty.*')
    else:
        raise EnvironmentError('Unsupported platform')

    result = []
    for port in ports:
        try:
            s = serial.Serial(port)
            s.close()
            result.append(port)
        except (OSError, serial.SerialException):
            pass
    return result
