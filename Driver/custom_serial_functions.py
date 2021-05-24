"""
TODO: It would be great to have at some point functions related to serial communication extracted to a separate place.
    Having it done, it would be not to much work to run physical cartpole directly from CartPole Simulator GUI
"""
import glob
import sys
import serial  # conda install pyserial

from globals import SERIAL_BAUD, SERIAL_PORT

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


def setup_serial_connection(CartPoleInstance, SERIAL_PORT):
    serialPorts = serial_ports()
    print('Available serial ports: ' + str(serialPorts))
    if len(serialPorts) == 0:
        print(
            'no serial ports available, or cannot open it; check linux permissions\n Under linux, sudo chmod a+rw [port] transiently, or add user to dialout or tty group')
        quit()

    if SERIAL_PORT is None:
        if len(serialPorts) > 1:
            print(str(len(serialPorts)) + ' serial ports, taking first one which is ' + str(serialPorts[0]))
        else:
            print('Taking the only available serial port which is ' + str(serialPorts[0]))
        SERIAL_PORT = str(serialPorts[0])

    try:
        CartPoleInstance.open(SERIAL_PORT, SERIAL_BAUD)
    except:
        print('cannot open port ' + str(SERIAL_PORT) + ': available ports are ' + str(serial_ports()))
        quit()
