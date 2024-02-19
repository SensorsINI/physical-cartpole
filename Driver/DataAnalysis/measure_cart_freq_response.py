# measures the cart frequency response by opening the board, setting the control frequency to 1kHz, and then
# sweeping frequency of cart and measuring the cart motor position
import sys,os,time
import atexit
import numpy as np
from matplotlib import pyplot as plt


# assume we start from physical-cartpole root
print('current working directory is '+os.getcwd())
print('sys.path is '+str(sys.path))
print('adding paths to sys.path')
sys.path.insert(0, os.path.abspath(os.path.join("..")))
sys.path.insert(1, os.path.abspath(os.path.join("..", "CartPoleSimulation")))
sys.path.insert(1, os.path.abspath(os.path.join("..", "CartPoleSimulation","Control_Toolkit","others")))
print('now sys.path is '+str(sys.path))

from globals import MOTOR_FULL_SCALE

from Driver.CartPoleSimulation.Control_Toolkit.others.globals_and_utils import get_logger
from Driver.DriverFunctions.interface import *

# log=get_logger(__name__)

# Enumeration for sweep mode
sweep_mode='exponential' # 'linear'
import numpy as np


from serial.tools import list_ports
def findCartpoleSerialPort():
    """
    Finds the cartpole serial port, or None if not present
    :returns:  the string name of the COM port
    """

    ports = list( serial.tools.list_ports.comports() )

    for port in ports:
        print(f'port={port.device} description={port.description}')
        if port.device and (port.description.startswith( "USB Serial") or port.description.startswith("Digilent")):
            print(f'found serial port {port.device} with description {port.description} that matches "USB Serial" or "Digilent"')
            return port.device
    print(f'could not find a serial port for cartpole')
    return None

# Function to generate sinusoidal sweep with linear or exponential frequency change
def sinusoidal_sweep(t:float, f_start:float, f_end:float, t_total:float, mode:str):
    # Calculate t modulo t_total once per call
    mod_t = np.mod(t, t_total)

    # # Update the flag based on the position of mod_t within the first 5% of t_total
    # if (mod_t >= 0 and mod_t <= (0.05 * t_total)):
    #     flag = 0 # Set flag to 1 if mod_t is within 0 to 5% of t_total
    # else:
    #     flag = 1 # Otherwise, set flag to 0

    # Calculate the instantaneous frequency
                        # linear                                                        #exp
    f_inst = (f_start + (f_end - f_start) * (mod_t / t_total)) if mode == 'linear' else (f_start * np.power(f_end / f_start, mod_t / t_total))

    # Calculate the phase increment
    phase = 2*np.pi*  f_inst * mod_t

    # Return the sinusoidal value
    return np.sin(phase) * (f_inst/f_start)**2 # square amplitude to get constant amplitude at low freq


f_start = .2       # Starting frequency in Hz
f_end = 3.      # Ending frequency in Hz
t_total = 20              # Total duration of sweep in seconds
speed_amplitude=MOTOR_FULL_SCALE/((f_end/f_start)**2)
print(f'computed speed_amplitude={speed_amplitude} to reach full scale {MOTOR_FULL_SCALE} at ending frequency {f_end}')
time.sleep(1)
control_period=5e-3
def turn_off_motor(interface:Interface):
    interface.close()
if __name__ == '__main__':
    port=findCartpoleSerialPort()
    if port is None:
        print('serial port not found')
        sys.exit(1)
    print(f'found port {port}')
    interface=Interface()
    interface.open(port=port,baud=230400)

    if not interface.ping():
        print('interface.ping() returned False; cartpole firmware not correct?')
        sys.exit(1)
    interface.calibrate()
    atexit.register(turn_off_motor,interface)
    interface.control_mode(False)
    interface.stream_output(True)
    interface.set_motor(0)
    time.sleep(1)
    interface.set_config_control(controlLoopPeriodMs=1,controlSync=False,angle_deviation=0, avgLen=1, correct_motor_dynamics=False)
    t_start=time.time()
    times=[]
    speeds=[]
    positions=[]
    print(f'starting sinusoidal sweep with speed_amplitude={speed_amplitude} f_start={f_start}Hz, f_end={f_end}Hz t_total={t_total}s')
    try:
        while True:
            t=time.time()-t_start
            if t>t_total: break
            norm_amp=sinusoidal_sweep(t, f_start, f_end,t_total,'exponential')
            # multiply by
            sp=int(norm_amp*speed_amplitude)
            interface.set_motor(sp)
            state=interface.read_state()
            pos=state[1]
            times.append(t)
            speeds.append(sp)
            positions.append(pos)
            # print(f'\rt: {t:10.3f} sp: {sp:10d} pos: {state[2]} angle: {state[0]}                            ',end='')
            time.sleep(control_period)
        print('done')
        interface.close()
    except KeyboardInterrupt:
        print('\n\ninterrupted, turning off motor')
        interface.close()

    times=np.array(times)
    positions=np.array(positions)
    speeds=np.array(speeds)
    plt.plot(times,positions)
    plt.xlabel('time (s)')
    plt.ylabel('cart motor position (encoder units)')
    plt.show()
