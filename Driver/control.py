import time

import numpy as np

import pygame.joystick as joystick  # https://www.pygame.org/docs/ref/joystick.html

from DriverFunctions.custom_logging import my_logger
from DriverFunctions.custom_serial_functions import setup_serial_connection
from DriverFunctions.pendulum import Pendulum
from DriverFunctions.kbhit import KBHit
from DriverFunctions.measure import StepResponseMeasurement
from DriverFunctions.utilities import help, calibrate, terminal_check
from DriverFunctions.joystick import setup_joystick, get_stick_position, motorCmd_from_joystick

from CartPole.state_utilities import create_cartpole_state, STATE_INDICES

from DriverFunctions.controllers_management import set_controller
from DriverFunctions.firmware_parameters import set_firmware_parameters
from DriverFunctions.csv_helpers import csv_init

from globals import *

from math import fmod

def wrap_angle_rad(angle: float) -> float:
    Modulo = fmod(angle, 2 * np.pi)  # positive modulo
    if Modulo < -np.pi:
        angle = Modulo + 2 * np.pi
    elif Modulo > np.pi:
        angle = Modulo - 2 * np.pi
    else:
        angle = Modulo
    return angle

# TODO Why after calibration Cartpole is not at 0 position?
# TODO Aftrer joystick is unplugged and plugged again it interferes with the calibration, it causes the motor to get stuck at some speed after calibration. Add this to the readme file to warn the user.

angle_smoothing = 1.0
# angleD_smoothing = 0.8

# TODO: You can easily switch between controllers in runtime using this and get_available_controller_names function
controller, _, _ = set_controller(controller_name=CONTROLLER_NAME)

log = my_logger(__name__)

# check that we are running from terminal, otherwise we cannot control it
terminal_check()

CartPoleInstance = Pendulum()

setup_serial_connection(CartPoleInstance, SERIAL_PORT=SERIAL_PORT)

CartPoleInstance.control_mode(False)
CartPoleInstance.stream_output(False)

log.info('\n opened ' + str(SERIAL_PORT) + ' successfully')

stick, joystickMode = setup_joystick()

if CALIBRATE:
    POSITION_OFFSET = calibrate(CartPoleInstance)

try:
    controller.loadparams()
except AttributeError:
    print('loadparams not defined for this controller')

time.sleep(1)

set_firmware_parameters(CartPoleInstance, ANGLE_AVG_LENGTH=ANGLE_AVG_LENGTH)

printCount = 0

controlEnabled = False
firmwareControl = False
manualMotorSetting = False

danceEnabled = False
danceAmpl = 0.1  # m
dancePeriodS = 20.0
dance_start_time = 0.0

loggingEnabled = False

try:
    kb = KBHit()  # can only use in posix terminal; cannot use from spyder ipython console for example
    kbAvailable = True
except:
    kbAvailable = False

measurement = StepResponseMeasurement()

try:
    controller.printparams()
except AttributeError:
    print('printparams not implemented for this controller.')

help()

startTime = time.time()
lastTime = startTime
lastControlTime = lastTime

angleErr = 0
positionErr = 0  # for printing even if not controlling

CartPoleInstance.stream_output(True)  # now start streaming state

calculatedMotorCmd = 0

csvfile = None
csvfilename = None
csvwriter = None

anglePrev = 0
positionPrev = 0

angleDPrev = 0
positionDPrev = 0

################################################################################
# CONTROL LOOP (PC BASED)
################################################################################
while True:

    # Keyboard input
    if kbAvailable & kb.kbhit():
        c = kb.getch()
        #Keys used in controller: 1,2,3,4,p, =, -, w, q, s, a, x, z, r, e, f, d, v, c, S, L, b, j
        try:
            controller.keyboard_input(c)
        except AttributeError:
            pass
        # FIXME ,./ commands not working as intended - control overwrites motor value
        if c == '.':  # zero motor
            controlEnabled = False
            calculatedMotorCmd = 0
            manualMotorSetting = False
            print('\r actual motor command after .', calculatedMotorCmd)
        elif c == ',':  # left
            controlEnabled = False
            calculatedMotorCmd += 100
            manualMotorSetting = True
            print('\r actual motor command after ,', calculatedMotorCmd)
        elif c == '/':  # right
            controlEnabled = False
            calculatedMotorCmd -= 100
            manualMotorSetting = True
            print('\r actual motor command after /', calculatedMotorCmd)
        elif c == 'D':
            # We want the sinusoid to start at predictible (0) position
            if danceEnabled is True:
                danceEnabled = False
            else:
                dance_start_time = time.time()
                danceEnabled = True
            print("\ndanceEnabled= {0}".format(danceEnabled))
        elif c == 'l':
            loggingEnabled = ~loggingEnabled
            print("\nloggingEnabled= {0}".format(loggingEnabled))
            if loggingEnabled:
                try:
                    csvfilename, csvfile, csvwriter = csv_init()
                    print("\n Started logging data to " + csvfilename)
                except Exception as e:
                    loggingEnabled = False
                    print("\n" + str(e) + ": Exception opening csvfile; logging disabled \r\n")
            else:
                csvfile.close()
                print("\n Stopped logging data to " + csvfilename)
        elif c == 'u': # toggle firmware control
            firmwareControl = not firmwareControl
            print(firmwareControl)
            print(firmwareControl)
            CartPoleInstance.control_mode(firmwareControl)
        elif c == 'k':
            if controlEnabled is False:
                controlEnabled = True
            else:
                controlEnabled = False
                controller.controller_reset()
                calculatedMotorCmd = 0
            danceEnabled = False
            print("\ncontrolEnabled= {0} \r\n".format(controlEnabled))
        elif c == 'K':
            controlEnabled = False
            POSITION_OFFSET = calibrate(CartPoleInstance)
        elif c == 'h' or c == '?':
            help()
        # Fine tune angle deviation
        elif c == '=':
            ANGLE_DEVIATION_FINETUNE += 0.01
            print("\nIncreased angle deviation fine tune value to {0}\n".format(ANGLE_DEVIATION_FINETUNE))
        # Decrease Target Angle
        elif c == '-':
            ANGLE_DEVIATION_FINETUNE -= 0.01
            print("\nDecreased angle deviation fine tune value to {0}\n".format(ANGLE_DEVIATION_FINETUNE))

        # Increase Target Position
        elif c == ']':
            POSITION_TARGET += 10 * POSITION_NORMALIZATION_FACTOR
            if POSITION_TARGET > 0.8*(POSITION_ENCODER_RANGE//2):
               POSITION_TARGET = 0.8*(POSITION_ENCODER_RANGE//2)
            print("\nIncreased target position to {0} cm\n".format(POSITION_TARGET*100))
        # Decrease Target Position
        elif c == '[':
            POSITION_TARGET -= 10 * POSITION_NORMALIZATION_FACTOR
            if POSITION_TARGET < -0.8*(POSITION_ENCODER_RANGE//2):
                POSITION_TARGET = -0.8*(POSITION_ENCODER_RANGE//2)
            print("\nDecreased target position to {0} cm\n".format(POSITION_TARGET*100))
        elif c == 'm':  # toggle measurement
            if measurement.is_idle():
                measurement.start()
            else:
                measurement.stop()
        elif c=='j':
            if joystickMode is None:
                log.warning('no joystick')
            elif joystickMode == 'not active':
                joystickMode = 'speed'
                log.info(f'set joystick to cart {joystickMode} control mode')
            elif joystickMode=='speed':
                joystickMode='position'
                log.info(f'set joystick to cart {joystickMode} control mode')
            elif joystickMode=='position':
                joystickMode='not active'
                log.info(f'set joystick to {joystickMode} mode')

        elif c=='b':
            angle_average = 0
            number_of_measurements = 100
            for _ in range(number_of_measurements):
                CartPoleInstance.clear_read_buffer()  # if we don't clear read buffer, state output piles up in serial buffer #TODO
                (angle, position, command, sent, received) = CartPoleInstance.read_state()
                # print('Sensor reading to adjust ANGLE_HANGING', angle)
                angle_average += angle
            angle_average = angle_average / float(number_of_measurements)
            print('Hanging angle average of {} measurements: {}     '.format(number_of_measurements,angle_average))

        # Exit
        elif ord(c) == 27:  # ESC
            log.info("\nquitting....")
            break

    # This function will block at the rate of the control loop
    CartPoleInstance.clear_read_buffer()  # if we don't clear read buffer, state output piles up in serial buffer #TODO
    (angle, position, command, sent, received) = CartPoleInstance.read_state()
    # angle count is more positive CCW facing cart

    position_centered = position-POSITION_OFFSET
    # position encoder count is grows to right facing cart for stock motor, grows to left for Pololu motor
    # Hence we revert sign for Pololu
    if MOTOR_TYPE == 'POLOLU':
        position_centered = -position_centered

    # Convert position and angle to physical units
    angle = (angle + ANGLE_DEVIATION - ANGLE_ADC_RANGE / 2) * ANGLE_NORMALIZATION_FACTOR - ANGLE_DEVIATION_FINETUNE
    position = position_centered * POSITION_NORMALIZATION_FACTOR

    # Filter
    angle = angle * (angle_smoothing) + (1 - angle_smoothing) * anglePrev
    angle = wrap_angle_rad(angle)

    # Time measurement
    timeNow = time.time()
    deltaTime = timeNow - lastTime
    if deltaTime == 0:
        deltaTime = 1e-6
    lastTime = timeNow
    elapsedTime = timeNow - startTime

    # Calculating derivatives (cart velocity and angular velocity of the pole)
    angleDerivative = (angle - anglePrev)/deltaTime  # rad/s
    positionDerivative = (position - positionPrev)/deltaTime  # m/s

    # Keep values of angle and position for next timestep, for smoothing and derivative calculation
    anglePrev = angle
    positionPrev = position

#    # Filtering of derivatives
#    angleDerivative = angleDerivative*angleD_smoothing + (1-angleD_smoothing)*angleDPrev
#    angleDPrev = angleDerivative

    # Calculate sine and cosie of the angle
    angle_cos = np.cos(angle)
    angle_sin = np.sin(angle)

    # Get the target position
    # if controlEnabled and danceEnabled:
    if danceEnabled:
        target_position = POSITION_TARGET + danceAmpl * np.sin(2 * np.pi * ((timeNow-dance_start_time) / dancePeriodS))
    else:
        target_position = POSITION_TARGET

    # Pack the state into interface acceptable for the controller
    s = create_cartpole_state()
    s[STATE_INDICES['position']] = position
    s[STATE_INDICES['angle']] = angle
    s[STATE_INDICES['positionD']] = positionDerivative
    s[STATE_INDICES['angleD']] = angleDerivative
    s[STATE_INDICES['angle_cos']] = angle_cos
    s[STATE_INDICES['angle_sin']] = angle_sin

    if controlEnabled and timeNow - lastControlTime >= CONTROL_PERIOD_MS * .001:
        lastControlTime = timeNow
        calculatedMotorCmd = controller.step(s=s, target_position=target_position, time=timeNow)
        calculatedMotorCmd *= MOTOR_FULL_SCALE
        calculatedMotorCmd = int(calculatedMotorCmd)

    if joystickMode is None or joystickMode == 'not active':
        stickPos = 0.0
        stickControl = False
        if controlEnabled and not manualMotorSetting:
            ...
        elif manualMotorSetting == False:
            calculatedMotorCmd = 0
    else:
        stickPos = get_stick_position(stick)
        stickPos = stickPos * POSITION_FULL_SCALE_N * POSITION_NORMALIZATION_FACTOR
        stickControl = True
        calculatedMotorCmd = motorCmd_from_joystick(joystickMode, stickPos, position)


    if not measurement.is_idle():
        try:
            measurement.update_state(angle, position, timeNow)
            calculatedMotorCmd = measurement.motor
        except TimeoutError as e:
            log.warning(f'timeout in measurement: {e}')

    # We save motor input BEFORE processing which should linearize (perceived) motor model
    # TODO: It is not fully cleare if it is the right place for the following line
    #   I would prefere to have it before "linearization" and after clipping, but lin. goes before clipping
    #   And I didn't want to have clipping twice (maybe I should?)
    actualMotorCmd = calculatedMotorCmd

    # A manual calibration to linearize around origin
    #  Model_velocity.py in CartPole simulator is the script to determine these values
    # The change dependent on velocity sign is motivated theory of classical friction
    if actualMotorCmd != 0:
        if np.sign(s[STATE_INDICES['positionD']]) > 0:
            actualMotorCmd += 387
        elif np.sign(s[STATE_INDICES['positionD']]) < 0:
            actualMotorCmd -= 330

    # clip motor to  limits - we clip it to the half of the max power
    # This assures both: safety against burning the motor and keeping motor in its linear range
    # NEVER TRY TO RUN IT WITH
    actualMotorCmd = int(0.6*MOTOR_MAX_PWM) if actualMotorCmd  > 0.6*MOTOR_MAX_PWM else actualMotorCmd
    actualMotorCmd = -int(0.6*MOTOR_MAX_PWM) if actualMotorCmd  < -0.6*MOTOR_MAX_PWM else actualMotorCmd

    # Temporary safety switch off if went to the boundary
    if abs(position_centered)>0.9*(POSITION_ENCODER_RANGE//2):
        controlEnabled = False
        controller.controller_reset()
        danceEnabled = False
        calculatedMotorCmd = 0

    # Reverse sign if you are using pololu motor and not the original one
    if MOTOR_TYPE == 'POLOLU':
        actualMotorCmd = -actualMotorCmd

    CartPoleInstance.set_motor(actualMotorCmd)

    if loggingEnabled:
        csvwriter.writerow([elapsedTime, deltaTime * 1000, angle, angleDerivative, angle_cos, angle_sin, position, positionDerivative, controller.ANGLE_TARGET, controller.angleErr, target_position, controller.positionErr, controller.angleCmd, controller.positionCmd, calculatedMotorCmd, calculatedMotorCmd/MOTOR_FULL_SCALE, stickControl, stickPos, measurement, sent, received, received-sent, CartPoleInstance.end-CartPoleInstance.start])

        # Print outputL
    printCount += 1
    if printCount >= (PRINT_PERIOD_MS / CONTROL_PERIOD_MS):
        printCount = 0
        positionErr = s[STATE_INDICES['position']] - target_position
        # print("\r a {:+6.3f}rad  p {:+6.3f}cm pErr {:+6.3f}cm aCmd {:+6d} pCmd {:+6d} mCmd {:+6d} dt {:.3f}ms  stick {:.3f}:{} meas={}        \r"
        print(
            "\r angle:{:+6.3f}rad  position:{:+6.3f}cm position error:{:+6.3f}cm mCmd {:+6d} dt {:.3f}ms  stick {:.3f}:{} meas={}  sent:{}, received:{}, latency:{}, python latency:{}     \r"
              .format(angle,
                      position*100,
                      positionErr*100,
                      calculatedMotorCmd,
                      deltaTime * 1000,
                      stickPos,
                      stickControl,
                      measurement,
                      sent, received, received-sent, CartPoleInstance.end-CartPoleInstance.start)
              , end='')
# if we pause like below, state info piles up in serial input buffer
# instead loop at max possible rate to get latest state info
#    time.sleep(CONTROL_PERIOD_MS*.001)  # not quite correct since there will be time for execution below

# when x hit during loop or other loop exit
CartPoleInstance.set_motor(0)  # turn off motor
CartPoleInstance.close()
joystick.quit()

if loggingEnabled:
    csvfile.close()

# todo fix get and set params (for chip interface), must be compatible with sensor readings
# todo check if position unit conversion works for the following features: dance mode (can be checked for a nice controller only)