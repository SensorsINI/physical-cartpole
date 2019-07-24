import time
import kbhit
from pendulum import Pendulum

SERIAL_PORT             = "/dev/ttyUSB0"
SERIAL_BAUD             = 230400
PRINT_PERIOD_MS         = 1000
CONTROL_PERIOD_MS       = 5
CALIBRATE               = False # True
MOTOR_FULL_SCALE        = 7199
MOTOR_MAX_PWM           = int(round(0.95 * MOTOR_FULL_SCALE))

ANGLE_TARGET            = 3104
ANGLE_CTRL_PERIOD_MS    = 5         # Must be a multiple of CONTROL_PERIOD_MS
ANGLE_AVG_LENGTH        = 32
ANGLE_SMOOTHING         = 1.0       # 1.0 turns off smoothing
ANGLE_KP                = 300
ANGLE_KD                = 400

POSITION_TARGET         = 0
POSITION_CTRL_PERIOD_MS = 25        # Must be a multiple of CONTROL_PERIOD_MS
POSITION_SMOOTHING      = 0.2       # 1.0 turns off smoothing
POSITION_KP             = 100
POSITION_KD             = 200

################################################################################
# OPEN SERIAL PORT
################################################################################
p = Pendulum()
p.open(SERIAL_PORT, SERIAL_BAUD)
p.control_mode(False)
p.stream_output(False)

if CALIBRATE:
    if not p.calibrate():
        print("Failed to connect to device")
        p.close()
        exit()

time.sleep(3)

################################################################################
# SET PARAMETERS
################################################################################
p.set_angle_config(     ANGLE_TARGET,
                        ANGLE_AVG_LENGTH,
                        ANGLE_SMOOTHING,
                        ANGLE_KP,
                        ANGLE_KD)

p.set_position_config(  POSITION_TARGET,
                        POSITION_CTRL_PERIOD_MS,
                        POSITION_SMOOTHING,
                        POSITION_KP,
                        POSITION_KD)

################################################################################
# GET PARAMETERS
################################################################################
(   ANGLE_TARGET, 
    ANGLE_AVG_LENGTH, 
    ANGLE_SMOOTHING, 
    ANGLE_KP,
    ANGLE_KD)           = p.get_angle_config()

(   POSITION_TARGET,
    POSITION_CTRL_PERIOD_MS,
    POSITION_SMOOTHING,
    POSITION_KP,
    POSITION_KD)        = p.get_position_config()

print("Angle PD Control Parameters")
print("    Set point       {0}".format(ANGLE_TARGET))
print("    Average Length  {0}".format(ANGLE_AVG_LENGTH))
print("    Smoothing       {0:.2f}".format(ANGLE_SMOOTHING))
print("    P Gain          {0:.2f}".format(ANGLE_KP))
print("    D Gain          {0:.2f}".format(ANGLE_KD))

print("Position PD Control Parameters")
print("    Set point       {0}".format(POSITION_TARGET))
print("    Control Period  {0} ms".format(POSITION_CTRL_PERIOD_MS))
print("    Smoothing       {0:.2f}".format(POSITION_SMOOTHING))
print("    P Gain          {0:.2f}".format(POSITION_KP))
print("    D Gain          {0:.2f}".format(POSITION_KD))

################################################################################
# CONTROL LOOP (PC BASED)
################################################################################
printCount          = 0
angleCtrlCount      = 0
positionCtrlCount   = 0

angleErrPrev        = 0
angleCmd            = 0
positionErrPrev     = 0
positionCmd         = 0

kb = kbhit.KBHit()
p.stream_output(True)

while True:
    # Adjust Parameters
    if kb.kbhit():
        c = kb.getch()

        # Increase Target Angle
        if c == '=':
            ANGLE_TARGET += 1
            print("Increased target angle to {0}".format(ANGLE_TARGET))
        # Decrease Target Angle
        elif c == '-':
            ANGLE_TARGET -= 1
            print("Decreased target angle to {0}".format(ANGLE_TARGET))

        # Increase Target Position
        elif c == ']':
            POSITION_TARGET += 1
            print("Increased target position to {0}".format(POSITION_TARGET))
        # Decrease Target Position
        elif c == '[':
            POSITION_TARGET -= 1
            print("Decreased target position to {0}".format(POSITION_TARGET))
            
        # Angle Gains
        if c == 'w':
            if ANGLE_KP < 1:
                ANGLE_KP = round(ANGLE_KP + 0.1, 1)
            else:
                ANGLE_KP += 1
            print("Increased angle KP {0}".format(ANGLE_KP))
        elif c == 'q':
            if ANGLE_KP <= 1:
                ANGLE_KP = round(ANGLE_KP - 0.1, 1)
            else:
                ANGLE_KP -= 1
            ANGLE_KP = 0 if ANGLE_KP < 0 else ANGLE_KP
            print("Decreased angle KP {0}".format(ANGLE_KP))
        elif c == 's':
            if ANGLE_KD < 1:
                ANGLE_KD = round(ANGLE_KD + 0.1, 1)
            else:
                ANGLE_KD += 1
            print("Increased angle KD {0}".format(ANGLE_KD))
        elif c == 'a':
            if ANGLE_KD <= 1:
                ANGLE_KD = round(ANGLE_KD - 0.1, 1)
            else:
                ANGLE_KD -= 1
            ANGLE_KD = 0 if ANGLE_KD < 0 else ANGLE_KD
            print("Decreased angle KD {0}".format(ANGLE_KD))

        # Position Gains
        if c == 'r':
            if POSITION_KP < 1:
                POSITION_KP = round(POSITION_KP + 0.1, 1)
            else:
                POSITION_KP += 1
            print("Increased position KP {0}".format(POSITION_KP))
        elif c == 'e':
            if POSITION_KP <= 1:
                POSITION_KP = round(POSITION_KP - 0.1, 1)
            else:
                POSITION_KP -= 1
            POSITION_KP = 0 if POSITION_KP < 0 else POSITION_KP
            print("Decreased position KP {0}".format(POSITION_KP))
        elif c == 'f':
            if POSITION_KD < 1:
                POSITION_KD = round(POSITION_KD + 0.1, 1)
            else:
                POSITION_KD += 1
            print("Increased position KD {0}".format(POSITION_KD))
        elif c == 'd':
            if POSITION_KD <= 1:
                POSITION_KD = round(POSITION_KD - 0.1, 1)
            else:
                POSITION_KD -= 1
            POSITION_KD = 0 if POSITION_KD < 0 else POSITION_KD
            print("Decreased position KD {0}".format(POSITION_KD))

        # Exit
        elif ord(c) == 27:
            break

    # This function will block at the rate of the control loop
    (angle, position, command) = p.read_state()

    # Balance PD Control
    angleCtrlCount += 1
    if angleCtrlCount == (ANGLE_CTRL_PERIOD_MS/CONTROL_PERIOD_MS):
        angleCtrlCount      = 0
        angleErr            = ANGLE_SMOOTHING*(angle - ANGLE_TARGET) + (1.0 - ANGLE_SMOOTHING)*angleErrPrev # First order low-pass filter
        angleErrDiff        = angleErr - angleErrPrev
        angleErrPrev        = angleErr
        angleCmd            = -ANGLE_KP*angleErr - ANGLE_KD*angleErrDiff

    # Position PD Control
    positionCtrlCount += 1
    if positionCtrlCount == (POSITION_CTRL_PERIOD_MS/CONTROL_PERIOD_MS):
        positionCtrlCount   = 0
        positionErr         = POSITION_SMOOTHING*(position - POSITION_TARGET) + (1.0 - POSITION_SMOOTHING)*positionErrPrev # First order low-P=pass filter
        positionErrDiff     = positionErr - positionErrPrev
        positionErrPrev     = positionErr
        positionCmd         = POSITION_KP*positionErr + POSITION_KD*positionErrDiff
    else:
        positionCmd         = 0

    # Send motor command
    motorCmd = int(round(angleCmd - positionCmd))
    motorCmd =  MOTOR_MAX_PWM if motorCmd >  MOTOR_MAX_PWM else motorCmd
    motorCmd = -MOTOR_MAX_PWM if motorCmd < -MOTOR_MAX_PWM else motorCmd
    p.set_motor(motorCmd)

    # Print output
    printCount += 1
    if printCount == (PRINT_PERIOD_MS/CONTROL_PERIOD_MS):
        printCount = 0
        print("{0:04} {1:+06} {2:+06} {3:+06} {4:+06}".format(angle, position, int(round(angleCmd)), int(round(positionCmd)), motorCmd))

p.close()
