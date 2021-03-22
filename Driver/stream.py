import time
import kbhit
from pendulum import Pendulum

SERIAL_PORT             = "/dev/ttyUSB0"
SERIAL_BAUD             = 230400
PRINT_PERIOD_MS         = 1000
CONTROL_PERIOD_MS       = 5
CALIBRATE               = False #True
SAVE_DATA               = True
REC_SESSION_SEC         = 5 * 60

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

time.sleep(1)

################################################################################
# GET PARAMETERS
################################################################################
(   ANGLE_TARGET, 
    ANGLE_AVG_LENGTH, 
    ANGLE_SMOOTHING, 
    ANGLE_KP,
    ANGLE_KD)           = p.get_angle_config

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
# STREAM LOOP
################################################################################
printCount = 0

kb = kbhit.KBHit()
#p.control_mode(True)
p.stream_output(True)

while True:
    # Adjust Parameters
    if kb.kbhit():
        angleUpdate     = False
        positionUpdate  = False

        c = kb.getch()

        # Increase Target Angle
        if c == '=':
            ANGLE_TARGET += 1
            print("Increased target angle to {0}".format(ANGLE_TARGET))
            angleUpdate = True
        # Decrease Target Angle
        elif c == '-':
            ANGLE_TARGET -= 1
            print("Decreased target angle to {0}".format(ANGLE_TARGET))
            angleUpdate = True

        # Increase Target Position
        elif c == ']':
            POSITION_TARGET += 1
            print("Increased target position to {0}".format(POSITION_TARGET))
            positionUpdate = True
        # Decrease Target Position
        elif c == '[':
            POSITION_TARGET -= 1
            print("Decreased target position to {0}".format(POSITION_TARGET))
            positionUpdate = True
            
        # Angle Gains
        if c == 'w':
            if ANGLE_KP < 1:
                ANGLE_KP = round(ANGLE_KP + 0.1, 1)
            else:
                ANGLE_KP += 1
            print("Increased angle KP {0}".format(ANGLE_KP))
            angleUpdate = True
        elif c == 'q':
            if ANGLE_KP <= 1:
                ANGLE_KP = round(ANGLE_KP - 0.1, 1)
            else:
                ANGLE_KP -= 1
            ANGLE_KP = 0 if ANGLE_KP < 0 else ANGLE_KP
            print("Decreased angle KP {0}".format(ANGLE_KP))
            angleUpdate = True
        elif c == 's':
            if ANGLE_KD < 1:
                ANGLE_KD = round(ANGLE_KD + 0.1, 1)
            else:
                ANGLE_KD += 1
            print("Increased angle KD {0}".format(ANGLE_KD))
            angleUpdate = True
        elif c == 'a':
            if ANGLE_KD <= 1:
                ANGLE_KD = round(ANGLE_KD - 0.1, 1)
            else:
                ANGLE_KD -= 1
            ANGLE_KD = 0 if ANGLE_KD < 0 else ANGLE_KD
            print("Decreased angle KD {0}".format(ANGLE_KD))
            angleUpdate = True

        # Position Gains
        if c == 'r':
            if POSITION_KP < 1:
                POSITION_KP = round(POSITION_KP + 0.1, 1)
            else:
                POSITION_KP += 1
            print("Increased position KP {0}".format(POSITION_KP))
            positionUpdate = True
        elif c == 'e':
            if POSITION_KP <= 1:
                POSITION_KP = round(POSITION_KP - 0.1, 1)
            else:
                POSITION_KP -= 1
            POSITION_KP = 0 if POSITION_KP < 0 else POSITION_KP
            print("Decreased position KP {0}".format(POSITION_KP))
            positionUpdate = True
        elif c == 'f':
            if POSITION_KD < 1:
                POSITION_KD = round(POSITION_KD + 0.1, 1)
            else:
                POSITION_KD += 1
            print("Increased position KD {0}".format(POSITION_KD))
            positionUpdate = True
        elif c == 'd':
            if POSITION_KD <= 1:
                POSITION_KD = round(POSITION_KD - 0.1, 1)
            else:
                POSITION_KD -= 1
            POSITION_KD = 0 if POSITION_KD < 0 else POSITION_KD
            print("Decreased position KD {0}".format(POSITION_KD))
            positionUpdate = True

        # Exit
        elif ord(c) == 27:
            break

        if angleUpdate:
            p.stream_output(False)
            p.set_angle_config(     ANGLE_TARGET,
                                    ANGLE_AVG_LENGTH,
                                    ANGLE_SMOOTHING,
                                    ANGLE_KP,
                                    ANGLE_KD)
            p.stream_output(True)
        elif positionUpdate:
            p.stream_output(False)
            p.set_position_config(  POSITION_TARGET,
                                    POSITION_CTRL_PERIOD_MS,
                                    POSITION_SMOOTHING,
                                    POSITION_KP,
                                    POSITION_KD)
            p.stream_output(True)

    # This function will block at the rate of the control loop
    (angle, position, command) = p.read_state()

    # Print output
    printCount += 1
    if printCount == (PRINT_PERIOD_MS/CONTROL_PERIOD_MS):
        printCount = 0
        print("{0:04} {1:+06} {2:+06}".format(angle, position, command))

p.close()
