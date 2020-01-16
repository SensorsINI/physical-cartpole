import time
import kbhit
import json
from pendulum import Pendulum

SERIAL_PORT             = "/dev/ttyUSB0" # might move if other devices plugged in
SERIAL_BAUD             = 230400  # default 230400, in firmware.   Alternatives if compiled and supported by USB serial intervace are are 115200, 128000, 153600, 230400, 460800, 921600, 1500000, 2000000
PRINT_PERIOD_MS         = 100  # shows state every this many ms
CONTROL_PERIOD_MS       = 5
CALIBRATE               = False # True
MOTOR_FULL_SCALE        = 7199
MOTOR_MAX_PWM           = int(round(0.95 * MOTOR_FULL_SCALE))

ANGLE_TARGET            = 3129 # 3383  # adjust to exactly vertical angle value, read by inspecting angle output
ANGLE_CTRL_PERIOD_MS    = 5         # Must be a multiple of CONTROL_PERIOD_MS
ANGLE_AVG_LENGTH        = 4 # adc routine in firmware reads ADC this many times quickly in succession to reduce noise
ANGLE_SMOOTHING         = 1      # 1.0 turns off smoothing
ANGLE_KP                = 400
ANGLE_KD                = 400

POSITION_TARGET         = 0 # 1200
POSITION_CTRL_PERIOD_MS = 25        # Must be a multiple of CONTROL_PERIOD_MS
POSITION_SMOOTHING      = 1       # 1.0 turns off smoothing
POSITION_KP             = 20
POSITION_KD             = 300

def saveparams():
    print("\nSaving parameters")
    p={}
    p['ANGLE_TARGET']=ANGLE_TARGET
    p['ANGLE_KP']=ANGLE_KP
    p['ANGLE_KD']=ANGLE_KD
    p['POSITION_TARGET']=POSITION_TARGET
    p['POSITION_KP']=POSITION_KP
    p['POSITION_KD']=POSITION_KD
    p['ANGLE_SMOOTHING']=ANGLE_SMOOTHING
    p['POSITION_SMOOTHING']=ANGLE_SMOOTHING
    with open('control.json','w') as f:
        json.dump(p,f)
    
def loadparams():
    print("\nLoading parameters")
    f=open('control.json')
    try:
        p=json.load(f)
        global ANGLE_TARGET, ANGLE_KP,ANGLE_KD,POSITION_TARGET,POSITION_KP,POSITION_KD,ANGLE_SMOOTHING,POSITION_SMOOTHING
        ANGLE_TARGET=p['ANGLE_TARGET']
        ANGLE_KP=p['ANGLE_KP']
        ANGLE_KD=p['ANGLE_KD']
        POSITION_TARGET=p['POSITION_TARGET']
        POSITION_KP=p['POSITION_KP']
        POSITION_KD=p['POSITION_KD']
        ANGLE_SMOOTHING=p['ANGLE_SMOOTHING']
        POSITION_SMOOTHING=p['POSITION_SMOOTHING']
    except:
        print("something went wrong loading parameters")
    printparams()
  
    

def help():
    print("\n***********************************")
    print("keystroke commands")
    print("ESC quit")
    print("k toggle control on/off (initially off)")
    print("K trigger motor position calibration")
    print("=/- increase/decrease angle target")
    print("[/] increase/decrease position target")
    print("w/q angle proportional gain")
    print("s/a angle derivative gain")
    print("z/x angle smoothing")    
    print("r/e position proportional gain")
    print("f/d position derivative gain")
    print("c/v position smoothing")
    print("S/L Save/Load param values from disk")
    print("***********************************")

def printparams():
    print("\nAngle PD Control Parameters")
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

ratio=1.05
   

def inc(param):
    if param < 2:
        param = round(param + 0.1, 1)
    else:
        old=param
        param = round(param*ratio)
        if param==old: 
            param+=1
    return param

def dec(param):
    if param < 2:
        param = max(0,round(param - 0.1, 1))
    else:
        old=param
        param = round(param/ratio)
        if param==old: 
            param-=1
    return param
    
    
    

if ANGLE_CTRL_PERIOD_MS < CONTROL_PERIOD_MS or POSITION_CTRL_PERIOD_MS <CONTROL_PERIOD_MS:
    raise Exception("angle or position control periods too short compared to CONTROL_PERIOD_MS")


################################################################################
# OPEN SERIAL PORT
################################################################################
p = Pendulum()
p.open(SERIAL_PORT, SERIAL_BAUD)
p.control_mode(False)
p.stream_output(False)

    


if CALIBRATE:
    print("Calibrating motor position....")
    if not p.calibrate():
        print("Failed to connect to device")
        p.close()
        exit()
    print("Done calibrating")

time.sleep(1)



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


################################################################################
# CONTROL LOOP (PC BASED)
################################################################################
printCount          = 0

angleErrPrev        = 0
angleCmd            = 0
positionErrPrev     = 0
positionCmd         = 0

controlEnabled=False

kbAvailable=True
try:
    kb = kbhit.KBHit() # can only use in posix terminal; cannot use from spyder ipython console for example
except:
    kbAvailable=False
    

printparams()
help()
startTime=time.time()
lastTime=startTime
lastAngleControlTime=lastTime
lastPositionControlTime=lastTime
angleErr=0
positionErr=0 # for printing even if not controlling
p.stream_output(True)  # now start streaming state
while True:
    
    # Adjust Parameters
    if kbAvailable & kb.kbhit():
        c = kb.getch()

        if c == 'k':
            controlEnabled=~controlEnabled
            print("\ncontrolEnabled= {0}".format(controlEnabled))
        elif c == 'K':
           controlEnabled=False
           print("\nCalibration triggered")
           p.calibrate()
        elif c == 'h' or c=='?':
            help()
        elif c == 'p' :
            printparams()
        # Increase Target Angle
        elif c == '=':
            ANGLE_TARGET += 1
            print("\nIncreased target angle to {0}".format(ANGLE_TARGET))
        # Decrease Target Angle
        elif c == '-':
            ANGLE_TARGET -= 1
            print("\nDecreased target angle to {0}".format(ANGLE_TARGET))

        # Increase Target Position
        elif c == ']':
            POSITION_TARGET += 10
            print("\nIncreased target position to {0}".format(POSITION_TARGET))
        # Decrease Target Position
        elif c == '[':
            POSITION_TARGET -= 10
            print("\nDecreased target position to {0}".format(POSITION_TARGET))
            
        # Angle Gains
        elif c == 'w':
            ANGLE_KP=inc(ANGLE_KP)
            print("\nIncreased angle KP {0}".format(ANGLE_KP))
        elif c == 'q':
            ANGLE_KP=dec(ANGLE_KP)           
            print("\nDecreased angle KP {0}".format(ANGLE_KP))
        elif c == 's':
            ANGLE_KD=inc(ANGLE_KD)
            print("\nIncreased angle KD {0}".format(ANGLE_KD))
        elif c == 'a':
            ANGLE_KD=dec(ANGLE_KD)
            print("\nDecreased angle KD {0}".format(ANGLE_KD))
        elif c == 'x':
            ANGLE_SMOOTHING=dec(ANGLE_SMOOTHING)
            if ANGLE_SMOOTHING>1: 
                ANGLE_SMOOTHING=1
            print("\nIncreased ANGLE_SMOOTHING {0}".format(ANGLE_SMOOTHING))
        elif c == 'z':
            ANGLE_SMOOTHING=inc(ANGLE_SMOOTHING)
            if ANGLE_SMOOTHING>1: 
                ANGLE_SMOOTHING=1
            print("\nDecreased ANGLE_SMOOTHING {0}".format(ANGLE_SMOOTHING))

        # Position Gains
        elif c == 'r':
            POSITION_KP=inc(POSITION_KP)
            print("\nIncreased position KP {0}".format(POSITION_KP))
        elif c == 'e':
            POSITION_KP=dec(POSITION_KP)
            print("\nDecreased position KP {0}".format(POSITION_KP))
        elif c == 'f':
            POSITION_KD=inc(POSITION_KD)
            print("\nIncreased position KD {0}".format(POSITION_KD))
        elif c == 'd':
            POSITION_KD=dec(POSITION_KD)
            print("\nDecreased position KD {0}".format(POSITION_KD))
        elif c == 'v':
            POSITION_SMOOTHING=dec(POSITION_SMOOTHING)
            if POSITION_SMOOTHING>1: 
                POSITION_SMOOTHING=1
            print("\nIncreased POSITION_SMOOTHING {0}".format(POSITION_SMOOTHING))
        elif c == 'c':
            POSITION_SMOOTHING=inc(POSITION_SMOOTHING)
            if POSITION_SMOOTHING>1: 
                POSITION_SMOOTHING=1
            print("\nDecreased POSITION_SMOOTHING {0}".format(POSITION_SMOOTHING))
        elif c=='S':
            saveparams()
        elif c=='L':
            loadparams()

        # Exit
        elif ord(c) == 27 : # ESC
            print("\nquitting....")
            break

    # This function will block at the rate of the control loop
#    p.clear_read_buffer() # if we don't clear read buffer, state output piles up in serial buffer
    (angle, position, command) = p.read_state() 
    # angle count is more positive CCW facing cart, position encoder count is more positive to right facing cart
    
     
    timeNow=time.time()
    # Balance PD Control
    # Position PD Control
    if timeNow -lastPositionControlTime >= POSITION_CTRL_PERIOD_MS*.001:
        lastPositionControlTime=timeNow
        positionErr         = POSITION_SMOOTHING*(position - POSITION_TARGET) + (1.0 - POSITION_SMOOTHING)*positionErrPrev # First order low-P=pass filter
        positionErrDiff     = positionErr - positionErrPrev
        positionErrPrev     = positionErr
        # Naive solution: if too positive (too right), move left (minus on positionCmd), 
        # but this does not produce correct control.
        # The correct strategy is that if cart is too positive (too right),
        # produce lean to the left by introducing a positive set point angle leaning slightly to left, 
        # i.e. more positve positionErr makes more positive effective ANGLE_TARGET
        # End result is that sign of positionCmd is flipped
        # Also, if positionErr is increasing more, then we want even more lean, so D sign is also positive
        positionCmd         = +(POSITION_KP*positionErr + POSITION_KD*positionErrDiff) 

    if timeNow-lastAngleControlTime >= ANGLE_CTRL_PERIOD_MS*.001:
        lastAngleControlTime=timeNow
        angleErr            = ANGLE_SMOOTHING*(angle - ANGLE_TARGET) + (1.0 - ANGLE_SMOOTHING)*angleErrPrev # First order low-pass filter
        angleErrDiff        = angleErr - angleErrPrev
        angleErrPrev        = angleErr
        angleCmd            = -(ANGLE_KP*angleErr + ANGLE_KD*angleErrDiff) # if too CCW (pos error), move cart left

    
    motorCmd = int(round(angleCmd + positionCmd)) # change to plus for original, check that when cart is displayed, the KP term for cart position leans cart the correct direction
    motorCmd =  MOTOR_MAX_PWM if motorCmd >  MOTOR_MAX_PWM else motorCmd
    motorCmd = -MOTOR_MAX_PWM if motorCmd < -MOTOR_MAX_PWM else motorCmd
    if controlEnabled:
        # Send motor command
        p.set_motor(motorCmd) # positive motor cmd moves cart right 
    else:
        p.set_motor(0) # turn off motor
    deltaTime=timeNow-lastTime
    lastTime=timeNow
    # Print output
    printCount += 1
    if printCount >= (PRINT_PERIOD_MS/CONTROL_PERIOD_MS):
        printCount = 0
        elapsedTime=timeNow-startTime
        print("\r angle {:+4d} angleErr {:+6.1f} position {:+6d} positionErr {:+6.1f} angleCmd {:+6d} positionCmd {:+6d} motorCmd {:+6d} dt {:.3f}ms            \r".format(int(angle), angleErr, int(position),  positionErr, int(round(angleCmd)), int(round(positionCmd)), motorCmd, deltaTime*1000) , end = '')
# if we pause like below, state info piles up in serial input buffer
        # instead loop at max possible rate to get latest state info
#    time.sleep(CONTROL_PERIOD_MS*.001)  # not quite correct since there will be time for execution below

# when x hit during loop or other loop exit
p.close()
    


