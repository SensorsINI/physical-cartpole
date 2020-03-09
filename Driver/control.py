import time
import json
import math
import csv
import serial # conda install pyserial
import sys
import glob
# pygame needs python 3.6, not available for 3.7
import pygame # conda install -c cogsci pygame; maybe because it only is supplied for earlier python, might need conda install -c evindunn pygame ; sudo apt-get install libsdl-ttf2.0-0
import pygame.joystick as joystick # https://www.pygame.org/docs/ref/joystick.html
from datetime import datetime
# our imports
import kbhit
from pendulum import Pendulum

POLOLU_MOTOR            = False # set true to set options for this motor, which has opposite sign for set_motor TODO needs fixing in firmware or wiring of motor

SERIAL_PORT             = "COM4" #"/dev/ttyUSB0" # might move if other devices plugged in
SERIAL_BAUD             = 230400  # default 230400, in firmware.   Alternatives if compiled and supported by USB serial intervace are are 115200, 128000, 153600, 230400, 460800, 921600, 1500000, 2000000
PRINT_PERIOD_MS         = 100  # shows state every this many ms
CONTROL_PERIOD_MS       = 5
CALIBRATE               = False #False # important to calibrate if running standalone to avoid motor burnout because limits are determined during this calibration
MOTOR_FULL_SCALE        =  7199 # 7199 # with pololu motor and scaling in firmware #7199 # with original motor 
MOTOR_MAX_PWM           = int(round(0.95 * MOTOR_FULL_SCALE))

JOYSTICK_SCALING        = MOTOR_MAX_PWM # how much joystick value -1:1 should be scaled to motor command
JOYSTICK_DEADZONE       = 0.05 # deadzone around joystick neutral position that stick is ignored

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

def serial_ports(): # from https://stackoverflow.com/questions/12090503/listing-available-com-ports-with-python
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
    p['POSITION_SMOOTHING']=POSITION_SMOOTHING
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
    print("l toggle logging data")
    print("S/L Save/Load param values from disk")
    print("D Toggle dance mode")
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

# check that we are running from terminal, otherwise we cannot control it
if sys.stdin.isatty():
    # running interactively
    print('running interactively from an interactive terminal, ok')
else:
   print('run from an interactive terminal to allow keyboard input')
   quit()

################################################################################
# OPEN SERIAL PORT
################################################################################
p = Pendulum()
serialPorts=serial_ports()
print('Available serial ports: '+str(serialPorts))
if len(serialPorts)==0:
    print('no serial ports available, or cannot open it; check linux permissions\n Under linux, sudo chmod a+rw [port] transiently, or add user to dialout or tty group')
    quit()

if len(serialPorts)>1:
    print(str(len(serialPorts))+' serial ports, taking first one which is '+str(serialPorts[0]))
SERIAL_PORT=str(serialPorts[0])
try:
    p.open(SERIAL_PORT, SERIAL_BAUD)
except:
    print('cannot open port '+str(SERIAL_PORT)+': available ports are '+str(serial_ports()))
    quit()

print('opened '+str(SERIAL_PORT)+' successfully')
p.control_mode(False)
p.stream_output(False)

joystickExists=False
pygame.init()
joystick.init()
if joystick.get_count()==1:
    stick = joystick.Joystick(0)
    stick.init()
    axisNum = stick.get_numaxes()
    buttonNum = stick.get_numbuttons()
    joystickExists=True
    print('joystick found with '+str(axisNum)+' axes and '+str(buttonNum)+' buttons')
else:
    print('no joystick found, only PD control or no control possible')

if CALIBRATE:
    print("Calibrating motor position....")
    if not p.calibrate():
        print("Failed to connect to device")
        p.close()
        exit()
    print("Done calibrating")

loadparams()
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

danceEnabled=False
danceAmpl=500
dancePeriodS=8

loggingEnabled=False

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

        if c=='D':
            danceEnabled=~danceEnabled
            print("\ndanceEnabled= {0}".format(danceEnabled))
        elif c == 'l':
            loggingEnabled=~loggingEnabled
            print("\nloggingEnabled= {0}".format(loggingEnabled))
            if loggingEnabled:
                try:
                    csvfilename=datetime.now().strftime("cartpole-%Y-%m-%d-%H-%M-%S.csv")
                    csvfile=open(csvfilename, 'w', newline='')
                    csvwriter = csv.writer(csvfile, delimiter=',')
                    csvwriter.writerow(['time'] + ['deltaTimeMs']+['angle'] + ['position'] + ['angleTarget']  + ['angleErr'] + ['positionTarget'] + ['positionErr'] + ['angleCmd'] + ['positionCmd'] + ['motorCmd']+['actualMotorCmd'])
                    print("\n Started logging data to "+csvfilename)
                except Exception as e:
                    loggingEnabled=False
                    print("\n" + str(e) + ": Exception opening csvfile; logging disabled")
            else:
                csvfile.close()
                print("\n Stopped logging data to "+csvfilename)

        elif c == 'k':
            controlEnabled=~controlEnabled
            print("\ncontrolEnabled= {0}".format(controlEnabled))
        elif c == 'K':
            controlEnabled=False
            print("\nCalibration triggered")
            p.calibrate()
            print("\nCalibration finished")
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
            POSITION_TARGET += 200
            print("\nIncreased target position to {0}".format(POSITION_TARGET))
        # Decrease Target Position
        elif c == '[':
            POSITION_TARGET -= 200
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
    deltaTime=timeNow-lastTime
    if deltaTime==0:
        deltaTime=1e-6
    lastTime=timeNow
    elapsedTime=timeNow-startTime
    diffFactor=(CONTROL_PERIOD_MS/(deltaTime*1000))
    
    
    positionTargetNow=POSITION_TARGET
    if controlEnabled and danceEnabled:
        positionTargetNow=POSITION_TARGET+danceAmpl*math.sin(2*math.pi*(elapsedTime/dancePeriodS))

        
    # Balance PD Control
    # Position PD Control
    if timeNow -lastPositionControlTime >= POSITION_CTRL_PERIOD_MS*.001:
        lastPositionControlTime=timeNow
        positionErr         = POSITION_SMOOTHING*(position - positionTargetNow) + (1.0 - POSITION_SMOOTHING)*positionErrPrev # First order low-P=pass filter
        positionErrDiff     = (positionErr - positionErrPrev)*diffFactor
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
        angleErrDiff        = (angleErr - angleErrPrev)*diffFactor # correct for actual sample interval; if interval is too long, reduce diff error
        angleErrPrev        = angleErr
        angleCmd            = -(ANGLE_KP*angleErr + ANGLE_KD*angleErrDiff) # if too CCW (pos error), move cart left

    
    motorCmd = int(round(angleCmd + positionCmd)) # change to plus for original, check that when cart is displayed, the KP term for cart position leans cart the correct direction
    motorCmd =  MOTOR_MAX_PWM if motorCmd >  MOTOR_MAX_PWM else motorCmd
    motorCmd = -MOTOR_MAX_PWM if motorCmd < -MOTOR_MAX_PWM else motorCmd

    stickPos=0.0
    if joystickExists:
        # for event in pygame.event.get(): # User did something.
        #     if event.type == pygame.QUIT: # If user clicked close.
        #         done = True # Flag that we are done so we exit this loop.
        #     elif event.type == pygame.JOYBUTTONDOWN:
        #         print("Joystick button pressed.")
        #     elif event.type == pygame.JOYBUTTONUP:
        #         print("Joystick button released.")
        pygame.event.get() # must call get() to handle internal queue
        stickPos=stick.get_axis(0) # 0 left right, 1 front back 2 rotate
    
    if abs(stickPos)>JOYSTICK_DEADZONE:
        actualMotorCmd=int(round(stickPos*JOYSTICK_SCALING))
    elif controlEnabled:
        actualMotorCmd=motorCmd
    else:
        actualMotorCmd=0

    if POLOLU_MOTOR==False:
        p.set_motor(-actualMotorCmd) # positive motor cmd moves cart right
    else:
        p.set_motor(actualMotorCmd)  # positive motor cmd moves cart right

    if loggingEnabled:
#      csvwriter.writerow(['time'] + ['deltaTimeMs']+['angle'] + ['position']  + ['angleErr'] + ['positionErr'] + ['angleCmd'] + ['positionCmd'] + ['motorCmd'])
       csvwriter.writerow([elapsedTime,deltaTime*1000,angle, position, ANGLE_TARGET, angleErr, positionTargetNow, positionErr, angleCmd,positionCmd,motorCmd,actualMotorCmd])
   
    # Print output
    printCount += 1
    if printCount >= (PRINT_PERIOD_MS/CONTROL_PERIOD_MS):
        printCount = 0
        print("\r angle {:+4d} angleErr {:+6.1f} position {:+6d} positionErr {:+6.1f} angleCmd {:+6d} positionCmd {:+6d} motorCmd {:+6d} dt {:.3f}ms  stick {:.3f}         \r".format(int(angle), angleErr, int(position),  positionErr, int(round(angleCmd)), int(round(positionCmd)), actualMotorCmd, deltaTime*1000, stickPos), end = '')
# if we pause like below, state info piles up in serial input buffer
        # instead loop at max possible rate to get latest state info
#    time.sleep(CONTROL_PERIOD_MS*.001)  # not quite correct since there will be time for execution below

# when x hit during loop or other loop exit
p.set_motor(0) # turn off motor
p.close()
joystick.quit()

if loggingEnabled:
    csvfile.close()
    


