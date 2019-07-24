import time
import kbhit
import numpy as np
from pendulum import Pendulum

SERIAL_PORT             = "/dev/ttyUSB0"
SERIAL_BAUD             = 230400
PRINT_PERIOD_MS         = 1000
CONTROL_PERIOD_MS       = 5
CALIBRATE               = False#True
SAVE_FILE               = 'pendulum_battery.txt'
REC_DURATION            = 60       # Seconds

ANGLE_TARGET            = 3103
ANGLE_AVG_LENGTH        = 32
ANGLE_SMOOTHING         = 1.0       # 1.0 turns off smoothing
ANGLE_KP                = 400
ANGLE_KD                = 400

POSITION_TARGET         = 0
POSITION_CTRL_PERIOD_MS = 25       # Must be a multiple of CONTROL_PERIOD_MS
POSITION_SMOOTHING      = 0.5      # 1.0 turns off smoothing
POSITION_KP             = 0#1
POSITION_KD             = 0#5

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
# STREAM LOOP
################################################################################
numSamples  = int(round((REC_DURATION * 1000) / CONTROL_PERIOD_MS))
samples     = np.zeros((numSamples, 3), dtype=np.int16)
quitRec     = False

kb = kbhit.KBHit()
p.control_mode(True)
p.stream_output(True)

print("Recording for {0} seconds ({1} samples)".format(REC_DURATION, numSamples))

for i in range(numSamples):
    if kb.kbhit():
        c = kb.getch()
        if ord(c) == 27:    # ESC key
            quitRec = True
            break

    samples[i] = p.read_state()
    if i % int(round(30 * 1000/CONTROL_PERIOD_MS)) == 0:
        print("{0} seconds".format(i))

print("Finished")
p.close()

if not quitRec:
    np.savetxt(SAVE_FILE, samples, fmt='%d')