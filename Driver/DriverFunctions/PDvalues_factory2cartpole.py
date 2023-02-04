from Driver.globals import *
import json

writetofile = '../Json/control-factory.json'

B_KP = 400/POSITION_NORMALIZATION_FACTOR/MOTOR_FULL_SCALE
B_KD = 400/POSITION_NORMALIZATION_FACTOR/MOTOR_FULL_SCALE
P_KP = 20/ANGLE_NORMALIZATION_FACTOR/MOTOR_FULL_SCALE
P_KD = 300/ANGLE_NORMALIZATION_FACTOR/MOTOR_FULL_SCALE


data = {
    "ANGLE_TARGET": 0,
    "ANGLE_KP": B_KP,
    "ANGLE_KI": 0,
    "ANGLE_KD": B_KD,
    "POSITION_KP": P_KP,
    "POSITION_KI": 0,
    "POSITION_KD": P_KD,
    "ANGLE_SMOOTHING": 1.0,
    "POSITION_SMOOTHING": 0.2
}

print(data.keys())
with open(writetofile, 'w') as outfile:
    json.dump(data, outfile)

