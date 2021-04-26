
ANGLE_AVG_LENGTH = 10  # adc routine in firmware reads ADC this many times quickly in succession to reduce noise


POSITION_FULL_SCALE=2047. # cart position should range over +- this value if calibrated for zero at center
POSITION_NORMALIZATION = 4660 # This is an empirical approximation
POSITION_FULL_SCALE_N = int(POSITION_NORMALIZATION)/2 # Corrected position full scale
TRACK_LENGTH = 0.396 # Total usable track length in meters

ratio = 1.05
# todo round or display it with 10th power

def inc(param):
    if param < 2:
        param = round(param + 0.1, 1)
    else:
        old = param
        param = round(param * ratio)
        if param == old:
            param += 1
    return param


def dec(param):
    if param < 2:
        param = max(0, round(param - 0.1, 1))
    else:
        old = param
        param = round(param / ratio)
        if param == old:
            param -= 1
    return param