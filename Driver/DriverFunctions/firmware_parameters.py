# TODO: These default values should be set the way cartpole firmware controller is working
#   @Asude, I'm sorry, I've lost the values you calculated...
ANGLE_TARGET_FIRMWARE_DEFAULT = 3059
ANGLE_AVG_LENGTH_FIRMWARE_DEFAULT = 10
ANGLE_SMOOTHING_FIRMWARE_DEFAULT = 1
ANGLE_KP_FIRMWARE_DEFAULT = 400
ANGLE_KD_FIRMWARE_DEFAULT = 400

POSITION_TARGET_FIRMWARE_DEFAULT = 0
POSITION_CTRL_PERIOD_MS_FIRMWARE_DEFAULT = 5
POSITION_SMOOTHING_FIRMWARE_DEFAULT = 1
POSITION_KP_FIRMWARE_DEFAULT = 20
POSITION_KD_FIRMWARE_DEFAULT = 300


def set_firmware_parameters(CartPoleInstance, **kwargs):
    """
    This function sets the firmware parameters, It requires CartPoleInstance and accepts optionally ANGLE_AVG_LENGTH
    If provided ANGLE_AVG_LENGTH is set from provided value, otherwise from the default list on top of this file.
    Other parameters are now set only from the list on top of this file.
    """
    if 'ANGLE_AVG_LENGTH' in kwargs:
        ANGLE_AVG_LENGTH_FIRMWARE = kwargs.get('ANGLE_AVG_LENGTH')
    else:
        ANGLE_AVG_LENGTH_FIRMWARE = ANGLE_AVG_LENGTH_FIRMWARE_DEFAULT

    ANGLE_TARGET_FIRMWARE, ANGLE_SMOOTHING_FIRMWARE, ANGLE_KP_FIRMWARE, ANGLE_KD_FIRMWARE = \
        ANGLE_TARGET_FIRMWARE_DEFAULT, ANGLE_SMOOTHING_FIRMWARE_DEFAULT, ANGLE_KP_FIRMWARE_DEFAULT, ANGLE_KD_FIRMWARE_DEFAULT

    POSITION_TARGET_FIRMWARE, POSITION_CTRL_PERIOD_MS_FIRMWARE, POSITION_SMOOTHING_FIRMWARE, POSITION_KP_FIRMWARE, POSITION_KD_FIRMWARE = \
        POSITION_TARGET_FIRMWARE_DEFAULT, POSITION_CTRL_PERIOD_MS_FIRMWARE_DEFAULT, POSITION_SMOOTHING_FIRMWARE_DEFAULT, POSITION_KP_FIRMWARE_DEFAULT, POSITION_KD_FIRMWARE_DEFAULT

    CartPoleInstance.set_angle_config(ANGLE_TARGET_FIRMWARE, # This must take care of both: target angle and 0-point offset
                                      ANGLE_AVG_LENGTH_FIRMWARE,
                                      ANGLE_SMOOTHING_FIRMWARE,
                                      ANGLE_KP_FIRMWARE,
                                      ANGLE_KD_FIRMWARE,
                                      )

    CartPoleInstance.set_position_config(POSITION_TARGET_FIRMWARE,
                                         POSITION_CTRL_PERIOD_MS_FIRMWARE,
                                         POSITION_SMOOTHING_FIRMWARE,
                                         POSITION_KP_FIRMWARE,
                                         POSITION_KD_FIRMWARE)


def get_firmware_parameters(CartPoleInstance):
    # Why is it getting parameters? To enable checking if they have been correctly written
    # setPoint, avgLen, smoothing, KP, KD
    (ANGLE_TARGET,  # This must take care of both: target angle and 0-point offset
     ANGLE_AVG_LENGTH,
     ANGLE_SMOOTHING,
     ANGLE_KP,
     ANGLE_KD) = CartPoleInstance.get_angle_config()

    (POSITION_TARGET,
     POSITION_CTRL_PERIOD_MS,
     POSITION_SMOOTHING,
     POSITION_KP,
     POSITION_KD) = CartPoleInstance.get_position_config()

    list_of_obtained_parameters = (ANGLE_TARGET, ANGLE_AVG_LENGTH, ANGLE_SMOOTHING, ANGLE_KP, ANGLE_KD,
                                   POSITION_TARGET, POSITION_CTRL_PERIOD_MS, POSITION_SMOOTHING, POSITION_KP, POSITION_KD
                                   )

    return list_of_obtained_parameters
