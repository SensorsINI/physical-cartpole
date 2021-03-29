# controls cart position with joystick
import time
# gains
KP=1
KD=1
CONTROL_PERIOD_MS = 5

class JoyStickCartController:

    def __init__(self) -> None:
        super().__init__()

    def __str__(self) -> str:
        return super().__str__()

    timeNow = time.time()
    deltaTime = timeNow - lastTime
    if deltaTime == 0:
        deltaTime = 1e-6
    lastTime = timeNow
    elapsedTime = timeNow - startTime
    diffFactor = (CONTROL_PERIOD_MS / (deltaTime * 1000))

    positionTargetNow = POSITION_TARGET

    if timeNow - lastPositionControlTime >= POSITION_CTRL_PERIOD_MS * .001:
        lastPositionControlTime = timeNow
        positionErr = POSITION_SMOOTHING * (position - positionTargetNow) + (1.0 - POSITION_SMOOTHING) * positionErrPrev  # First order low-P=pass filter
        positionErrDiff = (positionErr - positionErrPrev) * diffFactor
        positionErrPrev = positionErr
        # Naive solution: if too positive (too right), move left (minus on positionCmd),
        # but this does not produce correct control.
        # The correct strategy is that if cart is too positive (too right),
        # produce lean to the left by introducing a positive set point angle leaning slightly to left,
        # i.e. more positve positionErr makes more positive effective ANGLE_TARGET
        # End result is that sign of positionCmd is flipped
        # Also, if positionErr is increasing more, then we want even more lean, so D sign is also positive
        positionCmd = +(POSITION_KP * positionErr + POSITION_KD * positionErrDiff)
        #if abs(positionErr) < 1:
         #   positionCmd = 0
