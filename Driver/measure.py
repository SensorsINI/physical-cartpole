# measurements from cartpole, controlled by state machine.
# control.py calls update_state() if state is not 'idle'

from .pendulum import Pendulum

STARTING_POSITION = -2000  # cart starting position
ENDING_POSITION = 1000  # position to turn off motor
RESET_SPEED = 1500
SPEED_STEP=500
STARTING_SPEED=1000
ENDING_SPEED=8000

class StepResponseMeasurement:
    def __init__(self):
        self.state='idle'
        self.speed=RESET_SPEED
        self.motor=0

    def start(self):
        self.state='start'
        self.motor=0

    def stop(self):
        self.motor = 0
        self.state='idle'

    def get_state(self):
        return self.state

    def is_idle(self):
        return self.state == 'idle'

    def update_state(self, angle:int, position:int, time:float):
        if self.state=='idle':
            return
        elif self.state == 'start':
            self.speed = RESET_SPEED
            self.state = 'resetting'
        elif self.state=='resetting':
            if(position>STARTING_POSITION):
                self.motor=-RESET_SPEED
            else:
                self.motor=0
                self.speed=STARTING_SPEED
                self.state='starting_step'
        elif self.state=='starting_step':
            if(self.speed<ENDING_SPEED):
                self.speed+=SPEED_STEP
                self.motor=self.speed
                self.state='moving'
            else:
                self.speed=0
                self.motor=(0)
                self.state='idle'
        elif self.state=='moving':
            if position>ENDING_POSITION:
                self.motor=0
                self.state='resetting'

    def __str__(self):
        return f'{self.state}:{self.speed}:{self.motor}'


