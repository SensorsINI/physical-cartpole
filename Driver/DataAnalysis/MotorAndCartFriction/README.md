# Motor Calibration and Cart Friction

The script MotorCalibration.py allows for
motor calibration so that cart appears
to have no sliding friction
and the relationship between acceleration and normed input is linear up to rolling friction: \
dv/dt = a * Q - b * v \
It uses actual motor input (motor_input) for its calculations. \
You need to update the calculated values manually in the driver of physical cartpole!

The script CheckMotorCalibration.py
let you check if the calibration done with MotorCalibration.py
brought intended results.
It uses normed control input Q.

GetForceAndFriction.py helps to determine the values related to motor -
parameter mapping from normed motor input Q to physical force acting on the cart
and parameter of rolling friction of the cart - needed for the simulation.

All above files require a measurement of cart accelerating in both directions
with piecewise constant motor command (to achieve saturation velocity).
Use bidirectional step_response_experiment in measure.py to get it.

As a matter of example we provide two csv recordings - 
Original.csv and Pololu.csv -
taken with cartpoles we work while creating this files.
With these files the scripts above are tested to work without known problems.
Replace them with a measurement taken with your cartpole.

## Canonical procedure for neural control (force-fit)

For the neural-imitator controller the canonical calibration is the **force-based** fit,
because it anchors Q=1 to the force scale the network was trained with (u_max_target):

1. Record a fresh bidirectional step response (step_response_experiment) and save it as
   `CPP_step_response.csv` in this folder (the committed, reproducible recording).
2. Run: `python MotorCalibration.py --force-fit`
   (uses u_max_target and effective_mass set near the top of MotorCalibration.py;
   PWM must match the chip: 10000 Zynq / 7200 STM).
3. Paste the printed `MOTOR_CORRECTION` tuple into BOTH:
   - `Driver/globals.py` → `MOTOR_CORRECTION_POLOLU` (PC neural control), and
   - `Firmware/Src/CartPoleFirmware/parameters.c` → `MOTOR_CORRECTION` (on-chip; requires reflash).
4. The force-fit also prints a linear-range check; `Q=±1` should report `OK`.

After a motor swap, repeat steps 1–3 to get values that make the network work again.
`effective_mass` is the cart+pole+drivetrain effective mass assumption; keep it consistent
with the simulation/training, since it sets the absolute gain.

double_regression.py is a helper file used by MotorCalibration.py.
It is a linear regression algorithm which fits two lines to two datasets,
while imposing that the two lines are required to have the same slope; 
they are only allowed to have different intercept.
The proposed algorithm finds two such lines
which _together_ minimize the _total_ MSE of _both_ datasets.
This kind of regression is needed while fitting the lines to 
data containing sliding friction, which is the case for our cartpoles.