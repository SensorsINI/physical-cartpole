"""

It is a priori not clear for which range of Q to collect the date.
The easiest solution for the first data collection is to MOTOR_DYNAMICS_CORRECTED=False
and sweep step responses for Q in the range -1 to 1 which covers whole safe range of motor inputs.
If you want to test it for a wider range than what is set as -1 to 1 range
you can calculate the Q range as the safe range / factor used to convert Q to motor input.
Notice that by doing it you are neglecting the affine shift
and if you accidentally command command bigger than safe range the system stays safe,
but the callibration script may produce rubbish (TODO)


This script allows you to find right values to calibrate cartpole motors.
To keep our simulation simple we want to model cart acceleration as
dv/dt = a * Q - b * v      (1)
Instead we discover a relationship
dv/dt = a * Q - b * v - c * sign(v) + d(v^n,...) [some other effects visible for big abs(Q)]      (2)
We want to recalibrate motor so that
the system appears as if there would be no c * sign(v) term (sliding friction)
and we map Q to motor input the way that
abs(Q) = 1 corresponds to maximal motor_input for which we can consider contribution of d(v^n,...) negligible.

The relation (1) results in saturation velocity linear in Q: v_sat = (a/b) * Q (3)
The relationship (1) - c * sign(v) results in saturation velocity with additional offset v_sat = (a/b)*Q - (c/b)*sign(v)
The additional terms d(v^n,...) cause deviation from linear relationship v_sat = (a/b)*Q - (c/b)*sign(v) - δ(v^n,...) (4)

We assume that to correct a system described by equation (2) to appear as described by (1)
it is enough to apply corrections making us measure (3) instead of (4).

We measure for small motor input:
v_sat = A * motor_input + B, where B is dependent on direction (B_pos, B_neg),
We want:
v_sat = v_sat_max * Q,
    where Q is in the range -1 to 1 and
    v_sat_max is maximal saturation velocity at each deviation from linear behaviour is negligible

We determine A, B_pos, B_neg by fitting to recorded data of v_sat vs motor_input.
Use bidirectional StepResponseMeasurement in step_response_measure.py to get the data
and double_regression to fit two lines with identical slope but different intercept.
(in fact physics would suggest B_pos = -B_neg
 we allowed B_pos and B_neg to take arbitrary values,
 maybe one should change it in future, more check necessary, what is numerically more convenient)

We determine v_sat_max by eye.

There is a plot which allows you to distinguish:
points taken for linear regression (green)
points which abs < v_sat_max (orange)
all the remaining points (red)

With PLOT_CORRECTED you can set if the data is plotted after or before correcting for sliding friction and
with EVALUATION_SINGLE_FILE you can decide if
you want to plot results from only one file (whichever is uncommented above)
or from both files.

v_sat_max indicates which value should corresponds to Q = 1 and
v_sat_max_lin indicates the range of saturation velocities to be used for linear regression

Finally we determine the relationship motor_input(Q) = (v_sat_max/A) * Q - (B/A) = S * Q + I (S,I for "slope" and "intercept")
and calculate these coefficients.
You have to update these coefficients manually in physical cartpole driver, in get_motor_command()

Attention! a, b, A and B are here are not consistent with the names used in the code. Read carefully!
"""
import matplotlib.pyplot as plt

from double_regression import double_regression
import numpy as np
import pandas as pd
import platform

# Change backend for matplotlib to plot interactively in pycharm

from matplotlib import use

if platform.system() == 'Darwin':
    use('macOSX')
else:
    use('TkAgg')

EVALUATION_SINGLE_FILE = False

# Define the variables
# FILE_NAME = 'Pololu.csv'
FILE_NAME = 'Original.csv'

PLOT_CORRECTED = True

v_sat_max = 0.55
v_sat_max_lin = 0.55


def motor_calibration(FILE_NAME):

    PATH_TO_DATA = 'DataAnalysis/MotorAndCartFriction/'
    file_path = PATH_TO_DATA + FILE_NAME

    # Define functions
    def smooth(y, box_pts):
        box = np.ones(box_pts)/box_pts
        y_smooth = np.convolve(y, box, mode='same')
        return y_smooth

    # Load data

    data: pd.DataFrame = pd.read_csv(file_path, comment='#')

    data['dt'] = data['time'].shift(-1) - data['time']
    data['positionD_last'] = data['positionD'].shift(1)
    data['positionD_smoothed'] = smooth(data['positionD'], 2)
    try:
        data['motor_input'] = data['actualMotorSave'] # Older data sets
    except:
        pass
    data['motor_input'] = -data['motor_input']

    data = data.iloc[1:-1]
    data = data.reset_index(drop=True)


    # Cut data into part when cart accelerates in positive and negative direction

    motor_input_idx_min = data.motor_input.idxmin()
    motor_input_idx_max = data.motor_input.idxmax()

    motor_input_max = data.motor_input.max()
    motor_input_min = data.motor_input.min()

    cutting_index = min(motor_input_idx_min, motor_input_idx_max) + 400
    if cutting_index == motor_input_idx_min + 400:
        starting_direction = 'neg'
    else:
        starting_direction = 'pos'

    if starting_direction == 'pos':
        data_pos = data[:cutting_index]
        data_neg = data[cutting_index:]
    else:
        data_pos = data[cutting_index:]
        data_neg = data[:cutting_index]

    # Double regression

    # Group by Q
    gb_pos = data_pos.groupby(['motor_input'], as_index=False)
    data_stat_pos = gb_pos.size().reset_index(drop=True)
    data_stat_pos['v_max'] = gb_pos['positionD_smoothed'].max()['positionD_smoothed']
    data_stat_pos = data_stat_pos[(data_stat_pos['motor_input'] > 0) & (data_stat_pos['size'] > 40) & (data_stat_pos['v_max'] > 0)]

    gb_neg = data_neg.groupby(['motor_input'], as_index=False)
    data_stat_neg = gb_neg.size().reset_index(drop=True)
    data_stat_neg['v_max'] = gb_neg['positionD_smoothed'].min()['positionD_smoothed']
    data_stat_neg = data_stat_neg[(data_stat_neg['motor_input'] < 0) & (data_stat_neg['size'] > 40) & (data_stat_neg['v_max'] < 0)]

    motor_input_pos = data_stat_pos['motor_input'].to_numpy()
    v_sat_pos = data_stat_pos['v_max'].to_numpy()


    motor_input_neg = data_stat_neg['motor_input'].to_numpy()
    v_sat_neg = data_stat_neg['v_max'].to_numpy()

    # Take the linear part for regression
    data_stat_pos_lin = data_stat_pos[data_stat_pos['v_max'].abs() <= v_sat_max_lin]
    data_stat_neg_lin = data_stat_neg[data_stat_neg['v_max'].abs() <= v_sat_max_lin]


    # Get relevant variables for regression
    motor_input_pos_lin = data_stat_pos_lin['motor_input'].to_numpy()
    v_sat_pos_lin = data_stat_pos_lin['v_max'].to_numpy()
    motor_input_neg_lin = data_stat_neg_lin['motor_input'].to_numpy()
    v_sat_neg_lin = data_stat_neg_lin['v_max'].to_numpy()

    a, B_pos, B_neg = double_regression(motor_input_pos_lin, v_sat_pos_lin, motor_input_neg_lin, v_sat_neg_lin)

    print()
    print()
    print('**************************')
    print(FILE_NAME)
    print('---------------')
    print('Parameters obtained from regression')
    print('v_sat = a * motor_input + b')
    print('a     = {}'.format(a))
    print('B_pos = {}'.format(B_pos))
    print('B_neg = {}'.format(B_neg))
    print('x-intercept-pos = {}'.format(-B_pos/a))
    print('x-intercept-neg = {}'.format(-B_neg/a))
    print('')
    print('M = S * Q + B')
    S = v_sat_max/a
    I_pos = -B_pos/a
    I_neg = -B_neg/a
    print('S     = {}'.format(S))
    print('I_pos = {}'.format(I_pos))
    print('I_neg = {}'.format(I_neg))

    print('**************************')

    # Plot v_max vs. motor_input after shifting

    # Take the data you want to use as max range:
    # Take the linear part for regression
    data_stat_pos_max = data_stat_pos[data_stat_pos['v_max'].abs() <= v_sat_max]
    data_stat_neg_max = data_stat_neg[data_stat_neg['v_max'].abs() <= v_sat_max]

    motor_input_pos_max = data_stat_pos_max['motor_input'].to_numpy()
    v_sat_pos_max = data_stat_pos_max['v_max'].to_numpy()
    motor_input_neg_max = data_stat_neg_max['motor_input'].to_numpy()
    v_sat_neg_max = data_stat_neg_max['v_max'].to_numpy()

    # Plot
    if not PLOT_CORRECTED:
        I_neg = 0
        I_pos = 0

    fig, axes = plt.subplots()

    axes.set_title(FILE_NAME)

    motor_input_axis = np.linspace(motor_input_min, motor_input_max, 100)
    v_sat_axis = a*motor_input_axis
    p = axes.plot(motor_input_axis, v_sat_axis, color='blue')

    sn = axes.scatter(motor_input_neg-I_neg, v_sat_neg, color='red')
    axes.scatter(motor_input_neg_max-I_neg, v_sat_neg_max, color='orange')
    axes.scatter(motor_input_neg_lin-I_neg, v_sat_neg_lin, color='green')
    sp = axes.scatter(motor_input_pos-I_pos, v_sat_pos, color='red')
    axes.scatter(motor_input_pos_max-I_pos, v_sat_pos_max, color='orange')
    axes.scatter(motor_input_pos_lin-I_pos, v_sat_pos_lin, color='green')

    plt.show()

    return p[0], sn, sp

if __name__ == '__main__':

    if EVALUATION_SINGLE_FILE:
        motor_calibration(FILE_NAME)
    else:
        FILE_NAME = 'Original.csv'
        p_org, sn_org, sp_org = motor_calibration(FILE_NAME)
        FILE_NAME = 'Pololu.csv'
        p_pol, sn_pol, sp_pol = motor_calibration(FILE_NAME)

        figure, ax = plt.subplots()
        p_org, = ax.plot(p_org.get_data()[0], p_org.get_data()[1], color='lightgreen')
        sn_data = sn_org.get_offsets().data
        sp_data = sp_org.get_offsets().data
        sn_org = ax.scatter(sn_data[:, 0], sn_data[:, 1], color='green')
        sp_org = ax.scatter(sp_data[:, 0], sp_data[:, 1], color='green')

        p_pol, = ax.plot(p_pol.get_data()[0], p_pol.get_data()[1], color='lightblue')
        sn_data = sn_pol.get_offsets().data
        sp_data = sp_pol.get_offsets().data
        sn_pol = ax.scatter(sn_data[:, 0], sn_data[:, 1], color='blue')
        sp_pol = ax.scatter(sp_data[:, 0], sp_data[:, 1], color='blue')

        plt.show()




