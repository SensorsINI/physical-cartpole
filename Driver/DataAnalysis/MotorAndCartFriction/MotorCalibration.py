"""
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
Use bidirectional step_response_experiment in step_response_experiment.py to get the data
and double_regression to fit two lines with identical slope but different intercept.
(in fact physics would suggest B_pos = -B_neg
 we allowed B_pos and B_neg to take arbitrary values,
 maybe one should change it in future,
 but the results suggest much better fit if we allow them to be different,
 maybe this is some asymmetry due to construction of the motor?)

We determine v_sat_max by eye.

There is a plot which allows you to distinguish:
points taken for linear regression (green)
points which abs(v_sat) < v_sat_max (orange) - this is a range in which motor will operate: Q in the range [-1, 1]
all the remaining points (red)

With PLOT_CORRECTED you can set if the data is plotted after or before correcting for sliding friction and
with EVALUATION_SINGLE_FILE you can decide if
you want to plot results from only one file (whichever is uncommented above)
or from both files.

v_sat_max indicates which value should corresponds to Q = 1 and
v_sat_max_lin indicates the range of saturation velocities to be used for linear regression

Finally we determine the relationship motor_input(Q) = (v_sat_max/A) * Q - (B/A) = S * Q + I (S,I for "slope" and "intercept")
and calculate these coefficients.
You have to update these coefficients manually in physical cartpole driver,
in globals.py for control from PC and in firmware for control from STM or Zynq.
Refer to physical cartpole readme for more details.

Attention! a, b, A and B are here unfortunately not consistent with the names used in the code. Read carefully!
"""
import matplotlib.pyplot as plt

from double_regression import double_regression, double_regression_2
from pathlib import Path
import numpy as np
import pandas as pd
import platform
import argparse
import os

# Change backend for matplotlib to plot interactively in pycharm

from matplotlib import use

if platform.system() == 'Darwin':
    use('macOSX')
elif os.environ.get('DISPLAY'):
    use('TkAgg')
else:
    use('Agg')

DATA_SMOOTHING = 2  # May strongly influence what is the max velocity and hence the results of calibration

EVALUATION_SINGLE_FILE = True

MOTOR_PWM_PERIOD_IN_CLOCK_CYCLES = 10000  # 10000 Zynq, 7200 STM

# Define the variables
# FILE_NAME = 'Pololu.csv'
FILE_NAME = 'CPP_step_response.csv'
DEFAULT_DATA_DIR = Path(__file__).resolve().parents[2] / 'ExperimentRecordings'

PLOT_CORRECTED = True

v_sat_max = 0.55  # Velocity target for Q = 1 in this fit; this does not set the physical force u_max.
v_sat_max_lin = 0.85  # 2024 paper setup threshold selecting points used for the linear v_sat-vs-motor_input fit.
u_max_target = 2.62  # Force target in newtons for Q = 1 in --force-fit mode; matches the active paper network scale.
effective_mass = 0.230 + 0.087  # Effective moving mass [kg] (cart + pole). Approx: ignores belt/bearing inertia and wire; keep consistent with the simulation, it sets the absolute gain.


def motor_calibration(FILE_NAME):

    file_path = Path(FILE_NAME)
    if not file_path.is_absolute() and not file_path.exists():
        file_path = DEFAULT_DATA_DIR / file_path

    # Define functions
    def smooth(y, box_pts):
        box = np.ones(box_pts)/box_pts
        y_smooth = np.convolve(y, box, mode='same')
        return y_smooth

    # Load data

    data: pd.DataFrame = pd.read_csv(file_path, comment='#')

    data['dt'] = data['time'].shift(-1) - data['time']
    data['positionD_last'] = data['positionD'].shift(1)
    data['positionD_smoothed'] = smooth(data['positionD'], DATA_SMOOTHING)
    try:
        data['motor_input'] = data['actualMotorSave'] # Older data sets
    except:
        pass

    data = data.iloc[1:-1]
    data = data.reset_index(drop=True)

    motor_input_max = data.motor_input.max()
    motor_input_min = data.motor_input.min()

    data = data.loc[data['measurement'].str.contains('State:moving')]
    data = data.loc[np.logical_and(data['motor_input'] != 0, np.sign(data['motor_input']) == np.sign(data['positionD_last']))]

    data_pos = data.loc[data['motor_input'] > 0]
    data_neg = data.loc[data['motor_input'] < 0]

    # Double regression

    # Group by Q
    gb_pos = data_pos.groupby(['motor_input'], as_index=False)
    data_stat_pos = gb_pos.size().reset_index(drop=True)
    data_stat_pos['v_max'] = gb_pos['positionD_smoothed'].max()['positionD_smoothed']

    gb_neg = data_neg.groupby(['motor_input'], as_index=False)
    data_stat_neg = gb_neg.size().reset_index(drop=True)
    data_stat_neg['v_max'] = gb_neg['positionD_smoothed'].min()['positionD_smoothed']

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

    print('Motor correction give MOTOR_PWM_PERIOD_IN_CLOCK_CYCLES = {}:'.format(MOTOR_PWM_PERIOD_IN_CLOCK_CYCLES))
    S_normed = S / MOTOR_PWM_PERIOD_IN_CLOCK_CYCLES
    I_pos_normed = I_pos / MOTOR_PWM_PERIOD_IN_CLOCK_CYCLES
    I_neg_normed = I_neg / MOTOR_PWM_PERIOD_IN_CLOCK_CYCLES
    print('MOTOR_CORRECTION = ({:.7f}, {:.7f}, {:.7f})'.format(S_normed, I_pos_normed, -I_neg_normed))

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


def _fit_force_model(file_name):
    """Load a step-response recording and fit positionDD = A*motor_input + D*positionD + C_direction.

    Returns the motor slope A (motor_to_acceleration, [m/s^2 per motor_input unit]),
    the velocity damping D [1/s], the directional stiction offsets, the cleaned
    data and the linear-range limits. Shared by the single-run force calibration
    and the two-run mass identification so both use an identical fit.
    """
    file_path = Path(file_name)
    if not file_path.is_absolute() and not file_path.exists():
        file_path = DEFAULT_DATA_DIR / file_path

    def smooth(y, box_pts):
        box = np.ones(box_pts) / box_pts
        return np.convolve(y, box, mode='same')

    data: pd.DataFrame = pd.read_csv(file_path, comment='#')
    data['dt'] = data['time'].shift(-1) - data['time']
    data['positionD_last'] = data['positionD'].shift(1)
    data['positionD_smoothed'] = smooth(data['positionD'], DATA_SMOOTHING)
    data['positionDD'] = data['positionD_smoothed'].diff() / data['dt'].shift(1)
    if 'motor_input' not in data.columns:
        data['motor_input'] = data['actualMotorSave']

    data = data.iloc[2:-1].reset_index(drop=True)
    data = data.loc[data['measurement'].str.contains('State:moving')]
    data = data.loc[np.logical_and(data['motor_input'] != 0, np.sign(data['motor_input']) == np.sign(data['positionD_last']))]
    data = data.replace([np.inf, -np.inf], np.nan).dropna(
        subset=['motor_input', 'positionD_smoothed', 'positionDD']
    )

    data_pos = data.loc[data['motor_input'] > 0]
    data_neg = data.loc[data['motor_input'] < 0]

    gb_pos = data_pos.groupby(['motor_input'], as_index=False)
    data_stat_pos = gb_pos.size().reset_index(drop=True)
    data_stat_pos['v_max'] = gb_pos['positionD_smoothed'].max()['positionD_smoothed']

    gb_neg = data_neg.groupby(['motor_input'], as_index=False)
    data_stat_neg = gb_neg.size().reset_index(drop=True)
    data_stat_neg['v_max'] = gb_neg['positionD_smoothed'].min()['positionD_smoothed']

    data_stat_pos_lin = data_stat_pos[data_stat_pos['v_max'].abs() <= v_sat_max_lin]
    data_stat_neg_lin = data_stat_neg[data_stat_neg['v_max'].abs() <= v_sat_max_lin]
    motor_inputs_linear = set(data_stat_pos_lin['motor_input']).union(data_stat_neg_lin['motor_input'])
    data_linear = data[data['motor_input'].isin(motor_inputs_linear)]

    direction_pos = (data_linear['motor_input'] > 0).astype(float).to_numpy()
    direction_neg = (data_linear['motor_input'] < 0).astype(float).to_numpy()
    design = np.column_stack(
        (
            data_linear['motor_input'].to_numpy(),
            data_linear['positionD_smoothed'].to_numpy(),
            direction_pos,
            direction_neg,
        )
    )
    target = data_linear['positionDD'].to_numpy()
    motor_to_acceleration, damping, offset_pos, offset_neg = np.linalg.lstsq(design, target, rcond=None)[0]

    return {
        'file_path': file_path,
        'motor_to_acceleration': motor_to_acceleration,
        'damping': damping,
        'offset_pos': offset_pos,
        'offset_neg': offset_neg,
        'data': data,
        'data_linear': data_linear,
        'pos_linear_limit': data_stat_pos_lin['motor_input'].max(),
        'neg_linear_limit': data_stat_neg_lin['motor_input'].min(),
        'n_linear': len(data_linear),
    }


def motor_force_calibration(FILE_NAME):
    fit = _fit_force_model(FILE_NAME)
    file_path = fit['file_path']
    data = fit['data']
    data_linear = fit['data_linear']
    motor_to_acceleration = fit['motor_to_acceleration']
    damping = fit['damping']
    offset_pos = fit['offset_pos']
    offset_neg = fit['offset_neg']

    target_acceleration = u_max_target / effective_mass
    S = target_acceleration / motor_to_acceleration
    I_pos = -offset_pos / motor_to_acceleration
    I_neg = -offset_neg / motor_to_acceleration

    S_normed = S / MOTOR_PWM_PERIOD_IN_CLOCK_CYCLES
    I_pos_normed = I_pos / MOTOR_PWM_PERIOD_IN_CLOCK_CYCLES
    I_neg_normed = I_neg / MOTOR_PWM_PERIOD_IN_CLOCK_CYCLES

    motor_input_at_q_pos = S + I_pos
    motor_input_at_q_neg = -S + I_neg
    pos_linear_limit = fit['pos_linear_limit']
    neg_linear_limit = fit['neg_linear_limit']
    pos_in_linear_range = motor_input_at_q_pos <= pos_linear_limit
    neg_in_linear_range = motor_input_at_q_neg >= neg_linear_limit

    print()
    print()
    print('**************************')
    print(FILE_NAME)
    print('---------------')
    print('Force-based motor calibration')
    print('positionDD = A * motor_input + D * positionD + C_direction')
    print('A     = {}'.format(motor_to_acceleration))
    print('D     = {}'.format(damping))
    print('C_pos = {}'.format(offset_pos))
    print('C_neg = {}'.format(offset_neg))
    print('')
    print('Target force calibration')
    print('u_max_target = {} N'.format(u_max_target))
    print('effective_mass = {} kg'.format(effective_mass))
    print('target_acceleration = u_max_target / effective_mass = {} m/s^2'.format(target_acceleration))
    print('')
    print('M = S * Q + I_direction')
    print('S     = {}'.format(S))
    print('I_pos = {}'.format(I_pos))
    print('I_neg = {}'.format(I_neg))
    print('Motor correction give MOTOR_PWM_PERIOD_IN_CLOCK_CYCLES = {}:'.format(MOTOR_PWM_PERIOD_IN_CLOCK_CYCLES))
    print('MOTOR_CORRECTION = ({:.7f}, {:.7f}, {:.7f})'.format(S_normed, I_pos_normed, -I_neg_normed))
    print('')
    print('Linear-range check based on v_sat_max_lin = {}'.format(v_sat_max_lin))
    print('positive linear motor_input max = {}'.format(pos_linear_limit))
    print('negative linear motor_input min = {}'.format(neg_linear_limit))
    print('motor_input at Q=+1 = {} -> {}'.format(
        motor_input_at_q_pos, 'OK' if pos_in_linear_range else 'OUTSIDE LINEAR RANGE'))
    print('motor_input at Q=-1 = {} -> {}'.format(
        motor_input_at_q_neg, 'OK' if neg_in_linear_range else 'OUTSIDE LINEAR RANGE'))
    print('**************************')

    fig, axes = plt.subplots()
    axes.set_title(str(file_path))
    axes.scatter(data['motor_input'], data['positionDD'], color='red', s=8, label='all moving samples')
    axes.scatter(data_linear['motor_input'], data_linear['positionDD'], color='green', s=8, label='linear-range samples')
    motor_input_axis = np.linspace(data['motor_input'].min(), data['motor_input'].max(), 100)
    axes.plot(motor_input_axis, motor_to_acceleration * motor_input_axis, color='blue', label='force-fit motor slope')
    axes.set_xlabel('motor_input')
    axes.set_ylabel('positionDD [m/s^2]')
    axes.legend()
    plt.show()


def _direction_slope(data_linear, positive):
    """Fit positionDD = A*motor_input + D*positionD + C on one push direction only.

    Returns (A, D, n_samples). Used so a direction contaminated by manual help
    (e.g. pushing the weight back while retracting) can be excluded.
    """
    if positive:
        sub = data_linear[data_linear['motor_input'] > 0]
    else:
        sub = data_linear[data_linear['motor_input'] < 0]
    n = len(sub)
    if n < 3:
        return None, None, n
    design = np.column_stack(
        (
            sub['motor_input'].to_numpy(),
            sub['positionD_smoothed'].to_numpy(),
            np.ones(n),
        )
    )
    target = sub['positionDD'].to_numpy()
    A, D, _C = np.linalg.lstsq(design, target, rcond=None)[0]
    return A, D, n


def mass_identification(baseline_file, loaded_file, delta_mass, q1_gain=None, direction='pos'):
    """Identify the effective cart mass from two step-response runs that differ
    only by a known added mass (pole removed in both, identical drive).

    For a fixed motor_input the motor force k is the same in both runs, so the
    measured slope A = k / M_eff. With A1 (baseline) and A2 (extra mass delta_mass):
        A1 / A2 = (M_eff + delta_mass) / M_eff  =>  M_eff = delta_mass / (A1/A2 - 1)
    The reflected drivetrain inertia (rotor + gears + belt) is contained in M_eff
    in both runs and so is captured automatically. From M_eff the absolute force
    scale follows: k = A1 * M_eff [N per motor_input unit].

    Slopes are computed per push direction; `direction` ('pos'/'neg') selects which
    one drives the reported result, so a manually-contaminated direction can be
    excluded. The other direction is still printed as a cross-check.
    """
    fit_base = _fit_force_model(baseline_file)
    fit_load = _fit_force_model(loaded_file)

    A1_pos, D1_pos, n1_pos = _direction_slope(fit_base['data_linear'], True)
    A1_neg, D1_neg, n1_neg = _direction_slope(fit_base['data_linear'], False)
    A2_pos, D2_pos, n2_pos = _direction_slope(fit_load['data_linear'], True)
    A2_neg, D2_neg, n2_neg = _direction_slope(fit_load['data_linear'], False)

    def solve(A1, A2):
        if A1 is None or A2 is None or A2 == 0 or not (A1 / A2 > 1.0):
            return None
        return delta_mass / (A1 / A2 - 1.0)

    M_pos = solve(A1_pos, A2_pos)
    M_neg = solve(A1_neg, A2_neg)

    def fmt(x, unit=''):
        return 'n/a' if x is None else '{:.4f}{}'.format(x, unit)

    print()
    print()
    print('**************************')
    print('Effective-mass identification (two-run, known added mass)')
    print('---------------')
    print('baseline : {}  (linear samples: {})'.format(fit_base['file_path'], fit_base['n_linear']))
    print('loaded   : {}  (linear samples: {})'.format(fit_load['file_path'], fit_load['n_linear']))
    print('delta_mass = {} kg'.format(delta_mass))
    print('reported direction = {} (other direction shown as cross-check only)'.format(direction))
    print('')
    print('Per-direction slopes A [m/s^2 per motor_input] and resulting M_eff:')
    print('  forward (+): A1={}, A2={}, A1/A2={}, M_eff={}  (n1={}, n2={})'.format(
        fmt(A1_pos), fmt(A2_pos),
        fmt(A1_pos / A2_pos) if (A1_pos and A2_pos) else 'n/a', fmt(M_pos, ' kg'), n1_pos, n2_pos))
    print('  retract (-): A1={}, A2={}, A1/A2={}, M_eff={}  (n1={}, n2={})'.format(
        fmt(A1_neg), fmt(A2_neg),
        fmt(A1_neg / A2_neg) if (A1_neg and A2_neg) else 'n/a', fmt(M_neg, ' kg'), n1_neg, n2_neg))

    if direction == 'pos':
        A1, D1, M_eff = A1_pos, D1_pos, M_pos
    else:
        A1, D1, M_eff = A1_neg, D1_neg, M_neg

    if M_eff is None:
        print('')
        print('WARNING: cannot solve for the reported direction ({}). Expected A1/A2 > 1'.format(direction))
        print('(more mass -> smaller acceleration slope). Check the runs and try the other')
        print('direction with --direction. No mass reported.')
        print('**************************')
        return None

    k = A1 * M_eff  # N per motor_input unit
    F_full = k * MOTOR_PWM_PERIOD_IN_CLOCK_CYCLES  # N at motor_input = full PWM period
    M_fric_identified = -D1 * M_eff if D1 is not None else float('nan')  # N/(m/s); D = -M_fric/M_eff

    print('')
    print('Reported effective CART mass (no pole, no hinge; incl. drivetrain inertia):')
    print('M_eff = delta_mass / (A1/A2 - 1) = {:.4f} kg'.format(M_eff))
    print('  bare cart in config = 0.230 kg; difference is reflected belt/bearing/rotor inertia')
    print('  for operation add the hinge that rides with the cart: m_cart_model = M_eff + m_hinge')
    print('')
    print('Absolute force scale (direction {}):'.format(direction))
    print('k      = A1 * M_eff       = {:.6f} N per motor_input unit'.format(k))
    print('F_full = k * PWM_period   = {:.4f} N at full motor_input ({} units)'.format(
        F_full, MOTOR_PWM_PERIOD_IN_CLOCK_CYCLES))
    print('')
    print('Identified viscous friction (from {} damping D = {:.4f} 1/s):'.format(direction, D1))
    print('M_fric = -D * M_eff       = {:.4f} N/(m/s)   (model yml uses 3.22)'.format(M_fric_identified))

    if q1_gain is not None:
        u_max_q1 = k * q1_gain * MOTOR_PWM_PERIOD_IN_CLOCK_CYCLES
        print('')
        print('True force at Q=1 for MOTOR_CORRECTION gain S = {} :'.format(q1_gain))
        print('u_max(Q=1) = k * S * PWM_period = {:.4f} N'.format(u_max_q1))
        print('  -> set this as u_max in cartpole_physical_parameters.yml for an honest model')

    print('**************************')

    fig, axes = plt.subplots()
    axes.set_title('Effective-mass identification: slope change vs added mass (direction={})'.format(direction))
    axes.scatter(fit_base['data']['motor_input'], fit_base['data']['positionDD'],
                 color='tab:blue', s=6, alpha=0.4, label='baseline samples')
    axes.scatter(fit_load['data']['motor_input'], fit_load['data']['positionDD'],
                 color='tab:orange', s=6, alpha=0.4, label='loaded (+{} kg) samples'.format(delta_mass))
    motor_input_axis = np.linspace(
        min(fit_base['data']['motor_input'].min(), fit_load['data']['motor_input'].min()),
        max(fit_base['data']['motor_input'].max(), fit_load['data']['motor_input'].max()),
        100,
    )
    if A1 is not None:
        axes.plot(motor_input_axis, A1 * motor_input_axis, color='tab:blue', label='baseline fit (A1)')
    if A2_pos is not None and direction == 'pos':
        axes.plot(motor_input_axis, A2_pos * motor_input_axis, color='tab:orange', label='loaded fit (A2)')
    if A2_neg is not None and direction == 'neg':
        axes.plot(motor_input_axis, A2_neg * motor_input_axis, color='tab:orange', label='loaded fit (A2)')
    axes.set_xlabel('motor_input')
    axes.set_ylabel('positionDD [m/s^2]')
    axes.legend()
    plt.show()

    return M_eff


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Calculate motor correction from a bidirectional step-response recording.'
    )
    parser.add_argument(
        'file_name',
        nargs='?',
        default=FILE_NAME,
        help=f'CSV file path. Relative names default to {DEFAULT_DATA_DIR}.',
    )
    parser.add_argument(
        '--force-fit',
        action='store_true',
        help='Fit MOTOR_CORRECTION so Q=1 maps to u_max_target for cart+pole mass, and check linear range.',
    )
    parser.add_argument(
        '--mass-id',
        nargs=2,
        metavar=('BASELINE_CSV', 'LOADED_CSV'),
        help='Two step-response recordings (pole removed, identical drive); the second has a known '
             'extra mass. Identifies the effective cart mass (incl. drivetrain). Requires --delta-mass.',
    )
    parser.add_argument(
        '--delta-mass',
        type=float,
        default=None,
        help='Known added mass [kg] between the two --mass-id runs (e.g. 0.227).',
    )
    parser.add_argument(
        '--q1-gain',
        type=float,
        default=None,
        help='Optional MOTOR_CORRECTION normalized gain S; with --mass-id, report true u_max at Q=1.',
    )
    parser.add_argument(
        '--direction',
        choices=['pos', 'neg'],
        default='pos',
        help="Push direction used for the reported mass with --mass-id ('pos' = forward). "
             'Use this to exclude a direction contaminated by manual help.',
    )
    args = parser.parse_args()

    if args.mass_id:
        if args.delta_mass is None:
            parser.error('--mass-id requires --delta-mass (known added mass in kg)')
        mass_identification(args.mass_id[0], args.mass_id[1], args.delta_mass, args.q1_gain, args.direction)
    elif EVALUATION_SINGLE_FILE:
        if args.force_fit:
            motor_force_calibration(args.file_name)
        else:
            motor_calibration(args.file_name)
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




