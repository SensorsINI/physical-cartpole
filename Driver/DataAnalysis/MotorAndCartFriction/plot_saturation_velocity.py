"""Saturation-velocity vs Q from a bidirectional STEP-RESPONSE recording.

This reproduces the original MotorCalibration.py view of the motor: for each constant
motor_input (Q step) the cart accelerates until force balances damping, so the TERMINAL
velocity reached during that step IS the saturation velocity v_sat. We group the recording
by motor_input and take the max (positive push) / min (negative push) velocity -- exactly
as MotorCalibration.py does -- then plot v_sat against the EQUIVALENT Q so it lines up with
the EMF-identification figure (estimate_emf.py).

Why this is the calibration's own linearity test:
    relation (1)  dv/dt = a*Q - b*v            -> v_sat = (a/b) Q                (linear)
    with friction dv/dt = a*Q - b*v - c*sign(v)-> v_sat = (a/b) Q - (c/b) sign(v)
    with back-EMF / high-Q effects             -> v_sat bends below that line    (saturation)
So a straight v_sat-vs-Q line means linear; the point where the data peels below the fitted
line is where back-EMF / saturation begins. We check that Q=1 sits on the straight part.

Equivalent Q = motor_input / (gain * PWM_PERIOD), so Q=1 <-> the PWM that the controller's
Q=1 maps to today (same x-axis as estimate_emf.py).

Usage:
    python plot_saturation_velocity.py CPP_step_response-2.csv
    python plot_saturation_velocity.py CPP_step_response-2.csv --emf-umax 13.0 --emf-mfric 16.0
"""
import argparse
import platform
import os
from pathlib import Path

import numpy as np
import pandas as pd

from matplotlib import use
if platform.system() == 'Darwin':
    use('macOSX')
elif os.environ.get('DISPLAY'):
    use('TkAgg')
else:
    use('Agg')
import matplotlib.pyplot as plt

from double_regression import double_regression

DEFAULT_DATA_DIR = Path(__file__).resolve().parents[2] / 'ExperimentRecordings'
PWM_PERIOD = 10000
DEFAULT_GAIN = 0.6216901        # MOTOR_CORRECTION[0]; Q=1 -> PWM = gain*PWM_PERIOD
DATA_SMOOTHING = 2              # same as MotorCalibration.py (affects measured v_max)
V_SAT_MAX_LIN = 0.85            # |v_sat| threshold selecting points for the linear fit
MOTOR_FULL_SCALE_SAFE = 9500    # safe |PWM| clip, for marking the safe limit


def _smooth(y, n):
    return np.convolve(y, np.ones(n) / n, mode='same')


def saturation_points(file_name):
    """Replicate MotorCalibration.py: terminal (saturation) velocity per motor_input."""
    p = Path(file_name)
    if not p.is_absolute() and not p.exists():
        p = DEFAULT_DATA_DIR / p
    d = pd.read_csv(p, comment='#')
    d['positionD_last'] = d['positionD'].shift(1)
    d['positionD_smoothed'] = _smooth(d['positionD'].to_numpy(float), DATA_SMOOTHING)
    if 'motor_input' not in d.columns:
        d['motor_input'] = d['actualMotorSave']
    d = d.iloc[1:-1].reset_index(drop=True)
    d = d.loc[d['measurement'].str.contains('State:moving')]
    # keep only samples where the cart already moves the way it is pushed (terminal-ish)
    d = d.loc[(d['motor_input'] != 0) & (np.sign(d['motor_input']) == np.sign(d['positionD_last']))]

    pos = d.loc[d['motor_input'] > 0].groupby('motor_input')['positionD_smoothed'].max()
    neg = d.loc[d['motor_input'] < 0].groupby('motor_input')['positionD_smoothed'].min()
    mi_pos, vs_pos = pos.index.to_numpy(float), pos.to_numpy(float)
    mi_neg, vs_neg = neg.index.to_numpy(float), neg.to_numpy(float)
    return p, mi_pos, vs_pos, mi_neg, vs_neg


def main():
    ap = argparse.ArgumentParser(description='Saturation velocity vs Q from a step-response recording.')
    ap.add_argument('recording', nargs='?', default='CPP_step_response-2.csv',
                    help=f'step-response CSV. Relative names default to {DEFAULT_DATA_DIR}.')
    ap.add_argument('--gain', type=float, default=DEFAULT_GAIN, help='MOTOR_CORRECTION gain (Q=1 -> gain*PWM_PERIOD)')
    ap.add_argument('--emf-umax', type=float, default=None, help='overlay EMF-identified u_max(Q=1) line')
    ap.add_argument('--emf-mfric', type=float, default=None, help='EMF-identified M_fric (with --emf-umax)')
    ap.add_argument('--no-plot', action='store_true')
    args = ap.parse_args()

    path, mi_pos, vs_pos, mi_neg, vs_neg = saturation_points(args.recording)
    mi_Q1 = args.gain * PWM_PERIOD
    q_pos, q_neg = mi_pos / mi_Q1, mi_neg / mi_Q1
    q_safe = MOTOR_FULL_SCALE_SAFE / mi_Q1

    # linear-range points (as in MotorCalibration.py) and the double-line fit
    lp = np.abs(vs_pos) <= V_SAT_MAX_LIN
    ln = np.abs(vs_neg) <= V_SAT_MAX_LIN
    a, B_pos, B_neg = double_regression(mi_pos[lp], vs_pos[lp], mi_neg[ln], vs_neg[ln])
    # a is slope in motor_input units; in equivalent-Q units slope is a*mi_Q1
    slope_q = a * mi_Q1

    print(f'\nfile: {path}')
    print(f'samples: {len(mi_pos)} pos Q-levels, {len(mi_neg)} neg Q-levels')
    print(f'linear fit  v_sat = a*motor_input + B   ->  in Q units: v_sat = {slope_q:.3f}*Q + B')
    print(f'  a = {a:.6e} (per motor_input)   B_pos = {B_pos:.4f}   B_neg = {B_neg:.4f}')
    print(f'  v_sat at Q=+1 (linear extrapolation) = {slope_q*1 + B_pos:.3f} m/s')
    print(f'  max |v_sat| measured = {max(np.abs(vs_pos).max(), np.abs(vs_neg).max()):.3f} m/s')
    print(f'  max equivalent Q reached = {max(q_pos.max(), abs(q_neg.min())):.2f}')

    if args.no_plot:
        return

    fig, ax = plt.subplots(figsize=(8.5, 6))

    # measured saturation points, coloured by whether they were used for the linear fit
    ax.scatter(q_pos[lp], vs_pos[lp], color='tab:green', s=28, zorder=3, label=f'linear-fit points (|v_sat|<={V_SAT_MAX_LIN})')
    ax.scatter(q_neg[ln], vs_neg[ln], color='tab:green', s=28, zorder=3)
    ax.scatter(q_pos[~lp], vs_pos[~lp], color='tab:red', s=28, zorder=3, label='beyond linear range (saturation/back-EMF)')
    ax.scatter(q_neg[~ln], vs_neg[~ln], color='tab:red', s=28, zorder=3)

    # fitted straight lines extrapolated across the whole Q range -> deviation = saturation
    qx = np.linspace(min(q_neg.min(), -1.1), max(q_pos.max(), 1.1), 100)
    ax.plot(qx[qx > 0], slope_q * qx[qx > 0] + B_pos, color='tab:blue', lw=2, label='linear fit (forward)')
    ax.plot(qx[qx < 0], slope_q * qx[qx < 0] + B_neg, color='tab:blue', lw=2, ls='--', label='linear fit (reverse)')

    # optional EMF-identified line for direct comparison
    if args.emf_umax is not None and args.emf_mfric:
        coul = 0.0
        ax.plot(qx, (args.emf_umax * qx - np.sign(qx) * coul) / args.emf_mfric, color='tab:purple', lw=2, ls=':',
                label=f'EMF-ID  v_sat=u_max·Q/M_fric (u_max={args.emf_umax}, M_fric={args.emf_mfric})')

    for qv in (1.0, -1.0):
        ax.axvline(qv, color='red', lw=1.8)
    ax.axvline(q_safe, color='gray', ls=':', lw=1.2, label=f'safe PWM limit (Q=±{q_safe:.2f})')
    ax.axvline(-q_safe, color='gray', ls=':', lw=1.2)
    ax.axhline(0, color='k', lw=0.6)
    ax.text(1.02, ax.get_ylim()[0] * 0.9, 'Q=1', color='red')

    ax.set_xlabel('equivalent Q  (= motor_input / Q=1 PWM)')
    ax.set_ylabel('saturation (terminal) velocity  [m/s]')
    ax.set_title(f'Step-response saturation velocity vs Q\n{path.name}')
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8, loc='best')
    fig.tight_layout()
    out = path.with_name(path.stem + '_vsat_vs_Q.png')
    fig.savefig(out, dpi=130)
    plt.close(fig)
    print(f'\n[plot] saved figure -> {out}')


if __name__ == '__main__':
    main()
