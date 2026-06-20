"""Estimate motor force gain from pulse_identification_experiment recordings.

The protocol logs short known-power pulses at known cart velocities:

    M_eff * a = k * PWM - M_fric * v - C * sign(v)

For the most defensible fit, keep the step-response terminal-velocity slope fixed:

    v_terminal = (k / M_fric) * PWM

which means M_fric = k / terminal_slope_pwm. Then the pulse data only has to identify
the absolute force gain k (and Coulomb C). This combines:
  * step response: precise terminal ratio k / M_fric
  * pulse response: absolute force scale k through M_eff * acceleration

Usage:
    python estimate_pulse_identification.py CPP_pulse_identification-1.csv
    python estimate_pulse_identification.py CPP_pulse_identification-1.csv --free-mfric
"""
import argparse
import re
from pathlib import Path

import numpy as np
import pandas as pd


DEFAULT_DATA_DIR = Path(__file__).resolve().parents[2] / 'ExperimentRecordings'
DEFAULT_MASS = 2.82
DEFAULT_GAIN = 0.6216901
PWM_PERIOD = 10000

# From CPP_step_response-2.csv terminal velocity:
# v_terminal / PWM = 9.4185e-5 m/s per PWM count (direction-combined ~9.3e-5).
DEFAULT_TERMINAL_SLOPE_PWM = 9.418527e-5


def smooth(y, n):
    if n <= 1:
        return y
    return np.convolve(y, np.ones(n) / n, mode='same')


def load_pulse_samples(path, smooth_pts, latency_s, window_s):
    p = Path(path)
    if not p.is_absolute() and not p.exists():
        p = DEFAULT_DATA_DIR / p
    d = pd.read_csv(p, comment='#')
    t = d['time'].to_numpy(float)
    v = d['positionD'].to_numpy(float)
    pwm = d['actualMotorSave'].to_numpy(float)
    a = np.gradient(smooth(v, smooth_pts), t)

    measurement = d['measurement'].astype(str)
    is_pulse = measurement.str.contains('Phase:pulse', regex=False).to_numpy()
    pulse_t = measurement.str.extract(r'pulse_t:([-+0-9.]+)')[0].astype(float).to_numpy()
    v_target = measurement.str.extract(r'v_target:([-+0-9.]+)')[0].astype(float).to_numpy()
    power = measurement.str.extract(r'power:([-+0-9.]+)')[0].astype(float).to_numpy()

    ok = (
        is_pulse
        & np.isfinite(a)
        & np.isfinite(v)
        & np.isfinite(pwm)
        & np.isfinite(pulse_t)
        & (pulse_t >= latency_s)
        & (pulse_t <= latency_s + window_s)
        & (np.abs(pwm) > 300)
        & (np.abs(v) < 2.0)
        & (np.abs(a) < 80.0)
    )
    return p, t[ok], a[ok], v[ok], pwm[ok], v_target[ok], power[ok]


def constrained_fit(F, v, pwm, terminal_slope_pwm):
    """Fit F = k*PWM - (k/slope)*v - C*sign(v), linear in k and C."""
    x_k = pwm - v / terminal_slope_pwm
    X = np.column_stack([x_k, -np.sign(v)])
    k, coulomb = np.linalg.lstsq(X, F, rcond=None)[0]
    M_fric = k / terminal_slope_pwm
    return k, M_fric, coulomb


def free_fit(F, v, pwm):
    X = np.column_stack([pwm, -v, -np.sign(v)])
    return np.linalg.lstsq(X, F, rcond=None)[0]


def summarize_by_groups(v_target, power, residual):
    print('\nResidual by target velocity and pulse power (mean +/- std, N):')
    df = pd.DataFrame({'v_target': v_target, 'power': power, 'residual': residual})
    grouped = df.groupby(['v_target', 'power'])['residual']
    for (vt, pwr), r in grouped:
        if len(r) < 2:
            continue
        print(f'  v={vt:+.2f}, p={pwr:+.2f}: n={len(r):3d}, mean={r.mean():+6.2f}, std={r.std():5.2f}')


def main():
    ap = argparse.ArgumentParser(description='Estimate force gain from velocity-conditioned pulse recordings.')
    ap.add_argument('recording', help=f'CSV file. Relative names default to {DEFAULT_DATA_DIR}.')
    ap.add_argument('--mass', type=float, default=DEFAULT_MASS)
    ap.add_argument('--gain', type=float, default=DEFAULT_GAIN)
    ap.add_argument('--smooth', type=int, default=3)
    ap.add_argument('--latency', type=float, default=0.04, help='ignore this much time after pulse command [s]')
    ap.add_argument('--window', type=float, default=0.08, help='use this many seconds after latency [s]')
    ap.add_argument('--terminal-slope-pwm', type=float, default=DEFAULT_TERMINAL_SLOPE_PWM,
                    help='step-response terminal slope d(v_terminal)/d(PWM)')
    ap.add_argument('--free-mfric', action='store_true',
                    help='fit k and M_fric independently instead of constraining their ratio')
    args = ap.parse_args()

    path, t, a, v, pwm, v_target, power = load_pulse_samples(
        args.recording, args.smooth, args.latency, args.window
    )
    F = args.mass * a
    if len(F) < 10:
        raise SystemExit('Too few usable pulse samples; check latency/window or recording.')

    if args.free_mfric:
        k, M_fric, coulomb = free_fit(F, v, pwm)
        fit_name = 'free fit'
    else:
        k, M_fric, coulomb = constrained_fit(F, v, pwm, args.terminal_slope_pwm)
        fit_name = 'terminal-slope constrained fit'

    pred = k * pwm - M_fric * v - coulomb * np.sign(v)
    residual = F - pred
    r2 = 1.0 - np.sum(residual ** 2) / np.sum((F - F.mean()) ** 2)
    mi_Q1 = args.gain * PWM_PERIOD
    u_max = k * mi_Q1

    print(f'\nfile: {path}')
    print(f'usable pulse samples: {len(F)}   mass={args.mass:.3f} kg')
    print(f'fit: {fit_name}   latency={args.latency:.3f}s, window={args.window:.3f}s')
    print(f'k                 = {k:.6f} N/PWM_count')
    print(f'u_max(Q=1)        = {u_max:.2f} N   (Q=1 PWM={mi_Q1:.0f})')
    print(f'M_fric            = {M_fric:.2f} N/(m/s)')
    print(f'Coulomb           = {coulomb:.2f} N')
    print(f'v_terminal(Q=1)   = {u_max / M_fric:.3f} m/s')
    print(f'R2                = {r2:.3f}')

    summarize_by_groups(v_target, power, residual)


if __name__ == '__main__':
    main()
