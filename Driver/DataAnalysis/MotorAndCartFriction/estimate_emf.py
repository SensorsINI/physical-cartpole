"""EMF / force-velocity identification from an emf_identification recording.

Run the emf_identification_experiment protocol with the POLE REMOVED and NO added
mass, then point this script at the resulting CSV.

The protocol drives the cart with a pseudo-random *fraction of max motor power*, so
the analysis works in the physical PWM (power) domain using the logged actualMotorSave
(true applied PWM), independent of the Q->PWM command mapping. With no pole the cart
longitudinal force is simply F = M * accel, so we fit:

    F = k * PWM  -  M_fric * v  -  Coulomb * sign(v)

    k      [N/count]   : force per PWM count (the motor gain)
    M_fric [N/(m/s)]   : viscous + linear back-EMF
    Coulomb[N]         : residual sliding friction

From k the model force at any Q follows. Today Q=1 maps to PWM = gain*PWM_PERIOD, so
    u_max(Q=1) = k * gain * PWM_PERIOD.
The back-EMF stall speed (force -> 0 at that PWM) is u_max/M_fric ~ config v_max.

Two diagnostics this prints that the step response cannot:
  * corr(|PWM|,|v|): step response ~0.76 (collinear -> k/M_fric inseparable). A good
    emf_identification run should be well below that.
  * force-per-PWM vs |PWM| (at low speed): if it stays flat up to high power the motor
    is linear there and Q=1 could be raised; where it bends over is the true linear
    limit and where Q=1 should sit.

Usage:
    python estimate_emf.py RECORDING.csv
    python estimate_emf.py RECORDING.csv --mass 2.82 --gain 0.6216901
"""
import argparse
from pathlib import Path

import numpy as np
import pandas as pd

MOTOR_FULL_SCALE_SAFE = 9500   # safe |PWM| clip (globals.py); for marking the safe limit

DEFAULT_MASS = 2.82            # bare-cart effective mass [kg] from estimate_effective_mass.py
DEFAULT_GAIN = 0.6216901       # MOTOR_CORRECTION[0]; Q=1 -> PWM = gain*PWM_PERIOD
PWM_PERIOD = 10000
DEFAULT_DATA_DIR = Path(__file__).resolve().parents[2] / 'ExperimentRecordings'

YML_U_MAX = 1.77               # current yml, for comparison
YML_M_FRIC = 3.22


def _smooth(y, n):
    if n <= 1:
        return y
    return np.convolve(y, np.ones(n) / n, mode='same')


def _load(path, smooth_pts=3):
    p = Path(path)
    if not p.is_absolute() and not p.exists():
        p = DEFAULT_DATA_DIR / p
    d = pd.read_csv(p, comment='#')
    t = d['time'].to_numpy(float)
    v = d['positionD'].to_numpy(float)
    mi = d['actualMotorSave'].to_numpy(float)
    a = np.gradient(_smooth(v, smooth_pts), t)
    dt = np.gradient(t)
    ok = (np.isfinite(a) & (dt > 0.003) & (dt < 0.05) & (np.abs(a) < 60) & (np.abs(v) < 2.0))
    return p, a[ok], v[ok], mi[ok]


def main():
    ap = argparse.ArgumentParser(description='Identify motor force-velocity law from an EMF recording.')
    ap.add_argument('recording', help=f'CSV file. Relative names default to {DEFAULT_DATA_DIR}.')
    ap.add_argument('--mass', type=float, default=DEFAULT_MASS, help='effective cart mass [kg] (bare cart)')
    ap.add_argument('--gain', type=float, default=DEFAULT_GAIN, help='MOTOR_CORRECTION gain (Q=1 -> gain*PWM_PERIOD)')
    ap.add_argument('--smooth', type=int, default=3, help='box smoothing points for positionD before differentiation')
    ap.add_argument('--no-plot', action='store_true', help='skip saving the linearity / v_saturation figure')
    args = ap.parse_args()

    path, a, v, mi = _load(args.recording, args.smooth)
    M = args.mass
    mi_Q1 = args.gain * PWM_PERIOD
    F = M * a  # no pole, no added mass -> longitudinal force is just M*accel

    print(f'\nfile: {path}\nusable samples: {len(a)}   M = {M:.3f} kg   Q=1 PWM = {mi_Q1:.0f}')
    print(f'power coverage: max |PWM| reached = {np.abs(mi).max():.0f}  '
          f'({np.abs(mi).max() / PWM_PERIOD * 100:.0f}% of full; Q=1 is {mi_Q1 / PWM_PERIOD * 100:.0f}%)')

    drive = np.abs(mi) > 300
    corr = np.corrcoef(np.abs(mi[drive]), np.abs(v[drive]))[0, 1]
    print(f'corr(|PWM|,|v|) = {corr:.2f}   (step response ~0.76; lower -> k/M_fric separable)')

    # --- linear model fit: F = k*PWM - M_fric*v - Coulomb*sign(v) ---
    m = drive & (np.abs(v) > 0.02)
    X = np.column_stack([mi[m], -v[m], -np.sign(v[m])])
    c, *_ = np.linalg.lstsq(X, F[m], rcond=None)
    k, M_fric, coulomb = c
    resid = F[m] - X @ c
    r2 = 1.0 - np.sum(resid ** 2) / np.sum((F[m] - F[m].mean()) ** 2)

    u_max = k * mi_Q1
    v_stall = u_max / M_fric if M_fric > 0 else float('nan')
    print('\n=== identified law  F = k*PWM - M_fric*v - Coulomb*sign(v) ===')
    print(f'  k (force per PWM count) = {k:.5f} N/count     R2 = {r2:.3f}  (n={m.sum()})')
    print(f'  u_max(Q=1) = k*gain*PWM = {u_max:6.2f} N        (yml now {YML_U_MAX})')
    print(f'  M_fric                  = {M_fric:6.2f} N/(m/s)  (yml now {YML_M_FRIC})')
    print(f'  Coulomb                 = {coulomb:6.2f} N')
    print(f'  back-EMF stall speed    = u_max/M_fric = {v_stall:5.2f} m/s  (compare config v_max)')

    # --- linearity of force vs power (at low speed): where does it bend? ---
    print('\n  force-per-PWM vs |PWM| at low speed (|v|<0.15) -- flat = linear motor:')
    print('   |PWM| bin    power%    n     k_local [N/count]')
    lowv = drive & (np.abs(v) < 0.15)
    edges = [300, 2000, 3500, 5000, 6500, 8000, 9600]
    for lo, hi in zip(edges[:-1], edges[1:]):
        b = lowv & (np.abs(mi) >= lo) & (np.abs(mi) < hi)
        if b.sum() < 8:
            print(f'   {lo:5d}-{hi:5d}   {(lo+hi)/2/PWM_PERIOD*100:4.0f}%   {b.sum():4d}    (too few)')
            continue
        kl = np.sum(mi[b] * F[b]) / np.sum(mi[b] * mi[b])
        print(f'   {lo:5d}-{hi:5d}   {(lo+hi)/2/PWM_PERIOD*100:4.0f}%   {b.sum():4d}    {kl:.5f}')

    # --- EMF curve: force-per-PWM vs speed (motoring) ---
    print('\n  force-per-PWM vs |v| (motoring) -- the back-EMF collapse, measured:')
    print('   |v| bin (m/s)    n     k_local [N/count]')
    mo = drive & (np.sign(mi) == np.sign(v)) & (np.abs(v) > 0.02)
    vedges = [0.0, 0.15, 0.30, 0.45, 0.60, 0.80, 1.2]
    for lo, hi in zip(vedges[:-1], vedges[1:]):
        b = mo & (np.abs(v) >= lo) & (np.abs(v) < hi)
        if b.sum() < 8:
            print(f'   {lo:.2f}-{hi:.2f}      {b.sum():4d}    (too few)')
            continue
        kl = np.sum(mi[b] * F[b]) / np.sum(mi[b] * mi[b])
        print(f'   {lo:.2f}-{hi:.2f}      {b.sum():4d}    {kl:.5f}')

    print('\nSuggested cartpole_physical_parameters.yml (keep honest mass m_cart = {:.2f}):'.format(M))
    print(f'  u_max:  {u_max:.2f}    # = k*gain*PWM_PERIOD at the current Q=1 mapping')
    print(f'  M_fric: {M_fric:.2f}')
    print('If force-per-PWM stays flat well past the Q=1 PWM, the gain (MOTOR_CORRECTION[0]) '
          'can be raised to give RPGD more authority within the linear range.')
    print('For a fully physical model add an explicit back-EMF term so force -> 0 at '
          f'v_max ~ {v_stall:.2f} m/s instead of folding it into M_fric.')

    if not args.no_plot:
        plot_fit(path, a, v, mi, F, k, M_fric, coulomb, mi_Q1, args.gain)


def plot_fit(path, a, v, mi, F, k, M_fric, coulomb, mi_Q1, gain):
    """Two panels: (1) force-per-PWM vs equivalent Q -> is Q=1 still linear?
                    (2) saturation (terminal) velocity vs Q."""
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except Exception as e:
        print(f'[plot] matplotlib unavailable ({e}); skipping figure.')
        return

    q_eq = mi / mi_Q1                     # equivalent Q (Q=1 <-> PWM = gain*PWM_PERIOD)
    q_safe = MOTOR_FULL_SCALE_SAFE / mi_Q1
    drive = np.abs(mi) > 300

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 5.2))

    # --- Panel 1: linearity. force-per-PWM (k_local) vs equivalent Q at low speed ---
    lowv = drive & (np.abs(v) < 0.15)
    edges = np.array([300, 1500, 2500, 3500, 4500, 5500, 6500, 7500, 8500, 9600])
    qc, kc, ke = [], [], []
    for lo, hi in zip(edges[:-1], edges[1:]):
        b = lowv & (np.abs(mi) >= lo) & (np.abs(mi) < hi)
        if b.sum() < 8:
            continue
        kl = np.sum(mi[b] * F[b]) / np.sum(mi[b] * mi[b])
        qc.append((lo + hi) / 2 / mi_Q1)
        kc.append(kl)
        ke.append(np.std(F[b] - kl * mi[b]) / (np.sqrt(b.sum()) * np.mean(np.abs(mi[b]))))
    ax1.errorbar(qc, kc, yerr=ke, fmt='o-', color='tab:blue', capsize=3, label='measured force/PWM (|v|<0.15)')
    ax1.axhline(k, color='tab:green', ls='--', label=f'global linear fit k={k:.4f}')
    ax1.axvline(1.0, color='red', lw=2, label='Q = 1 (controller max)')
    ax1.axvline(q_safe, color='gray', ls=':', label=f'safe PWM limit (Q={q_safe:.2f})')
    ax1.set_xlabel('equivalent Q  (= applied PWM / Q=1 PWM)')
    ax1.set_ylabel('force per PWM count  [N/count]')
    ax1.set_title('Linearity: is Q=1 in the linear regime?')
    ax1.set_ylim(bottom=0)
    ax1.grid(alpha=0.3)
    ax1.legend(fontsize=8, loc='lower left')

    # --- Panel 2: saturation (terminal) velocity vs Q ---
    qline = np.linspace(0, max(1.5, q_safe), 100)
    v_sat_line = (k * mi_Q1 * qline - coulomb) / M_fric
    ax2.plot(qline, np.clip(v_sat_line, 0, None), 'tab:green', lw=2,
             label=f'model  v_sat = (u_max·Q - Coul)/M_fric')
    # empirical near-terminal points: motoring & quasi-steady (|accel| small)
    steady = drive & (np.sign(mi) == np.sign(v)) & (np.abs(v) > 0.05) & (np.abs(a) < 1.5)
    ax2.scatter(np.abs(q_eq[steady]), np.abs(v[steady]), s=6, alpha=0.25,
                color='tab:blue', label='measured (motoring, |accel|<1.5)')
    v_sat_q1 = (k * mi_Q1 - coulomb) / M_fric
    ax2.axvline(1.0, color='red', lw=2, label='Q = 1')
    ax2.plot([1.0], [v_sat_q1], 'r*', ms=14)
    ax2.annotate(f'v_sat(Q=1) ≈ {v_sat_q1:.2f} m/s', (1.0, v_sat_q1),
                 textcoords='offset points', xytext=(8, 6), color='red')
    ax2.set_xlabel('equivalent Q  (= applied PWM / Q=1 PWM)')
    ax2.set_ylabel('saturation / terminal velocity  [m/s]')
    ax2.set_title('Saturation velocity vs Q')
    ax2.set_xlim(left=0)
    ax2.set_ylim(bottom=0)
    ax2.grid(alpha=0.3)
    ax2.legend(fontsize=8, loc='upper left')

    fig.suptitle(f'EMF identification: {Path(path).name}', fontsize=11)
    fig.tight_layout(rect=[0, 0, 1, 0.96])
    out = Path(path).with_name(Path(path).stem + '_emf_fit.png')
    fig.savefig(out, dpi=130)
    plt.close(fig)
    print(f'\n[plot] saved figure -> {out}')


if __name__ == '__main__':
    main()
