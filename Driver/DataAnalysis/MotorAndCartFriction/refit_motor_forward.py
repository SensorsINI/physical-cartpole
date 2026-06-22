"""Re-identify the cartpole motor force law by FORWARD velocity-trajectory fitting.

Why this exists
---------------
The earlier scripts (``estimate_emf.py`` / ``estimate_pulse_identification.py``)
regress the *acceleration* ``a = dv/dt`` against ``(PWM, v, sign v)``. That:
  1. differentiates ``positionD`` a SECOND time (positionD is already a
     derivative of position) -> very noisy;
  2. on a single step response leaves ``PWM`` and ``v`` collinear (they ramp
     together), so the fit trades force gain against damping and inflates u_max.

This script instead fixes the robust effective mass (``m_cart = 2.82 kg`` from
the added-mass experiment) and fits ``(force_gain, M_fric, Coulomb)`` by
minimizing the error of a *multi-step forward rollout of the velocity*:

    v_{n+1} = v_n + dt * (force(cmd_n) - M_fric*v_n - Coulomb*sign(v_n)) / m_cart

seeded from the measured velocity and fed the recorded command. It uses
``positionD`` directly (one derivative, not two) and is fitted JOINTLY across
three recordings that together span the whole (command, velocity) plane, which
breaks the collinearity:

  * step response  -> long constant-command dwells -> clean damping / terminal v
  * pulse test     -> short pulses at many fixed velocities -> clean force gain
  * EMF shuttle    -> full-track decorrelated force-velocity sweeps

Two domains are reported:
  * Q-domain   (primary): force = u_max * Q     -- exactly the model's own law,
                so the resulting u_max is what belongs in the yml.
  * PWM-domain (cross-check): force = k * actualMotorSave, then
                u_max = k * gain * PWM_PERIOD, matching estimate_emf's convention
                so the numbers are directly comparable to the old EMF result.

Usage:
    python refit_motor_forward.py
    python refit_motor_forward.py --mass 2.82 --horizon 10
"""
import argparse
from pathlib import Path

import numpy as np
import pandas as pd
from scipy.optimize import least_squares

DATA = Path(__file__).resolve().parent / "motor_force_identification_2026-06" / "data"
RECORDINGS = {
    "step":  DATA / "CPP_step_response-2.csv",
    "pulse": DATA / "CPP_pulse_identification-1.csv",
    "emf":   DATA / "CPP_emf_identification-2.csv",
}

DEFAULT_MASS = 2.82            # bare-cart effective mass [kg], estimate_effective_mass.py
GAIN = 0.6216901              # MOTOR_CORRECTION[0] in globals during these recordings
PWM_PERIOD = 10000
V_PHYS_MAX = 1.2             # m/s; reject positionD spikes above the physical top speed

# invariants for cross-checking
V_MAX_INVARIANT = 0.59       # terminal speed at Q=1 (step-response saturation)
GAIN_INVARIANT = 7.7         # u_max / m_cart of the working hand-tuned set & pulse fit


def load(path):
    d = pd.read_csv(path, comment="#")
    return dict(
        t=d["time"].to_numpy(float),
        v=d["positionD"].to_numpy(float),
        Q=d["Q"].to_numpy(float),
        pwm=d["actualMotorSave"].to_numpy(float),
    )


def make_windows(rec, horizon, stride, dt):
    """Return (cmd_Q, cmd_pwm, v0, v_truth) arrays of windows with uniform dt,
    physically-bounded velocity, and some drive activity."""
    t, v, Q, pwm = rec["t"], rec["v"], rec["Q"], rec["pwm"]
    tol = 0.5 * dt
    qW, pW, v0, vT = [], [], [], []
    for s0 in range(0, len(t) - horizon - 1, stride):
        sl = slice(s0, s0 + horizon + 1)
        if np.any(np.abs(np.diff(t[sl]) - dt) > tol):
            continue
        if np.any(np.abs(v[sl]) > V_PHYS_MAX):           # reject derivative spikes
            continue
        if np.mean(np.abs(pwm[s0:s0 + horizon])) < 400:   # need real drive
            continue
        qW.append(Q[s0:s0 + horizon])
        pW.append(pwm[s0:s0 + horizon])
        v0.append(v[s0])
        vT.append(v[s0 + 1:s0 + horizon + 1])
    if not v0:
        return None
    return (np.asarray(qW), np.asarray(pW), np.asarray(v0), np.asarray(vT))


def rollout_residuals(params, cmd, v0, v_truth, dt, mass):
    """Multi-step forward velocity rollout; return (v_pred - v_truth) flattened."""
    gain_force, M_fric, coulomb = params
    n, H = cmd.shape
    v = v0.astype(float).copy()
    res = np.empty((n, H))
    for h in range(H):
        a = (gain_force * cmd[:, h] - M_fric * v - coulomb * np.sign(v)) / mass
        v = v + dt * a
        res[:, h] = v - v_truth[:, h]
    return res.ravel()


def fit(domain, windows, dt_list, mass, x0):
    """domain: 'Q' uses Q command, 'PWM' uses actualMotorSave command."""
    cmd_idx = 0 if domain == "Q" else 1

    def resid(p):
        chunks = []
        for w, dt in zip(windows, dt_list):
            chunks.append(rollout_residuals(p, w[cmd_idx], w[2], w[3], dt, mass))
        return np.concatenate(chunks)

    sol = least_squares(resid, x0=x0, loss="soft_l1", f_scale=0.05, max_nfev=4000)
    r = resid(sol.x)
    # R^2 against the multi-step truth
    truth = np.concatenate([w[3].ravel() for w in windows])
    pred = truth + r
    r2 = 1.0 - np.sum(r ** 2) / np.sum((truth - truth.mean()) ** 2)
    return sol.x, r2, len(truth)


def naive_acceleration_regression(recs, mass):
    """The OLD method, for contrast: m*a = gain*cmd - M_fric*v - C*sign(v) with
    a = d(positionD)/dt (a SECOND derivative). Reports the Q-v collinearity that
    makes it inflate u_max on a step response."""
    print("\n  Naive one-step acceleration regression (the old, noisy method):")
    print(f"    {'recording':<8s}{'corr(|Q|,|v|)':>15s}{'u_max(Q)':>11s}{'M_fric':>9s}")
    for name, rec in recs.items():
        t, v, Q = rec["t"], rec["v"], rec["Q"]
        a = np.gradient(v, t)
        dt = np.gradient(t)
        ok = (np.abs(v) < V_PHYS_MAX) & (dt > 0.003) & (dt < 0.05) & (np.abs(Q) > 0.05) & np.isfinite(a)
        F = mass * a[ok]
        X = np.column_stack([Q[ok], -v[ok], -np.sign(v[ok])])
        c = np.linalg.lstsq(X, F, rcond=None)[0]
        corr = np.corrcoef(np.abs(Q[ok]), np.abs(v[ok]))[0, 1]
        print(f"    {name:<8s}{corr:>15.2f}{c[0]:>11.2f}{c[1]:>9.2f}")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--mass", type=float, default=DEFAULT_MASS)
    ap.add_argument("--horizon", type=int, default=10, help="rollout length [steps]")
    ap.add_argument("--stride", type=int, default=3)
    args = ap.parse_args()
    mass = args.mass

    recs, windows, dts, names = {}, [], [], []
    for name, path in RECORDINGS.items():
        if not path.exists():
            print(f"[skip] missing {path}")
            continue
        rec = load(path)
        recs[name] = rec
        dt = float(np.median(np.diff(rec["t"])[(np.diff(rec["t"]) > 0.003) & (np.diff(rec["t"]) < 0.05)]))
        w = make_windows(rec, args.horizon, args.stride, dt)
        if w is None:
            print(f"[skip] no usable windows in {name}")
            continue
        windows.append(w); dts.append(dt); names.append(name)
        print(f"  {name:<8s}: {len(w[2]):5d} windows  dt={dt*1000:.1f}ms  "
              f"|v| up to {np.abs(w[3]).max():.2f} m/s  |pwm| up to {np.abs(w[1]).max():.0f}")

    print("\n" + "=" * 78)
    print(f"FORWARD VELOCITY-TRAJECTORY FIT  (m_cart fixed = {mass:.2f} kg, joint over recordings)")
    print("=" * 78)

    # --- Q-domain (primary; force = u_max * Q, exactly the model law) ---
    x0_q = [20.0, 30.0, 1.0]
    (u_max, M_fric, coul), r2q, nq = fit("Q", windows, dts, mass, x0_q)
    v_term = u_max / M_fric
    print("\n  [Q-domain] force = u_max * Q   (this is what goes in the yml)")
    print(f"    u_max   = {u_max:6.2f} N        (force at Q=1)")
    print(f"    M_fric  = {M_fric:6.2f} N/(m/s)")
    print(f"    Coulomb = {coul:6.2f} N")
    print(f"    v_term(Q=1) = u_max/M_fric = {v_term:.3f} m/s")
    print(f"    accel gain  = u_max/m_cart = {u_max / mass:.2f} m/s^2")
    print(f"    R2 (multi-step v) = {r2q:.3f}   (n={nq})")

    # --- PWM-domain (cross-check; comparable to estimate_emf) ---
    x0_p = [u_max / (GAIN * PWM_PERIOD), M_fric, coul]
    (k, M_fric_p, coul_p), r2p, _ = fit("PWM", windows, dts, mass, x0_p)
    u_max_p = k * GAIN * PWM_PERIOD
    print("\n  [PWM-domain] force = k * PWM, then u_max = k*gain*PWM_PERIOD "
          f"(gain={GAIN}); comparable to estimate_emf:")
    print(f"    k       = {k:.6f} N/count")
    print(f"    u_max   = {u_max_p:6.2f} N")
    print(f"    M_fric  = {M_fric_p:6.2f} N/(m/s)")
    print(f"    R2 (multi-step v) = {r2p:.3f}")

    naive_acceleration_regression(recs, mass)

    # --- cross-check against the clean invariants ---
    print("\n" + "=" * 78)
    print("CROSS-CHECK vs the three clean invariants")
    print("=" * 78)
    print(f"  effective mass m_cart      = {mass:.2f} kg  (added-mass ratio; robust)")
    print(f"  terminal-velocity invariant: v_term(Q=1) should be ~{V_MAX_INVARIANT:.2f} m/s"
          f"  -> fit gives {v_term:.2f}")
    print(f"  acceleration-gain invariant: u_max/m_cart should be ~{GAIN_INVARIANT:.1f} m/s^2"
          f"  -> fit gives {u_max / mass:.2f}")
    print("\n  Comparison of every u_max estimate:")
    print(f"    {'method':<34s}{'u_max':>8s}{'M_fric':>9s}{'gain':>8s}")
    rows = [
        ("working hand-tuned (m=0.230)", 1.77, 3.22, 1.77 / 0.230),
        ("step response (collinear)", 25.8, 42.4, 25.8 / 2.82),
        ("EMF shuttle (current yml)", 13.0, 22.0, 13.0 / 2.82),
        ("pulse (README)", 21.6, 36.5, 21.6 / 2.82),
        ("THIS forward-fit (Q-domain)", u_max, M_fric, u_max / 2.82),
        ("THIS forward-fit (PWM-domain)", u_max_p, M_fric_p, u_max_p / 2.82),
    ]
    for nm, um, mf, g in rows:
        print(f"    {nm:<34s}{um:>8.2f}{mf:>9.2f}{g:>8.2f}")

    print("\n  Recommended yml (gated on the Step-2 forward-prediction arbiter):")
    print(f"    m_cart: {mass:.2f}")
    print(f"    u_max:  {u_max:.1f}")
    print(f"    M_fric: {M_fric:.1f}")
    print(f"    v_max:  {v_term:.2f}")


if __name__ == "__main__":
    main()
