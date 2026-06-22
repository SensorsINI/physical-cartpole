"""Offline forward-prediction validation of cartpole physical parameters.

This script uses the project's own ``predictor_ODE`` (the exact analytical
cart-pole model that RPGD/MPC integrates) as an *offline arbiter* for competing
physical-parameter sets. It does NOT touch hardware.

Idea
----
A recorded trajectory contains, at every logged step, the true state
(angle, angleD, position, positionD) and the control ``Q`` that was applied.
For a candidate parameter set we take many short windows, seed ``predictor_ODE``
with the true state at the start of each window, feed it the *recorded* control
sequence, and roll the model forward. The model that best reproduces the real
plant has the lowest multi-step prediction error -- on data alone, independent of
whether a controller happened to "work".

This disentangles the long-standing confusion that the working hand-tuned set
(m_cart=0.230, u_max=1.77) only matched the *acceleration gain* u_max/m_cart:
forward prediction also exercises the absolute mass (pole back-reaction) and the
friction at speed, so a set with the right gain but wrong absolutes still
mispredicts angle and velocity.

Usage
-----
    python validate_forward_prediction.py                 # full arbiter + pole-effect report
    python validate_forward_prediction.py --horizon 25    # change rollout length (steps)
    python validate_forward_prediction.py --quick          # fewer windows, faster

Run from anywhere; paths are resolved relative to this file.
"""
import argparse
import os
import sys
from pathlib import Path

import numpy as np
import pandas as pd

# --------------------------------------------------------------------------- #
# Make the CartPoleSimulation submodule importable (mirrors Driver/control.py)
# --------------------------------------------------------------------------- #
THIS = Path(__file__).resolve()
DRIVER = THIS.parents[2]                       # .../physical-cartpole/Driver
CPS = DRIVER / "CartPoleSimulation"            # the simulation submodule
SI_SRC = CPS / "SI_Toolkit" / "src"
for p in (str(CPS), str(SI_SRC)):
    if p not in sys.path:
        sys.path.insert(0, p)
# load_config / CartPoleParameters look for the yml relative to cwd
os.chdir(CPS)

from SI_Toolkit.computation_library import NumpyLibrary                       # noqa: E402
from SI_Toolkit.Predictors.predictor_ODE import predictor_ODE                 # noqa: E402
from CartPole.state_utilities import (                                        # noqa: E402
    ANGLE_IDX, ANGLED_IDX, ANGLE_COS_IDX, ANGLE_SIN_IDX,
    POSITION_IDX, POSITIOND_IDX,
)

# --------------------------------------------------------------------------- #
# Recording locations
# --------------------------------------------------------------------------- #
ID_DATA = THIS.parent / "motor_force_identification_2026-06" / "data"
REC_DIR = DRIVER / "ExperimentRecordings"

# The real working swing-up (pole attached, real plant behaving well).
MPC_WORKING = REC_DIR / "CPP_mpc__2026-06-20_21-02-10.csv"
# Pole-removed longitudinal runs (cart subsystem only).
STEP_REC = ID_DATA / "CPP_step_response-2.csv"
PULSE_REC = ID_DATA / "CPP_pulse_identification-1.csv"

# --------------------------------------------------------------------------- #
# Candidate parameter sets (m_cart, u_max, M_fric). m_pole/L kept at physical.
# --------------------------------------------------------------------------- #
M_POLE = 0.087
L_POLE = 0.395

PARAM_SETS = {
    "A_old_handtuned":  dict(m_cart=0.230, u_max=1.77,  M_fric=3.22),   # only set that drove HW
    "B_emf_current_yml": dict(m_cart=2.82, u_max=13.0,  M_fric=22.0),   # EMF shuttle -> current yml
    "C_pulse_invariant": dict(m_cart=2.82, u_max=21.7,  M_fric=36.8),   # pulse / 3-invariant set
    "D_forward_fit":     dict(m_cart=2.82, u_max=19.0,  M_fric=29.6),   # refit_motor_forward.py
}


# --------------------------------------------------------------------------- #
# Decoupled (no pole back-reaction) cart ODE: cart is a pure force-driven mass
# (m_cart + m_pole) with viscous + Coulomb-free friction; the pole is still
# driven by the resulting cart acceleration but does NOT push back on the cart.
# --------------------------------------------------------------------------- #
def _cartpole_ode_decoupled(ca, sa, angleD, positionD, u,
                            k, m_cart, m_pole, g, J_fric, M_fric, L):
    F_fric = -M_fric * positionD
    T_fric = -J_fric * angleD
    L_half = L / 2.0
    positionDD = (u + F_fric) / (m_cart + m_pole)
    angleDD = (g * sa + positionDD * ca + T_fric / (m_pole * L_half)) / ((k + 1) * L_half)
    return angleDD, positionDD


# --------------------------------------------------------------------------- #
# Predictor construction with per-call parameter overrides
# --------------------------------------------------------------------------- #
def build_predictor(dt, intermediate_steps=10, overrides=None, decouple_pole=False):
    """Create an uncompiled NumPy predictor_ODE and overwrite its physical params."""
    pred = predictor_ODE(
        dt=dt,
        computation_library=NumpyLibrary(),
        intermediate_steps=intermediate_steps,
        disable_individual_compilation=True,   # keep NumPy + allow param/monkeypatch
        batch_size=1,
    )
    params = pred.next_step_predictor.cpe.params
    o = dict(m_pole=M_POLE, L=L_POLE)
    if overrides:
        o.update(overrides)
    for key, val in o.items():
        setattr(params, key, np.float32(val))
    if decouple_pole:
        pred.next_step_predictor.cpe._cartpole_ode = _cartpole_ode_decoupled
    return pred


# --------------------------------------------------------------------------- #
# Recording loading
# --------------------------------------------------------------------------- #
def load_recording(path):
    d = pd.read_csv(path, comment="#")
    cols = {c: d[c].to_numpy(float) for c in
            ["time", "angle", "angleD", "angle_cos", "angle_sin",
             "position", "positionD", "Q"]}
    return cols


def median_dt(t):
    dts = np.diff(t)
    return float(np.median(dts[(dts > 0.003) & (dts < 0.05)]))


def build_state_matrix(rec):
    """(N, 6) state matrix in predictor index order."""
    N = len(rec["time"])
    s = np.zeros((N, 6), dtype=np.float32)
    s[:, ANGLE_IDX] = rec["angle"]
    s[:, ANGLED_IDX] = rec["angleD"]
    s[:, ANGLE_COS_IDX] = rec["angle_cos"]
    s[:, ANGLE_SIN_IDX] = rec["angle_sin"]
    s[:, POSITION_IDX] = rec["position"]
    s[:, POSITIOND_IDX] = rec["positionD"]
    return s


# --------------------------------------------------------------------------- #
# Windowed multi-step rollout and error accumulation
# --------------------------------------------------------------------------- #
def angle_err(pred_angle, true_angle):
    """Wrapped absolute angular error [rad]."""
    d = pred_angle - true_angle
    return np.abs(np.arctan2(np.sin(d), np.cos(d)))


def _uniform_dt_starts(t, horizon, stride, dt):
    starts = []
    tol = 0.5 * dt
    for s0 in range(0, len(t) - horizon - 1, stride):
        seg_dt = np.diff(t[s0:s0 + horizon + 1])
        if np.all(np.abs(seg_dt - dt) < tol):
            starts.append(s0)
    return np.asarray(starts)


def rollout(pred, rec, horizon, stride, dt):
    """Batched multi-step rollout over uniform-dt windows.

    Returns (predicted, truth, starts) where predicted/truth are
    (n_windows, horizon, 6). Returns (None, None, None) if no windows.

    We drive the single-step ODE model directly instead of
    predictor_ODE.predict(): the shared NumPy autoregression loop collapses the
    control axis (scalar gather), and a single-step loop here is equivalent,
    faster, and lets the decoupled-pole monkeypatch take effect.
    """
    t = rec["time"]
    s_all = build_state_matrix(rec)
    Q = rec["Q"].astype(np.float32)

    starts = _uniform_dt_starts(t, horizon, stride, dt)
    if len(starts) == 0:
        return None, None, None
    n = len(starts)

    init = s_all[starts]
    Qwin = np.stack([Q[s0:s0 + horizon] for s0 in starts])

    step = pred.next_step_predictor.step
    s = init.astype(np.float32).copy()
    predicted = np.zeros((n, horizon, 6), dtype=np.float32)
    for h in range(horizon):
        Qh = Qwin[:, h][:, None].astype(np.float32)
        s = np.asarray(step(s, Qh), dtype=np.float32)
        predicted[:, h, :] = s

    truth = np.stack([s_all[s0 + 1:s0 + horizon + 1] for s0 in starts])
    return predicted, truth, starts


def errors_from(predicted, truth):
    """Per-state forward-prediction RMSE (vs recorded truth)."""
    def rmse(idx):
        e = predicted[:, :, idx] - truth[:, :, idx]
        return float(np.sqrt(np.mean(e ** 2)))

    ang = angle_err(predicted[:, :, ANGLE_IDX], truth[:, :, ANGLE_IDX])
    ang_final = angle_err(predicted[:, -1, ANGLE_IDX], truth[:, -1, ANGLE_IDX])
    return {
        "n_windows": predicted.shape[0],
        "position": rmse(POSITION_IDX),
        "positionD": rmse(POSITIOND_IDX),
        "angle": float(np.sqrt(np.mean(ang ** 2))),
        "angleD": rmse(ANGLED_IDX),
        "position_final": float(np.sqrt(np.mean(
            (predicted[:, -1, POSITION_IDX] - truth[:, -1, POSITION_IDX]) ** 2))),
        "angle_final": float(np.sqrt(np.mean(ang_final ** 2))),
    }


def evaluate(pred, rec, horizon, stride, dt):
    predicted, truth, _ = rollout(pred, rec, horizon, stride, dt)
    if predicted is None:
        return None
    return errors_from(predicted, truth)


# --------------------------------------------------------------------------- #
# Reports
# --------------------------------------------------------------------------- #
def _fmt_row(name, r, keys):
    cells = "  ".join(f"{r[k]:>10.4f}" for k in keys)
    return f"  {name:<20s} {cells}   (n={r['n_windows']})"


def report_arbiter(horizon, stride, quick):
    print("=" * 92)
    print("STEP 2 - PARAMETER-SET ARBITER  (lower forward-prediction error = better model)")
    print("=" * 92)

    targets = [
        ("REAL swing-up (pole attached) " + MPC_WORKING.name, MPC_WORKING,
         ["position", "positionD", "angle", "angleD", "position_final", "angle_final"]),
        ("Cart-only step response " + STEP_REC.name, STEP_REC,
         ["position", "positionD", "position_final"]),
        ("Cart-only pulse test " + PULSE_REC.name, PULSE_REC,
         ["position", "positionD", "position_final"]),
    ]

    for title, path, keys in targets:
        if not path.exists():
            print(f"\n[skip] missing recording: {path}")
            continue
        rec = load_recording(path)
        dt = median_dt(rec["time"])
        stride_eff = stride * (3 if quick else 1)
        print(f"\n>> {title}")
        print(f"   dt={dt*1000:.1f} ms   horizon={horizon} steps ({horizon*dt*1000:.0f} ms)   stride={stride_eff}")
        header = "  " + f"{'param set':<20s} " + "  ".join(f"{k:>10s}" for k in keys)
        print(header)
        print("  " + "-" * (len(header) - 2))
        for name, ov in PARAM_SETS.items():
            pred = build_predictor(dt, overrides=ov)
            r = evaluate(pred, rec, horizon, stride_eff, dt)
            if r is None:
                print(f"  {name:<20s}  (no uniform-dt windows)")
                continue
            print(_fmt_row(name, r, keys))


def backreaction_fraction(rec, ov, k=1.0 / 3.0, g=9.81, J_fric=5.0e-5):
    """Per-sample fraction by which the pole back-reaction changes the cart's
    horizontal acceleration, evaluated on the recorded states (one-step, no
    integration). Directly comparable to the README's '~3% on average' claim.

    full:      positionDD with the full coupled cart-pole ODE
    decoupled: positionDD of a standalone force-driven mass (m_cart + m_pole)
    """
    m_cart, u_max, M_fric = ov["m_cart"], ov["u_max"], ov["M_fric"]
    m_pole, L = M_POLE, L_POLE
    ca, sa = rec["angle_cos"], rec["angle_sin"]
    angleD, positionD = rec["angleD"], rec["positionD"]
    u = u_max * rec["Q"]

    L_half = L / 2.0
    F_fric = -M_fric * positionD
    T_fric = -J_fric * angleD
    A = (k + 1) * (m_cart + m_pole) - m_pole * ca ** 2
    pos_full = (m_pole * g * sa * ca
                + (T_fric * ca) / L_half
                + (k + 1) * (-(m_pole * L_half * angleD ** 2 * sa) + F_fric + u)) / A
    pos_deco = (u + F_fric) / (m_cart + m_pole)

    diff = np.abs(pos_full - pos_deco)
    # Magnitude-weighted aggregate ratio: sum|delta|/sum|accel|. This is the
    # honest "average effect" -- it does NOT blow up on near-idle samples where
    # the cart acceleration is ~0 (those samples carry negligible weight).
    drive = np.abs(u) > 0.05 * (u_max if u_max > 0 else 1.0)  # actively driven samples
    swing = np.abs(angleD) > 5.0                              # violent swing-up regime
    agg = lambda mask: (float(diff[mask].sum() / max(np.abs(pos_full[mask]).sum(), 1e-9) * 100)
                        if mask.any() else float("nan"))
    return {
        "all": agg(np.ones_like(diff, dtype=bool)),
        "driven": agg(drive),
        "swing": agg(swing),
        "peak": float(np.percentile(diff / np.maximum(np.abs(pos_full), 1e-3), 99) * 100),
    }


def report_pole_effect(horizon, stride, quick):
    print("\n" + "=" * 92)
    print("STEP 3 - POLE BACK-REACTION ON FORWARD PREDICTION  (full ODE vs decoupled cart)")
    print("=" * 92)
    print("Decoupled = cart is a standalone force-driven mass (m_cart+m_pole); the pole is still")
    print("driven by the cart but does NOT push back on it.")
    if not MPC_WORKING.exists():
        print(f"[skip] missing {MPC_WORKING}")
        return
    rec = load_recording(MPC_WORKING)
    dt = median_dt(rec["time"])
    stride_eff = stride * (3 if quick else 1)

    # ---- (a) acceleration-level back-reaction fraction (README-comparable) ----
    print("\n  (a) Pole back-reaction as a fraction of the cart's horizontal acceleration")
    print("      (magnitude-weighted). 'driven' (cart actively commanded) is the headline number;")
    print("      'idle-incl' is high only because when u~0 the pole is the ONLY horizontal force")
    print("      (negligible absolute accel). Lower 'driven' => pole barely moves the cart.")
    print(f"\n      {'param set':<20s}{'driven':>9s}{'idle-incl':>11s}{'swing-up':>10s}")
    print("      " + "-" * 50)
    for name, ov in PARAM_SETS.items():
        f = backreaction_fraction(rec, ov)
        print(f"      {name:<20s}{f['driven']:>8.1f}%{f['all']:>10.1f}%{f['swing']:>9.1f}%")

    # ---- (b) absolute change in the predicted cart trajectory over 500 ms ----
    print("\n  (b) Absolute change in the PREDICTED cart trajectory when the pole is decoupled")
    print(f"      (rollout over {horizon} steps = {horizon*dt*1000:.0f} ms windows):")
    print(f"\n      {'param set':<20s}{'mean |dpos|':>13s}{'max |dpos|':>13s}")
    print("      " + "-" * 46)
    for name, ov in PARAM_SETS.items():
        pf, _, _ = rollout(build_predictor(dt, overrides=ov), rec, horizon, stride_eff, dt)
        pd_, _, _ = rollout(build_predictor(dt, overrides=ov, decouple_pole=True),
                            rec, horizon, stride_eff, dt)
        if pf is None:
            continue
        dpos = np.abs(pf[:, :, POSITION_IDX] - pd_[:, :, POSITION_IDX])
        print(f"      {name:<20s}{dpos.mean()*1000:>11.2f}mm{dpos.max()*1000:>11.2f}mm")

    print("\n  Interpretation: with the honest heavy mass the pole changes the cart acceleration by")
    print("  only a few percent (single-digit mm over half a second), so the cart may be treated as")
    print("  a standalone 2nd-order system. With the old 0.23 kg mass the back-reaction dominates")
    print("  (tens of percent, cm-scale) and cannot be dropped -- which is also why that set")
    print("  mispredicts the real cart in Step 2 despite having the right acceleration gain.")


def report_umax_sweep(horizon, stride, quick):
    """1-D data-optimal u_max on the REAL swing-up (m_cart=2.82 fixed, M_fric tied
    to the measured terminal-velocity ratio). Pins the honest u_max from data."""
    print("\n" + "=" * 92)
    print("STEP 4b - DATA-OPTIMAL u_max ON THE REAL SWING-UP  (m_cart=2.82, M_fric=u_max/v_term)")
    print("=" * 92)
    if not MPC_WORKING.exists():
        print(f"[skip] missing {MPC_WORKING}")
        return
    rec = load_recording(MPC_WORKING)
    dt = median_dt(rec["time"])
    stride_eff = stride * (3 if quick else 1)
    v_term = 0.62  # measured terminal speed at Q=1 (step saturation ~0.59, forward-fit ~0.64)
    print(f"\n  {'u_max':>7s}{'M_fric':>9s}{'gain':>7s}{'pos RMSE':>11s}{'posD RMSE':>11s}{'angle RMSE':>12s}")
    print("  " + "-" * 56)
    best_pos = (None, 1e9)
    best_vel = (None, 1e9)
    for u_max in [13, 15, 17, 18, 19, 20, 21, 22, 24, 26]:
        ov = dict(m_cart=2.82, u_max=float(u_max), M_fric=float(u_max / v_term))
        r = evaluate(build_predictor(dt, overrides=ov), rec, horizon, stride_eff, dt)
        if r is None:
            continue
        if r["position"] < best_pos[1]:
            best_pos = (u_max, r["position"])
        if r["positionD"] < best_vel[1]:
            best_vel = (u_max, r["positionD"])
        print(f"  {u_max:>7.0f}{u_max/v_term:>9.1f}{u_max/2.82:>7.2f}"
              f"{r['position']:>11.4f}{r['positionD']:>11.4f}{r['angle']:>12.4f}")
    print(f"\n  -> min positionD RMSE at u_max = {best_vel[0]:.0f} N (gain {best_vel[0]/2.82:.2f} m/s^2)"
          f"  [velocity is the direct indicator]")
    print(f"  -> position RMSE keeps creeping down to the sweep edge (near-saturation Q during")
    print(f"     swing-up is collinear); differences past ~21 N are sub-0.2 mm. Honest band ~19-22 N.")


def plot_overlay(horizon):
    """Overlay predicted vs recorded swing-up for each parameter set (visual arbiter)."""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as e:
        print(f"[plot] matplotlib unavailable ({e}); skipping figure.")
        return
    if not MPC_WORKING.exists():
        return
    rec = load_recording(MPC_WORKING)
    dt = median_dt(rec["time"])
    t = rec["time"]
    # pick a window containing real motion (largest cart travel over the horizon)
    starts = _uniform_dt_starts(t, horizon, 1, dt)
    s_all = build_state_matrix(rec)
    travel = [np.ptp(s_all[s0:s0 + horizon, POSITION_IDX]) for s0 in starts]
    s0 = int(starts[int(np.argmax(travel))])

    tt = (t[s0:s0 + horizon + 1] - t[s0]) * 1000.0
    fig, (axp, axa) = plt.subplots(1, 2, figsize=(13, 5))
    axp.plot(tt, s_all[s0:s0 + horizon + 1, POSITION_IDX] * 100, "k", lw=2.5, label="recorded")
    axa.plot(tt, s_all[s0:s0 + horizon + 1, ANGLE_IDX], "k", lw=2.5, label="recorded")
    palette = ["tab:red", "tab:orange", "tab:green", "tab:blue", "tab:purple"]
    colors = {name: palette[i % len(palette)] for i, name in enumerate(PARAM_SETS)}
    for name, ov in PARAM_SETS.items():
        pred = build_predictor(dt, overrides=ov)
        step = pred.next_step_predictor.step
        s = s_all[s0:s0 + 1].astype(np.float32).copy()
        pos, ang = [s_all[s0, POSITION_IDX]], [s_all[s0, ANGLE_IDX]]
        for h in range(horizon):
            Qh = np.array([[rec["Q"][s0 + h]]], dtype=np.float32)
            s = np.asarray(step(s, Qh), dtype=np.float32)
            pos.append(s[0, POSITION_IDX]); ang.append(s[0, ANGLE_IDX])
        axp.plot(tt, np.array(pos) * 100, "--", color=colors[name], label=name)
        axa.plot(tt, ang, "--", color=colors[name], label=name)
    axp.set_xlabel("time [ms]"); axp.set_ylabel("cart position [cm]")
    axp.set_title("Cart position: predicted vs recorded"); axp.grid(alpha=0.3); axp.legend(fontsize=8)
    axa.set_xlabel("time [ms]"); axa.set_ylabel("pole angle [rad]")
    axa.set_title("Pole angle: predicted vs recorded"); axa.grid(alpha=0.3); axa.legend(fontsize=8)
    fig.suptitle(f"Forward-prediction overlay ({horizon*dt*1000:.0f} ms) on {MPC_WORKING.name}")
    fig.tight_layout(rect=[0, 0, 1, 0.96])
    out = ID_DATA / "forward_prediction_overlay.png"
    fig.savefig(out, dpi=130); plt.close(fig)
    print(f"\n[plot] saved overlay -> {out}")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--horizon", type=int, default=25, help="rollout length in control steps")
    ap.add_argument("--stride", type=int, default=10, help="window stride in steps")
    ap.add_argument("--quick", action="store_true", help="3x larger stride for a fast pass")
    ap.add_argument("--plot", action="store_true", help="save predicted-vs-recorded overlay figure")
    args = ap.parse_args()

    report_arbiter(args.horizon, args.stride, args.quick)
    report_umax_sweep(args.horizon, args.stride, args.quick)
    report_pole_effect(args.horizon, args.stride, args.quick)
    if args.plot:
        plot_overlay(args.horizon)


if __name__ == "__main__":
    main()
