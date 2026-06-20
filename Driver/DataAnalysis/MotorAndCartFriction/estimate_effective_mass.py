"""Identify the cart's EFFECTIVE translational mass (and the honest force/friction
scale) from two dense bidirectional step-response recordings: one bare ("baseline")
and one with a precisely known extra mass ("loaded").

WHY EFFECTIVE MASS?
    The cart's body weighs only ~0.23 kg, but the motor's rotor spins G=19x faster
    than the drive pulley and is coupled to linear motion through a small pulley
    (radius r ~= 16 mm). The rotor's moment of inertia J therefore appears at the
    cart as a *reflected* translational mass m_refl = J * G^2 / r^2, which dominates.
    What matters for the cart-pole equations is exactly this effective translational
    inertia, so it is the right number to use as "m_cart".

METHOD 1 - mass (robust, the headline result):
    Per-sample longitudinal dynamics in each run:
        M * a = k*u - b*v - c*sign(v)              (u = actual motor command [PWM counts])
    =>  a = alpha*u + beta*v + gamma*sign(v),  with alpha = k/M.
    The motor constant k is identical in both runs, so
        M_load / M_base = alpha_base / alpha_load,
        M_base = dm / (alpha_base/alpha_load - 1),   M_load = M_base + dm.
    Crucially the *ratio* alpha_base/alpha_load is immune to:
      - u-v collinearity (identical in both runs -> cancels),
      - back-EMF / viscous / Coulomb friction (nuisance regressors beta, gamma),
      - any friction *increase* caused by the added mass (only shifts gamma).
    That is why M_eff comes out with a tight bootstrap confidence interval.

METHOD 2 - honest force & friction scale (uses M from method 1):
    With M known, k = alpha*M [N/PWM count], and the model force at controller Q=1 is
        u_max = k * (MOTOR_CORRECTION_gain * PWM_PERIOD).
    M_fric (viscous) = -beta*M, Coulomb = -gamma*M. NOTE: because u and v are
    correlated in step data, the alpha/beta *split* has ~+-15% collinearity
    uncertainty (unlike the mass ratio). Treat u_max/M_fric as good estimates, not
    exact. The mass M is the reliable number.

Usage:
    python estimate_effective_mass.py                       # uses committed CSVs, dm=0.32773
    python estimate_effective_mass.py BASE.csv LOAD.csv DM  # custom
"""
import sys
import numpy as np
import pandas as pd

# committed reference recordings (recorded 20.06.2026, 8 repeats each, ZYNQ, 50 Hz)
DEFAULT_BASE = 'CPP_massID_baseline.csv'
DEFAULT_LOAD = 'CPP_massID_loaded_327p73g.csv'
DEFAULT_DM = 0.32773  # kg, the extra mass on the loaded run

# ZYNQ motor command scaling (see Driver/globals.py)
PWM_PERIOD = 10000
MOTOR_CORRECTION_GAIN = 0.6216901   # MOTOR_CORRECTION_POLOLU[0]; controller Q=1 -> PWM = gain*PWM_PERIOD

# data filters
V_LO, V_HI = 0.05, 1.2     # m/s; clearly moving, but reject re-zero derivative spikes
U_MIN = 200                # PWM counts; ignore near-zero / coast / calibration samples
PWM_LIN = 7000             # PWM counts; stay below H-bridge saturation for the force slope
A_CLIP = 30.0              # m/s^2; reject differentiation outliers
N_BOOT = 4000


def _load(path):
    df = pd.read_csv(path, comment='#')
    t = df['time'].to_numpy(float)
    v = df['positionD'].to_numpy(float)
    u = df['actualMotorSave'].to_numpy(float)
    a = np.gradient(v, t)
    dt = np.gradient(t)
    good = (np.isfinite(a) & (np.abs(v) > V_LO) & (np.abs(v) < V_HI)
            & (np.abs(u) > U_MIN) & (np.sign(u) == np.sign(v))
            & (np.abs(a) < A_CLIP) & (dt > 0.005) & (dt < 0.05))
    return u[good], v[good], a[good]


def _alpha(u, v, a, pwm_lin=np.inf):
    m = np.abs(u) < pwm_lin
    X = np.column_stack([u[m], v[m], np.sign(v[m]), np.ones(m.sum())])
    coef, *_ = np.linalg.lstsq(X, a[m], rcond=None)
    return coef  # alpha, beta, gamma, intercept


def main():
    base = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_BASE
    load = sys.argv[2] if len(sys.argv) > 2 else DEFAULT_LOAD
    dm = float(sys.argv[3]) if len(sys.argv) > 3 else DEFAULT_DM

    ub, vb, ab = _load(base)
    ul, vl, al = _load(load)
    print(f"baseline '{base}': {len(ub)} samples")
    print(f"loaded   '{load}': {len(ul)} samples   (+{dm*1000:.2f} g)")

    cb = _alpha(ub, vb, ab)
    cl = _alpha(ul, vl, al)
    ratio = cb[0] / cl[0]
    M_base = dm / (ratio - 1.0)
    M_load = M_base + dm
    print("\n=== METHOD 1: effective mass ===")
    print(f"alpha(k/M): baseline={cb[0]:.5f}  loaded={cl[0]:.5f}  (baseline must be larger)")
    print(f"ratio = {ratio:.4f}")
    print(f"M_base (bare effective mass) = {M_base:.3f} kg")
    print(f"M_load (+{dm*1000:.1f} g)        = {M_load:.3f} kg")

    rng = np.random.default_rng(0)
    boots = []
    nb, nl = len(ab), len(al)
    for _ in range(N_BOOT):
        ib = rng.integers(0, nb, nb)
        il = rng.integers(0, nl, nl)
        r = _alpha(ub[ib], vb[ib], ab[ib])[0] / _alpha(ul[il], vl[il], al[il])[0]
        if r > 1.0:
            boots.append(dm / (r - 1.0))
    boots = np.array(boots)
    lo, hi = np.percentile(boots, [2.5, 97.5])
    print(f"bootstrap 95% CI: [{lo:.2f}, {hi:.2f}] kg  (median {np.median(boots):.2f}, "
          f"{len(boots)}/{N_BOOT} valid)")

    # Method 2: honest force / friction, restricted to the linear PWM range
    cbl = _alpha(ub, vb, ab, pwm_lin=PWM_LIN)
    k = cbl[0] * M_base
    u_max = k * (MOTOR_CORRECTION_GAIN * PWM_PERIOD)
    M_fric = -cbl[1] * M_base
    coulomb = -cbl[2] * M_base
    print("\n=== METHOD 2: honest force & friction (M from method 1) ===")
    print(f"k (force per PWM count)   = {k:.5f} N/count")
    print(f"u_max (force at Q=1)      = {u_max:.1f} N   (+-~15% collinearity)")
    print(f"M_fric (viscous)          = {M_fric:.1f} N/(m/s)")
    print(f"Coulomb friction          = {coulomb:.2f} N")
    print("\nFor the model (cartpole_physical_parameters.yml): m_cart := M_base, "
          "u_max := u_max, M_fric := M_fric.")


if __name__ == '__main__':
    main()
