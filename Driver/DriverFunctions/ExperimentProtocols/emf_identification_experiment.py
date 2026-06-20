# EMF / force-velocity identification protocol  (full-track decorrelated shuttle).
# control.py calls update_state() if state is not 'idle'.
#
# PURPOSE
# -------
# Identify the motor force-velocity law  F = u_max*Q - b*v  (b = viscous + back-EMF)
# WITHOUT the collinearity that cripples the ordinary step response, across the FULL
# motor-power range, AND across the FULL velocity range the bare cart can reach on the
# track (~1.0-1.5 m/s) -- i.e. into the regime where back-EMF actually bites during
# swing-up.
#
# WHY A SHUTTLE (and not random short reversals)
# ----------------------------------------------
# An earlier version held random-sign power for ~60 ms each. The cart never built up
# speed before reversing, so it topped out at ~0.8 m/s and spent half the run in a
# closed-loop "return to centre" that produced correlated, throw-away data.
# Two fixes, both physically motivated:
#  1. SPEED: with accel/Q ~ 5-7 m/s^2, accelerating over ~0.3 m of track reaches
#     v = sqrt(2*a*d) ~ 1.3 m/s. So we let the cart ACCELERATE across the full track in
#     one direction (excitation sign biased toward travel) to actually reach swing-up
#     speeds, instead of jittering in place.
#  2. THE TURN IS DATA, NOT OVERHEAD: at each end we do NOT hand the cart to a PD
#     return controller (closed-loop -> power and v correlated -> useless for ID).
#     Instead we command a KNOWN open-loop reverse power. During that turn the cart is
#     moving fast one way while force is applied the other way: power and velocity have
#     OPPOSITE signs -> maximally decorrelated -> the cleanest possible separation of the
#     force gain k from the speed damping b. This data is recorded and used by the fit.
#
# Within a sweep the power MAGNITUDE is still randomised (and coasts are injected), so at
# any given speed we see many power levels -> the (power, v) plane is filled instead of
# lying on a line. Net effect: high v IS reached, and corr(|PWM|,|v|) drops.
#
# Excitation is commanded as a FRACTION OF MAX MOTOR POWER p in [-POWER_MAX, POWER_MAX],
# converted to the protocol command by dividing out the MOTOR_CORRECTION gain so we can
# drive beyond the Q=1 point (Q=1 today is only ~0.62 of full PWM). The true applied PWM
# is logged as actualMotorSave, so the fit uses measured power regardless of the mapping.
#
# RUN IT WITH THE POLE REMOVED AND NO ADDED MASS so the recorded cart acceleration is
# purely the motor + cart-friction longitudinal dynamics.
#
# Real-data care taken here:
#   * Each power level is held >= HOLD_MIN_S (several control steps) so every sample is
#     past the command/actuation latency.
#   * Coast (p=0) segments are injected on purpose: there accel = -(b*v + Coulomb)/M,
#     a confound-free read of the speed damping at whatever v the cart currently has.
#   * Boundary protection is STOPPING-DISTANCE based (v^2/2a), not a fixed look-ahead, so
#     it brakes early enough at high speed -- this is what stops the cart overshooting
#     the soft limit (the previous fixed-look-ahead brake let it reach 20.8 cm).
#   * The encoder zero drifts over a long run, so the firmware position calibration is
#     re-run between blocks (single continuous recording, like step_response).
#   * A seeded RNG makes the excitation reproducible.

import numpy as np
from DriverFunctions.ExperimentProtocols import template_experiment_protocol
from globals import (
    MOTOR_CORRECTION,
    CORRECT_MOTOR_DYNAMICS,
    MOTOR_PWM_PERIOD_IN_CLOCK_CYCLES,
    MOTOR_FULL_SCALE_SAFE,
)
from CartPoleSimulation.CartPole.state_utilities import POSITIOND_IDX

# ---- excitation (in FRACTION OF MAX MOTOR POWER) --------------------------
RNG_SEED = 12345              # reproducible pseudo-random drive
POWER_MAX = 0.85              # peak |power| as fraction of full PWM (safe limit ~0.95;
                              #   Q=1 today is only ~0.62). Raise toward 0.95 to probe the
                              #   very top of the motor, lower if the cart is too lively.
P_FORWARD = 0.72              # prob the excitation sign follows travel direction (builds
                              #   speed). Otherwise random sign -> decorrelating sample.
P_COAST = 0.18                # probability a segment is a pure coast (p = 0)
HOLD_MIN_S = 0.08             # min duration a power level is held (>> control period & latency)
HOLD_MAX_S = 0.25             # max hold; short enough to vary power while v stays high

# ---- track boundary protection (stopping-distance based) ------------------
# Usable half-track is ~0.20 m (track_length 0.44, cart 0.044). Stay well inside.
# IMPORTANT: protection keys off the ACTUAL (position, velocity) toward whichever wall
# the cart is really approaching -- NOT off the commanded travel direction. The random
# opposite-sign excitation can flip the real motion while the bias dir stays put, so a
# dir-based brake watched the wrong wall and let the cart sail into the other one.
TURN_SOFT = 0.14              # |position| [m] the cart must always be able to stop within
HARD_LIMIT = 0.18            # |position| [m] emergency: command full reverse immediately
A_BRAKE = 5.0                 # m/s^2 conservative reverse decel used to predict stop distance
TURN_MARGIN = 0.03            # extra [m] safety room added to the predicted stop point
LATENCY_S = 0.04              # command/actuation latency added to the stop prediction
REVERSE_POWER = 0.85          # open-loop |power| commanded during a turn (KNOWN -> data!)
V_FLIP = 0.05                 # m/s: turn is complete once velocity heads back this fast
REVERSE_TIMEOUT_S = 1.5       # safety: never stay stuck in a reversal
V_CLIP = 3.0                  # clip noisy velocity before using it for predictions

# ---- centring (brief, PD; only between blocks, excluded from the fit) -----
INNER_LIMIT = 0.04            # |position| [m] at which we consider the cart re-centred
RETURN_KP = 5.0               # P-gain (1/m) for the centring push (in power units)
RETURN_KD = 1.5               # D-gain (s/m)
RETURN_POWER_MAX = 0.90       # cap on the centring push (fraction of max power)
CENTER_TIMEOUT_S = 15.0       # max time allowed to drive the cart to the centre

# ---- blocks / recalibration ----------------------------------------------
EXCITE_DURATION_S = 20.0      # excitation time per block before re-zeroing the encoder
NUM_BLOCKS = 8                # number of excitation blocks in one recording
RECALIBRATE_BETWEEN_BLOCKS = True
CALIBRATION_SAFETY_TIMEOUT_S = 30  # abort if firmware calibration never reports back


class emf_identification_experiment(template_experiment_protocol):
    def __init__(self, driver):
        super().__init__(
            driver=driver,
            experiment_protocol_name=self.__class__.__name__[:-len('_experiment')],
        )
        self.rng = None
        self.gain = None
        self.time_state_changed = None
        self.segment_end_time = None
        self.block_start_time = None
        self.blocks_done = 0
        self.exciting_power = 0.0
        self.dir = 1              # excitation-bias direction (+1 / -1)
        self.reverse_wall = 1     # wall currently being braked away from

    # -- power (fraction of max PWM) -> protocol command --------------------
    def _q_for_power(self, p):
        """Return the protocol Q so that the net applied PWM is ~ p * PWM_PERIOD.

        With CORRECT_MOTOR_DYNAMICS the driver computes PWM = (gain*Q + offset)*PERIOD,
        so dividing by the gain lets us command beyond the Q=1 point up to the safe
        clip. p=0 returns 0 exactly (motor off / coast, no stiction offset added).
        """
        if p == 0.0:
            return 0.0
        if CORRECT_MOTOR_DYNAMICS and self.gain:
            return p / self.gain
        return p

    # -- setup --------------------------------------------------------------
    def set_up_experiment(self, first_iteration=True):
        self.rng = np.random.default_rng(RNG_SEED)
        self.gain = MOTOR_CORRECTION[0]
        self.blocks_done = 0
        self.exciting_power = 0.0
        self.dir = 1
        self.reverse_wall = 1
        self.time_state_changed = None
        self.segment_end_time = None
        self.block_start_time = None

        safe_frac = MOTOR_FULL_SCALE_SAFE / MOTOR_PWM_PERIOD_IN_CLOCK_CYCLES
        if POWER_MAX > safe_frac:
            print(f'[emf-id] WARNING: POWER_MAX={POWER_MAX} exceeds safe {safe_frac:.2f}; '
                  f'driver will clip at {MOTOR_FULL_SCALE_SAFE}.')

        self.current_experiment_phase = 'centering'
        self.Q = 0.0
        self.start_new_recording()

    # -- main dispatch ------------------------------------------------------
    def update_state(self, angle: int, position: int, time_now: float):
        if self.current_experiment_phase == 'idle':
            pass
        elif self.current_experiment_phase == 'centering':
            self.action_centering(position, time_now)
        elif self.current_experiment_phase == 'exciting':
            self.action_exciting(position, time_now)
        elif self.current_experiment_phase == 'reversing':
            self.action_reversing(position, time_now)
        elif self.current_experiment_phase == 'calibrating':
            self.action_calibrating(time_now)
        else:
            raise Exception(f'unknown state {self.current_experiment_phase}')

        self.data_to_save_measurement = {'measurement': self}

    # -- helpers ------------------------------------------------------------
    def _velocity(self):
        try:
            v = float(self.driver.s[POSITIOND_IDX])
        except Exception:
            return 0.0
        return float(np.clip(v, -V_CLIP, V_CLIP))

    def _approaching_wall(self, position):
        """Return +1 / -1 for the wall the cart will hit, else 0 -- from ACTUAL motion.

        Predicts where the cart stops under max braking (stop dist ~ v^2/2a) plus one
        step of latency. Keying off real (position, velocity) means it watches whichever
        wall is actually being approached, regardless of the commanded bias direction.
        """
        v = self._velocity()
        stop_dist = (v * v) / (2.0 * A_BRAKE)
        stop_pos = position + v * LATENCY_S + np.sign(v) * stop_dist
        if stop_pos + TURN_MARGIN > TURN_SOFT or position > TURN_SOFT:
            return 1
        if stop_pos - TURN_MARGIN < -TURN_SOFT or position < -TURN_SOFT:
            return -1
        return 0

    def _return_command(self, position):
        """PD push back toward centre (centring only), in power fraction -> Q."""
        v = self._velocity()
        power = -(RETURN_KP * position + RETURN_KD * v)
        power = float(np.clip(power, -RETURN_POWER_MAX, RETURN_POWER_MAX))
        return self._q_for_power(power)

    # -- centring -----------------------------------------------------------
    def action_centering(self, position, time_now):
        if self.time_state_changed is None:
            self.time_state_changed = time_now
        if abs(position) <= INNER_LIMIT:
            self.Q = 0.0
            self._begin_block(time_now)
        else:
            self.Q = self._return_command(position)
            if time_now - self.time_state_changed > CENTER_TIMEOUT_S:
                print('[emf-id] Centring timed out; starting excitation anyway.')
                self._begin_block(time_now)

    # -- excitation: accelerate across the track, random-magnitude power ----
    def action_exciting(self, position, time_now):
        if abs(position) > HARD_LIMIT:
            self._enter_reversing(1 if position > 0 else -1, time_now)
            return

        wall = self._approaching_wall(position)
        if wall != 0:
            self._enter_reversing(wall, time_now)
            return

        if time_now - self.block_start_time > EXCITE_DURATION_S:
            self._end_block(time_now)
            return

        if self.segment_end_time is None or time_now >= self.segment_end_time:
            self._draw_new_segment(time_now)

        self.Q = self._q_for_power(self.exciting_power)

    def _draw_new_segment(self, time_now):
        hold = float(self.rng.uniform(HOLD_MIN_S, HOLD_MAX_S))
        if self.rng.uniform() < P_COAST:
            self.exciting_power = 0.0  # coast: accel = -(b*v + Coulomb)/M, confound-free
        else:
            mag = float(self.rng.uniform(0.0, POWER_MAX))
            sign = self.dir if self.rng.uniform() < P_FORWARD else float(self.rng.choice([-1.0, 1.0]))
            self.exciting_power = sign * mag
        self.segment_end_time = time_now + hold

    # -- reversing: KNOWN open-loop reverse power (recorded as data) --------
    def _enter_reversing(self, wall, time_now):
        """Brake/push AWAY from `wall` (the wall actually being approached)."""
        self.reverse_wall = wall
        self.current_experiment_phase = 'reversing'
        self.time_state_changed = time_now
        self.exciting_power = -wall * REVERSE_POWER
        self.Q = self._q_for_power(self.exciting_power)

    def action_reversing(self, position, time_now):
        wall = self.reverse_wall
        # if a glitch pushed us hard against the far wall, re-target the real one
        if abs(position) > HARD_LIMIT:
            wall = 1 if position > 0 else -1
            self.reverse_wall = wall
        self.exciting_power = -wall * REVERSE_POWER
        self.Q = self._q_for_power(self.exciting_power)

        v_back = -self._velocity() * wall              # >0 once heading back off the wall
        off_wall = abs(position) < (TURN_SOFT - TURN_MARGIN)
        timed_out = (time_now - self.time_state_changed) > REVERSE_TIMEOUT_S
        if (v_back > V_FLIP and off_wall) or timed_out:
            self.dir = -wall                           # bias next sweep back toward centre
            if time_now - self.block_start_time > EXCITE_DURATION_S:
                self._end_block(time_now)
            else:
                self.current_experiment_phase = 'exciting'
                self._draw_new_segment(time_now)
                self.Q = self._q_for_power(self.exciting_power)

    # -- block bookkeeping --------------------------------------------------
    def _begin_block(self, time_now):
        self.block_start_time = time_now
        self.segment_end_time = None
        self.dir = -1 if self._position_sign() > 0 else 1   # head away from the near wall
        self.current_experiment_phase = 'exciting'
        self._draw_new_segment(time_now)
        self.Q = self._q_for_power(self.exciting_power)
        print(f'[emf-id] Block {self.blocks_done + 1}/{NUM_BLOCKS}: shuttle excitation '
              f'started (|power| up to {POWER_MAX} of full motor).')

    def _position_sign(self):
        try:
            from CartPoleSimulation.CartPole.state_utilities import POSITION_IDX
            return 1.0 if float(self.driver.s[POSITION_IDX]) > 0 else -1.0
        except Exception:
            return -1.0

    def _end_block(self, time_now):
        self.blocks_done += 1
        self.Q = 0.0
        if self.blocks_done >= NUM_BLOCKS:
            print('[emf-id] All blocks done.')
            self.stop()
            return
        if RECALIBRATE_BETWEEN_BLOCKS:
            self._begin_recalibration()
        else:
            self._begin_block(time_now)

    # -- recalibration (re-zero encoder; does NOT stop the recording) -------
    def _begin_recalibration(self):
        self.Q = 0.0
        self.driver.InterfaceInstance.set_motor(0)
        self.driver.InterfaceInstance.pc_control_mode(False)
        self.driver.InterfaceInstance.start_calibration()
        self.current_experiment_phase = 'calibrating'
        self.time_state_changed = None
        print('[emf-id] Re-zeroing encoder between blocks (firmware calibration)...')

    def action_calibrating(self, time_now):
        if self.time_state_changed is None:
            self.time_state_changed = time_now
        self.Q = 0.0
        if not self.driver.InterfaceInstance.calibration_in_progress:
            self.driver.InterfaceInstance.pc_control_mode(True)
            print('[emf-id] Recalibration done, resuming.')
            self.current_experiment_phase = 'centering'
            self.time_state_changed = None
        elif time_now - self.time_state_changed > CALIBRATION_SAFETY_TIMEOUT_S:
            print('[emf-id] Calibration did not report back in time; stopping experiment.')
            self.stop()

    def __str__(self):
        v = self._velocity()
        moving = 'State:moving' if self.current_experiment_phase in ('exciting', 'reversing') else 'State:idle'
        return (f' EMF-ID ({moving}, Phase:{self.current_experiment_phase}, '
                f'dir:{self.dir:+d}, power:{self.exciting_power:.2f}, Q:{self.Q}, v:{v:.3f})')
