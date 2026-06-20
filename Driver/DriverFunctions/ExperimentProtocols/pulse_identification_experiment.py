# Velocity-conditioned pulse identification protocol.
#
# PURPOSE
# -------
# Get conclusive absolute motor-force values by measuring acceleration during short,
# known-power pulses at known cart velocities.
#
# Why this protocol exists:
#   * The step-response terminal velocity cleanly identifies only u_max / M_fric.
#   * The EMF shuttle decorrelates PWM and velocity, but high-speed samples are mostly
#     transient; it is not ideal for nailing the absolute force gain.
#   * This protocol deliberately creates samples of the form
#
#         M_eff * a = k * PWM - M_fric * v - C * sign(v)
#
#     by first bringing the bare cart to a requested velocity, then applying a short
#     known PWM pulse. The analysis should use only the early pulse window after command
#     latency, before the velocity changes much.
#
# RUN WITH POLE REMOVED AND NO ADDED MASS.
#
# Measurement string contains:
#   Phase:<...>, Trial:<n>, v_target:<...>, power:<...>, pulse_t:<...>
# This lets the analysis script select only pulse samples and discard centering/recovery.

import numpy as np

from DriverFunctions.ExperimentProtocols import template_experiment_protocol
from globals import (
    MOTOR_CORRECTION,
    CORRECT_MOTOR_DYNAMICS,
    MOTOR_PWM_PERIOD_IN_CLOCK_CYCLES,
    MOTOR_FULL_SCALE_SAFE,
)
from CartPoleSimulation.CartPole.state_utilities import POSITIOND_IDX


# ---- experiment design ----------------------------------------------------
RNG_SEED = 24680

# Target velocities before the pulse. 0 probes static / near-static launch, nonzero
# targets identify the velocity term while the pulse sign varies independently.
TARGET_VELOCITIES = [-0.60, -0.40, -0.20, 0.0, 0.20, 0.40, 0.60]  # [m/s]

# Pulse powers are fractions of full PWM, not controller Q. Q=1 today is only
# MOTOR_CORRECTION[0] of full PWM, so values above ~0.62 probe above current Q=1.
PULSE_POWERS = [-0.85, -0.60, -0.35, 0.35, 0.60, 0.85]
REPEATS = 3

PULSE_DURATION_S = 0.14          # short: enough samples, small velocity drift
TARGET_V_TOL = 0.04              # start pulse once measured velocity is close enough
TARGET_ZERO_V_TOL = 0.035
PREP_POWER = 0.78                # open-loop power used to build target velocity
ZERO_BRAKE_POWER = 0.20          # gentle dither/brake when preparing v_target=0
MAX_PREP_TIME_S = 2.0            # skip trial if target velocity is not reached
MAX_PREP_ATTEMPTS = 2            # then skip; prevents retry loops near boundaries
PAUSE_BEFORE_PULSE_S = 0.04      # one or two control cycles after crossing the target


# ---- track safety ---------------------------------------------------------
INNER_LIMIT = 0.035              # considered centred
CENTER_V_LIMIT = 0.08
TURN_SOFT = 0.135
HARD_LIMIT = 0.18
A_BRAKE = 5.0
TURN_MARGIN = 0.035
LATENCY_S = 0.04
V_CLIP = 3.0

RECOVERY_KP = 5.0                # power units per metre
RECOVERY_KD = 1.5                # power units per (m/s)
RECOVERY_POWER_MAX = 0.85
RECOVERY_TIMEOUT_S = 5.0


# ---- recalibration --------------------------------------------------------
RECALIBRATE_EVERY_N_TRIALS = 14
CALIBRATION_SAFETY_TIMEOUT_S = 30


class pulse_identification_experiment(template_experiment_protocol):
    def __init__(self, driver):
        super().__init__(
            driver=driver,
            experiment_protocol_name=self.__class__.__name__[:-len('_experiment')],
        )
        self.rng = None
        self.gain = None
        self.trials = []
        self.trial_idx = 0
        self.trials_since_calibration = 0
        self.time_state_changed = None
        self.pulse_start_time = None
        self.current_v_target = 0.0
        self.current_power = 0.0
        self.prep_attempts = 0

    def set_up_experiment(self, first_iteration=True):
        self.rng = np.random.default_rng(RNG_SEED)
        self.gain = MOTOR_CORRECTION[0]
        self.trials = [
            (float(v), float(p))
            for _ in range(REPEATS)
            for v in TARGET_VELOCITIES
            for p in PULSE_POWERS
        ]
        self.rng.shuffle(self.trials)
        self.trial_idx = 0
        self.trials_since_calibration = 0
        self.time_state_changed = None
        self.pulse_start_time = None
        self.current_v_target = 0.0
        self.current_power = 0.0
        self.prep_attempts = 0

        safe_frac = MOTOR_FULL_SCALE_SAFE / MOTOR_PWM_PERIOD_IN_CLOCK_CYCLES
        if max(abs(p) for p in PULSE_POWERS) > safe_frac:
            print(f'[pulse-id] WARNING: requested pulse exceeds safe fraction {safe_frac:.2f}; driver will clip.')

        self.current_experiment_phase = 'centering'
        self.Q = 0.0
        self.start_new_recording()
        print(f'[pulse-id] Prepared {len(self.trials)} velocity-conditioned pulses.')

    def update_state(self, angle: int, position: int, time_now: float):
        if self.current_experiment_phase == 'idle':
            pass
        elif self.current_experiment_phase == 'centering':
            self.action_centering(position, time_now)
        elif self.current_experiment_phase == 'preparing_velocity':
            self.action_preparing_velocity(position, time_now)
        elif self.current_experiment_phase == 'settling_before_pulse':
            self.action_settling_before_pulse(position, time_now)
        elif self.current_experiment_phase == 'pulse':
            self.action_pulse(position, time_now)
        elif self.current_experiment_phase == 'recovery':
            self.action_recovery(position, time_now)
        elif self.current_experiment_phase == 'calibrating':
            self.action_calibrating(time_now)
        else:
            raise Exception(f'unknown state {self.current_experiment_phase}')

        self.data_to_save_measurement = {'measurement': self}

    # -- motor-command helpers ---------------------------------------------
    def _q_for_power(self, power):
        if power == 0.0:
            return 0.0
        if CORRECT_MOTOR_DYNAMICS and self.gain:
            return power / self.gain
        return power

    def _velocity(self):
        try:
            v = float(self.driver.s[POSITIOND_IDX])
        except Exception:
            return 0.0
        return float(np.clip(v, -V_CLIP, V_CLIP))

    # -- safety / recovery --------------------------------------------------
    def _approaching_wall(self, position):
        v = self._velocity()
        stop_dist = (v * v) / (2.0 * A_BRAKE)
        stop_pos = position + v * LATENCY_S + np.sign(v) * stop_dist
        if stop_pos + TURN_MARGIN > TURN_SOFT or position > TURN_SOFT:
            return 1
        if stop_pos - TURN_MARGIN < -TURN_SOFT or position < -TURN_SOFT:
            return -1
        return 0

    def _recovery_q(self, position):
        v = self._velocity()
        power = -(RECOVERY_KP * position + RECOVERY_KD * v)
        power = float(np.clip(power, -RECOVERY_POWER_MAX, RECOVERY_POWER_MAX))
        return self._q_for_power(power)

    # -- trial sequencing ---------------------------------------------------
    def _load_current_trial(self):
        if self.trial_idx >= len(self.trials):
            print('[pulse-id] All pulses done.')
            self.stop()
            return False
        self.current_v_target, self.current_power = self.trials[self.trial_idx]
        return True

    def _start_preparing_velocity(self, time_now):
        if not self._load_current_trial():
            return
        self.current_experiment_phase = 'preparing_velocity'
        self.time_state_changed = time_now
        self.pulse_start_time = None
        self.prep_attempts += 1
        print(f'[pulse-id] Trial {self.trial_idx + 1}/{len(self.trials)}: '
              f'v_target={self.current_v_target:+.2f} m/s, pulse_power={self.current_power:+.2f} '
              f'(attempt {self.prep_attempts}/{MAX_PREP_ATTEMPTS})')

    def _complete_trial(self, time_now):
        self.trial_idx += 1
        self.trials_since_calibration += 1
        self.prep_attempts = 0
        self.Q = 0.0
        if self.trial_idx >= len(self.trials):
            print('[pulse-id] All pulses done.')
            self.stop()
        elif self.trials_since_calibration >= RECALIBRATE_EVERY_N_TRIALS:
            self._begin_recalibration()
        else:
            self.current_experiment_phase = 'recovery'
            self.time_state_changed = time_now

    def _skip_trial(self, time_now, reason):
        print(f'[pulse-id] Skipping trial {self.trial_idx + 1}/{len(self.trials)}: {reason}')
        self._complete_trial(time_now)

    # -- centering ----------------------------------------------------------
    def action_centering(self, position, time_now):
        if self.time_state_changed is None:
            self.time_state_changed = time_now
        if abs(position) < INNER_LIMIT and abs(self._velocity()) < CENTER_V_LIMIT:
            self.Q = 0.0
            self._start_preparing_velocity(time_now)
        else:
            self.Q = self._recovery_q(position)
            if time_now - self.time_state_changed > RECOVERY_TIMEOUT_S:
                print('[pulse-id] Centering timed out; continuing from current position.')
                self._start_preparing_velocity(time_now)

    # -- prepare target velocity -------------------------------------------
    def action_preparing_velocity(self, position, time_now):
        wall = self._approaching_wall(position)
        if wall != 0 or abs(position) > HARD_LIMIT:
            if self.prep_attempts >= MAX_PREP_ATTEMPTS:
                self._skip_trial(time_now, 'target velocity would violate boundary safety')
            else:
                self.current_experiment_phase = 'recovery'
                self.time_state_changed = time_now
                self.Q = self._recovery_q(position)
            return

        v = self._velocity()
        target = self.current_v_target
        if time_now - self.time_state_changed > MAX_PREP_TIME_S:
            if self.prep_attempts >= MAX_PREP_ATTEMPTS:
                self._skip_trial(time_now, 'target velocity not reached')
            else:
                self.current_experiment_phase = 'recovery'
                self.time_state_changed = time_now
                self.Q = self._recovery_q(position)
            return

        if abs(target) < TARGET_ZERO_V_TOL:
            if abs(v) < TARGET_ZERO_V_TOL and abs(position) < INNER_LIMIT:
                self.current_experiment_phase = 'settling_before_pulse'
                self.time_state_changed = time_now
                self.Q = 0.0
            else:
                # Damp velocity and pull gently toward centre.
                power = -(ZERO_BRAKE_POWER * np.sign(v) + RECOVERY_KP * position)
                power = float(np.clip(power, -RECOVERY_POWER_MAX, RECOVERY_POWER_MAX))
                self.Q = self._q_for_power(power)
            return

        if np.sign(target) * v >= abs(target) - TARGET_V_TOL:
            self.current_experiment_phase = 'settling_before_pulse'
            self.time_state_changed = time_now
            self.Q = 0.0
            return

        self.Q = self._q_for_power(np.sign(target) * PREP_POWER)

    def action_settling_before_pulse(self, position, time_now):
        if self._approaching_wall(position) != 0 or abs(position) > HARD_LIMIT:
            if self.prep_attempts >= MAX_PREP_ATTEMPTS:
                self._skip_trial(time_now, 'unsafe before pulse')
            else:
                self.current_experiment_phase = 'recovery'
                self.time_state_changed = time_now
                self.Q = self._recovery_q(position)
            return
        self.Q = 0.0
        if time_now - self.time_state_changed >= PAUSE_BEFORE_PULSE_S:
            self.current_experiment_phase = 'pulse'
            self.pulse_start_time = time_now
            self.Q = self._q_for_power(self.current_power)

    # -- pulse --------------------------------------------------------------
    def action_pulse(self, position, time_now):
        # If safety says we are too near the wall, end the pulse early. This trial can
        # simply be excluded by analysis if pulse_t is too short.
        if self._approaching_wall(position) != 0 or abs(position) > HARD_LIMIT:
            self._complete_trial(time_now)
            return

        self.Q = self._q_for_power(self.current_power)
        if time_now - self.pulse_start_time >= PULSE_DURATION_S:
            self._complete_trial(time_now)

    # -- recovery / recalibration ------------------------------------------
    def action_recovery(self, position, time_now):
        if self.time_state_changed is None:
            self.time_state_changed = time_now
        if abs(position) < INNER_LIMIT and abs(self._velocity()) < CENTER_V_LIMIT:
            self.Q = 0.0
            self._start_preparing_velocity(time_now)
        else:
            self.Q = self._recovery_q(position)
            if time_now - self.time_state_changed > RECOVERY_TIMEOUT_S:
                print('[pulse-id] Recovery timed out; attempting next pulse anyway.')
                self._start_preparing_velocity(time_now)

    def _begin_recalibration(self):
        self.Q = 0.0
        self.trials_since_calibration = 0
        self.driver.InterfaceInstance.set_motor(0)
        self.driver.InterfaceInstance.pc_control_mode(False)
        self.driver.InterfaceInstance.start_calibration()
        self.current_experiment_phase = 'calibrating'
        self.time_state_changed = None
        print('[pulse-id] Re-zeroing encoder between pulse batches...')

    def action_calibrating(self, time_now):
        if self.time_state_changed is None:
            self.time_state_changed = time_now
        self.Q = 0.0
        if not self.driver.InterfaceInstance.calibration_in_progress:
            self.driver.InterfaceInstance.pc_control_mode(True)
            print('[pulse-id] Recalibration done, resuming.')
            self.current_experiment_phase = 'centering'
            self.time_state_changed = None
        elif time_now - self.time_state_changed > CALIBRATION_SAFETY_TIMEOUT_S:
            print('[pulse-id] Calibration did not report back in time; stopping experiment.')
            self.stop()

    def __str__(self):
        pulse_t = 0.0
        try:
            if self.pulse_start_time is not None and self.driver.th is not None:
                pulse_t = float(self.driver.th.time_current_measurement - self.pulse_start_time)
        except Exception:
            pulse_t = 0.0
        moving = 'State:moving' if self.current_experiment_phase in (
            'preparing_velocity', 'settling_before_pulse', 'pulse', 'recovery'
        ) else 'State:idle'
        return (f' Pulse-ID ({moving}, Phase:{self.current_experiment_phase}, '
                f'Trial:{self.trial_idx}, v_target:{self.current_v_target:+.2f}, '
                f'power:{self.current_power:+.2f}, pulse_t:{pulse_t:.3f}, '
                f'Q:{self.Q}, v:{self._velocity():+.3f})')
