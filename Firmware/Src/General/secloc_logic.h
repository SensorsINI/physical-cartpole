#ifndef SECLOC_LOGIC_H
#define SECLOC_LOGIC_H

#include <stdint.h>

/*
 * Sparse Event-Based Closed-Loop Control (SecLoc) gate.
 *
 * Pure decision logic; Python counterpart is
 * Control_Toolkit_ASF/Controllers/secloc_logic.py (CartPoleSimulation), kept
 * in lockstep by tests/test_secloc_c_python_parity.py. Keep this file free of
 * firmware and controller dependencies so it stays compilable standalone.
 *
 * Decides whether a fresh controller computation is due, or whether the
 * previous control value should be held. Matches the Python SeclocLogic:
 *   - optional ref_period throttle: after an accepted update the gate is not
 *     consulted again until ref_period_ticks control loop iterations (ticks of
 *     time_quantum_s) have elapsed; 0 and 1 both mean the gate is consulted
 *     every iteration (ref_period_ticks > 0 requires time_quantum_s > 0),
 *   - angle and position are checked independently; either can fire,
 *   - the angle shift is measured from the active target equilibrium
 *     (|angle| for target up, pi - |angle| for target down),
 *   - on update only the reference of the axis (or axes) that fired
 *     is refreshed.
 */

typedef struct {
    float log_base;
    int32_t ref_period_ticks;
    float ang_dead_band;
    float pos_dead_band;
    float time_quantum_s;
} SeclocConfig;

typedef struct {
    float ang_last_shift;
    float pos_last_shift;
    float last_Q;
    uint8_t has_init;
    float time_last;
    int32_t tick_last;
} SeclocState;

void secloc_reset(SeclocState* state);

/* Angle distance from the active target equilibrium (te > 0: up, te < 0: down). */
float secloc_angle_shift_from_target(float a, float te);

/* 1 when the ref_period throttle allows a gate decision at this time (i.e. the
 * gate is actually consulted); 0 when the call would be throttled. Read-only. */
int secloc_gate_evaluated(
    const SeclocState* state,
    const SeclocConfig* config,
    float time
);

/* Returns 1 when a new control sample should be computed, 0 to hold last_Q. */
int secloc_should_sample(
    SeclocState* state,
    const SeclocConfig* config,
    float p,
    float pd,
    float a,
    float ad,
    float tp,
    float te,
    float time
);

#endif /* SECLOC_LOGIC_H */
